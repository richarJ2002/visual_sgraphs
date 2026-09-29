/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "Optimizer.h"

#include "OptimizableTypes.h"
#include "OptimizerEdgeLookup.h"
#include "Utils/Utils/objects/Utils.h"

#include "../private_functions.h"
#include "System.h"

namespace vs_graphs
{
namespace core
{

void Optimizer::bundleAdjustment(
    const std::vector<vs_graphs::core::KeyFrame *>          &keyFrames_in,
    const std::vector<vs_graphs::core::MapPoint *>          &mapPoints_in,
    const std::vector<vs_graphs::core::semantic::Marker *>  &markers_in,
    const std::vector<vs_graphs::core::geometric::Plane *>  &planes_in,
    const std::vector<vs_graphs::core::semantic::Passage *> &doorways_in,
    const std::vector<vs_graphs::core::semantic::Room *>    &rooms_in,
    const std::vector<vs_graphs::core::semantic::Floor *>   &floors_in,
    int                                                      iterationCount_in,
    bool                                                    *p_stopFlag_inout,
    const unsigned long                                      loopKeyFrameId_in,
    const bool                                               useRobustKernel_in,
    double                                                   markerImpact_in,
    const std::atomic_bool                                  *p_stopRequested_in)
{
    // System parameters
    vs_graphs::core::types::SystemParams *p_systemParams = nullptr;
    if (vs_graphs::core::types::SystemParams::getParams(p_systemParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }

    // Variables
    std::vector<bool> mapPointExcludedFlags;
    mapPointExcludedFlags.resize(mapPoints_in.size());

    if (keyFrames_in.empty())
        return;

    vs_graphs::core::Map *p_map = keyFrames_in[0]->getMap();

    AtomicOptimizerStopBridge stopBridge(p_stopRequested_in, p_stopFlag_inout);

    // Set up the solver
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver;
    p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *p_blockSolver = new g2o::BlockSolverX(p_linearSolver);
    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(p_blockSolver);
    optimizer.setAlgorithm(p_solver);
    optimizer.setVerbose(false);

    if (p_stopFlag_inout)
        optimizer.setForceStopFlag(p_stopFlag_inout);

    if (p_stopRequested_in != nullptr && p_stopFlag_inout != nullptr)
    {
        optimizer.addPreIterationAction(&stopBridge);
        optimizer.addPostIterationAction(&stopBridge);
    }

    long unsigned int maxKeyFrameId = 0;

    const int expectedEdgeCount = (keyFrames_in.size()) * mapPoints_in.size();

    vector<vs_graphs::core::EdgeSE3ProjectXYZ *> monoEdges;
    monoEdges.reserve(expectedEdgeCount);

    vector<vs_graphs::core::EdgeSE3ProjectXYZToBody *> bodyEdges;
    bodyEdges.reserve(expectedEdgeCount);

    vector<KeyFrame *> monoEdgeKeyFrames;
    monoEdgeKeyFrames.reserve(expectedEdgeCount);

    vector<KeyFrame *> bodyEdgeKeyFrames;
    bodyEdgeKeyFrames.reserve(expectedEdgeCount);

    vector<MapPoint *> monoEdgeMapPoints;
    monoEdgeMapPoints.reserve(expectedEdgeCount);

    vector<MapPoint *> bodyEdgeMapPoints;
    bodyEdgeMapPoints.reserve(expectedEdgeCount);

    vector<g2o::EdgeStereoSE3ProjectXYZ *> stereoEdges;
    stereoEdges.reserve(expectedEdgeCount);

    vector<KeyFrame *> stereoEdgeKeyFrames;
    stereoEdgeKeyFrames.reserve(expectedEdgeCount);

    vector<MapPoint *> stereoEdgeMapPoints;
    stereoEdgeMapPoints.reserve(expectedEdgeCount);

    // [GBA] KeyFrames
    for (size_t elementIndex = 0; elementIndex < keyFrames_in.size();
         elementIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames_in[elementIndex];
        if (p_keyFrame->isBad())
            continue;
        g2o::VertexSE3Expmap *p_keyFramePoseVertex = new g2o::VertexSE3Expmap();
        Sophus::SE3<float>    Tcw                  = p_keyFrame->getPose();
        p_keyFramePoseVertex->setEstimate(
            g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                         Tcw.translation().cast<double>()));
        p_keyFramePoseVertex->setId(p_keyFrame->id);
        p_keyFramePoseVertex->setFixed(p_keyFrame->id ==
                                       p_map->getInitKeyFrameId());
        optimizer.addVertex(p_keyFramePoseVertex);
        if (p_keyFrame->id > maxKeyFrameId)
            maxKeyFrameId = p_keyFrame->id;
    }

    const float huberThreshold1D = sqrt(3.841);
    const float huberThreshold2D = sqrt(5.99);
    const float huberThreshold3D = sqrt(7.815);

    int planeCount              = 1;
    int roomCount               = 1;
    int floorCount              = 1;
    int maxGlobalOptimizationId = 0;
    int markerCount             = 1;
    int doorwayCount            = 1;

    // [GBA] MapPoints
    for (size_t elementIndex = 0; elementIndex < mapPoints_in.size();
         elementIndex++)
    {
        MapPoint *p_mapPoint = mapPoints_in[elementIndex];
        if (p_mapPoint->isBad())
            continue;
        g2o::VertexSBAPointXYZ *p_mapPointVertex = new g2o::VertexSBAPointXYZ();
        p_mapPointVertex->setEstimate(p_mapPoint->getWorldPos().cast<double>());
        const int mapPointVertexId = p_mapPoint->id + maxKeyFrameId + 1;
        p_mapPointVertex->setId(mapPointVertexId);
        p_mapPointVertex->setMarginalized(true);
        optimizer.addVertex(p_mapPointVertex);

        // Update the maxOpId to hold the biggest value
        if (mapPointVertexId > maxGlobalOptimizationId)
            maxGlobalOptimizationId = mapPointVertexId;

        const map<KeyFrame *, tuple<int, int>> observations =
            p_mapPoint->getObservations();

        int mapPointEdgeCount = 0;
        // SET EDGES
        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                 featureObservationIt = observations.begin();
             featureObservationIt != observations.end();
             featureObservationIt++)
        {
            KeyFrame *p_keyFrame = featureObservationIt->first;
            if (p_keyFrame->isBad() || p_keyFrame->id > maxKeyFrameId)
                continue;
            if (optimizer.vertex(mapPointVertexId) == nullptr ||
                optimizer.vertex(p_keyFrame->id) == nullptr)
                continue;
            mapPointEdgeCount++;

            const int leftIndex = get<0>(featureObservationIt->second);

            if (leftIndex != -1 &&
                p_keyFrame->uRight[get<0>(featureObservationIt->second)] < 0)
            {
                const cv::KeyPoint &undistortedKeyPoint =
                    p_keyFrame->keyPointsUndistorted[leftIndex];

                Eigen::Matrix<double, 2, 1> observation;
                observation << undistortedKeyPoint.pt.x,
                    undistortedKeyPoint.pt.y;

                vs_graphs::core::EdgeSE3ProjectXYZ *p_edge =
                    new vs_graphs::core::EdgeSE3ProjectXYZ();

                p_edge->setVertex(0,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(mapPointVertexId)));
                p_edge->setVertex(1,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(p_keyFrame->id)));
                p_edge->setMeasurement(observation);
                const float &inverseSigmaSquared =
                    p_keyFrame
                        ->invLevelSigmaSquared[undistortedKeyPoint.octave];
                p_edge->setInformation(Eigen::Matrix2d::Identity() *
                                       inverseSigmaSquared);

                if (useRobustKernel_in)
                {
                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    p_edge->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(huberThreshold2D);
                }

                p_edge->p_camera = p_keyFrame->p_camera;

                optimizer.addEdge(p_edge);

                monoEdges.push_back(p_edge);
                monoEdgeKeyFrames.push_back(p_keyFrame);
                monoEdgeMapPoints.push_back(p_mapPoint);
            }
            else if (leftIndex != -1 &&
                     p_keyFrame->uRight[leftIndex] >= 0) // Stereo observation
            {
                const cv::KeyPoint &undistortedKeyPoint =
                    p_keyFrame->keyPointsUndistorted[leftIndex];

                Eigen::Matrix<double, 3, 1> observation;
                const float                 rightUCoordinate =
                    p_keyFrame->uRight[get<0>(featureObservationIt->second)];
                observation << undistortedKeyPoint.pt.x,
                    undistortedKeyPoint.pt.y, rightUCoordinate;

                g2o::EdgeStereoSE3ProjectXYZ *p_edge =
                    new g2o::EdgeStereoSE3ProjectXYZ();

                p_edge->setVertex(0,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(mapPointVertexId)));
                p_edge->setVertex(1,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(p_keyFrame->id)));
                p_edge->setMeasurement(observation);
                const float &inverseSigmaSquared =
                    p_keyFrame
                        ->invLevelSigmaSquared[undistortedKeyPoint.octave];
                Eigen::Matrix3d informationMatrix =
                    Eigen::Matrix3d::Identity() * inverseSigmaSquared;
                p_edge->setInformation(informationMatrix);

                if (useRobustKernel_in)
                {
                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    p_edge->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(huberThreshold3D);
                }

                p_edge->fx = p_keyFrame->fx;
                p_edge->fy = p_keyFrame->fy;
                p_edge->cx = p_keyFrame->cx;
                p_edge->cy = p_keyFrame->cy;
                p_edge->bf = p_keyFrame->mbf;

                optimizer.addEdge(p_edge);

                stereoEdges.push_back(p_edge);
                stereoEdgeKeyFrames.push_back(p_keyFrame);
                stereoEdgeMapPoints.push_back(p_mapPoint);
            }

            if (p_keyFrame->p_camera2)
            {
                int rightIndex = get<1>(featureObservationIt->second);

                if (rightIndex != -1 && static_cast<size_t>(rightIndex) <
                                            p_keyFrame->keyPointsRight.size())
                {
                    rightIndex -= p_keyFrame->leftKeyPointCount;

                    Eigen::Matrix<double, 2, 1> observation;
                    cv::KeyPoint                rightKeyPoint =
                        p_keyFrame->keyPointsRight[rightIndex];
                    observation << rightKeyPoint.pt.x, rightKeyPoint.pt.y;

                    vs_graphs::core::EdgeSE3ProjectXYZToBody *p_edge =
                        new vs_graphs::core::EdgeSE3ProjectXYZToBody();

                    p_edge->setVertex(
                        0,
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(mapPointVertexId)));
                    p_edge->setVertex(
                        1,
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(p_keyFrame->id)));
                    p_edge->setMeasurement(observation);
                    const float &inverseSigmaSquared =
                        p_keyFrame->invLevelSigmaSquared[rightKeyPoint.octave];
                    p_edge->setInformation(Eigen::Matrix2d::Identity() *
                                           inverseSigmaSquared);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    p_edge->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(huberThreshold2D);

                    Sophus::SE3f Trl = p_keyFrame->getRelativePoseTrl();
                    p_edge->mTrl =
                        g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                     Trl.translation().cast<double>());

                    p_edge->p_camera = p_keyFrame->p_camera2;

                    optimizer.addEdge(p_edge);
                    bodyEdges.push_back(p_edge);
                    bodyEdgeKeyFrames.push_back(p_keyFrame);
                    bodyEdgeMapPoints.push_back(p_mapPoint);
                }
            }
        }

        if (mapPointEdgeCount == 0)
        {
            optimizer.removeVertex(p_mapPointVertex);
            mapPointExcludedFlags[elementIndex] = true;
        }
        else
        {
            mapPointExcludedFlags[elementIndex] = false;
        }
    }

    // [GBA] Markers
    for (const auto &marker : markers_in)
    {
        // Adding a vertex for each marker
        g2o::VertexSE3Expmap *p_markerPoseVertex = new g2o::VertexSE3Expmap();
        Sophus::SE3f          markerGlobalPose{};
        if (marker->getGlobalPose(markerGlobalPose) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            // getGlobalPose cannot fail; continue as before.
        }
        Sophus::SE3f markerGlobalPose2{};
        if (marker->getGlobalPose(markerGlobalPose2) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            // getGlobalPose cannot fail; continue as before.
        }
        p_markerPoseVertex->setEstimate(
            g2o::SE3Quat(markerGlobalPose.unit_quaternion().cast<double>(),
                         markerGlobalPose2.translation().cast<double>()));
        int globalOptimizationId = maxGlobalOptimizationId + markerCount;
        p_markerPoseVertex->setId(globalOptimizationId);
        optimizer.addVertex(p_markerPoseVertex);
        markerCount++;

        // Setting the Global Optimization ID for the marker
        if (marker->setOpIdG(globalOptimizationId) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            // setOpIdG cannot fail; continue as before.
        }

        /*!
         * The edge used to connect a Marker vertex (SE3) to a KeyFrame vertex
         * (SE3) 🚧 [vS-Graphs v.2.0] This edge is not used anymore, in contrast
         * to the previous version. [Note]: it creates constraint for six
         * measurements, i.e., (x, y, z, roll, pitch, yaw)
         */
    }

    maxGlobalOptimizationId += markerCount;

    // [GBA] Planes
    for (const auto &plane : planes_in)
    {
        // Skip undefined planes (if not wall for now)
        geometric::Plane::PlaneVariant planeType{};
        if (plane->getPlaneType(planeType) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getPlaneType cannot fail; continue as before.
        }
        if (planeType == geometric::Plane::PlaneVariant::UNDEFINED)
            continue;
        // Adding a vertex for each plane
        g2o::VertexPlane *p_planeVertex = new g2o::VertexPlane();
        int globalOptimizationId        = maxGlobalOptimizationId + planeCount;
        p_planeVertex->setId(globalOptimizationId);
        g2o::Plane3D planeGetGlobalEquation{};
        if (plane->getGlobalEquation(planeGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getGlobalEquation cannot fail; continue as before.
        }
        p_planeVertex->setEstimate(planeGetGlobalEquation);
        if (p_systemParams->optimization.shouldMarginalizePlanes)
            p_planeVertex->setMarginalized(true);
        optimizer.addVertex(p_planeVertex);
        planeCount++;

        // Setting the global optimization ID for the plane
        if (plane->setOpIdG(globalOptimizationId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // setOpIdG cannot fail; continue as before.
        }

        // Adding an edge between the plane and the keyframes
        map<KeyFrame *, vs_graphs::core::geometric::Plane::Observation>
            observations{};
        if (plane->getObservations(observations) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getObservations cannot fail; continue as before.
        }
        for (map<KeyFrame *,
                 vs_graphs::core::geometric::Plane::Observation>::const_iterator
                 planeObservationIt  = observations.begin(),
                 planeObservationEnd = observations.end();
             planeObservationIt != planeObservationEnd;
             planeObservationIt++)
        {
            KeyFrame *p_observingKeyFrame = planeObservationIt->first;
            vs_graphs::core::geometric::Plane::Observation observation =
                planeObservationIt->second;

            if (p_observingKeyFrame->isBad())
            {
                std::cout
                    << "[Optimizer] Bad KeyFrame detected for GBA! Skipping..."
                    << std::endl;
                if (plane->eraseObservation(p_observingKeyFrame) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // eraseObservation cannot fail; continue as before.
                }
                continue;
            }

            g2o::Plane3D planeLocalEquation = observation.localPlane;

            if (optimizer.vertex(globalOptimizationId) &&
                optimizer.vertex(p_observingKeyFrame->id))
            {
                if (p_systemParams->optimization.planeKf.enabled)
                {
                    vs_graphs::core::EdgeVertexPlaneProjectSE3KF *p_edge =
                        new vs_graphs::core::EdgeVertexPlaneProjectSE3KF();
                    p_edge->setVertex(
                        0,
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(p_observingKeyFrame->id)));
                    p_edge->setVertex(
                        1,
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(globalOptimizationId)));
                    p_edge->setInformation(
                        Eigen::Matrix<double, 3, 3>::Identity() *
                        observation.confidence *
                        p_systemParams->optimization.planeKf.informationGain);
                    p_edge->setMeasurement(planeLocalEquation);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    p_edge->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(huberThreshold3D);
                    optimizer.addEdge(p_edge);
                }

                // adding plane-point constraints
                if (p_systemParams->optimization.planePoint.enabled)
                {
                    // get the class index of the plane
                    int                            planeClassIndex{};
                    geometric::Plane::PlaneVariant planeType2{};
                    if (plane->getPlaneType(planeType2) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getPlaneType cannot fail; continue as before.
                    }
                    if (utils::utils::Utils::getClassIdFromPlaneType(
                            planeType2,
                            planeClassIndex) !=
                        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                    {
                        // getClassIdFromPlaneType cannot fail; continue as
                        // before.
                    }
                    if (planeClassIndex != -1)
                    {
                        // add the plane-point constraint
                        vs_graphs::core::EdgeSE3KFPointToPlane *p_edge =
                            new vs_graphs::core::EdgeSE3KFPointToPlane();
                        p_edge->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(p_observingKeyFrame->id)));
                        p_edge->setVertex(
                            1,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(globalOptimizationId)));
                        p_edge->setInformation(
                            Eigen::Matrix<double, 1, 1>::Identity() *
                            observation.confidence *
                            p_systemParams->optimization.planePoint
                                .informationGain);
                        p_edge->setMeasurement(
                            observation.pointPlaneConstraintMatrix);

                        g2o::RobustKernelHuber *p_robustKernel =
                            new g2o::RobustKernelHuber;
                        p_edge->setRobustKernel(p_robustKernel);
                        p_robustKernel->setDelta(huberThreshold1D);
                        optimizer.addEdge(p_edge);
                    }
                }
            }
        }

        // 🚧 [vS-Graphs v.2.0] in contrast with the first version of visual
        // S-Graphs, where there was an edge between Markers and Planes, in this
        // version we removed that edge vector<Marker *> attachedMarkers =
        // vpPlane->getMarkers(); for (const auto &planeMarker :
        // attachedMarkers)
        // {
        //     // Adding an edge between the Plane and the Marker
        //     vs_graphs::core::EdgeVertexPlaneProjectSE3M *e = new
        //     vs_graphs::core::EdgeVertexPlaneProjectSE3M(); e->setVertex(1,
        //     dynamic_cast<g2o::OptimizableGraph::Vertex
        //     *>(optimizer.vertex(opIdG))); e->setVertex(0,
        //     dynamic_cast<g2o::OptimizableGraph::Vertex
        //     *>(optimizer.vertex(planeMarker->getOpIdG())));
        //     e->setInformation(Eigen::Matrix<double, 4, 4>::Identity());

        //     g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        //     e->setRobustKernel(rk);
        //     rk->setDelta(thHuber2D);
        //     optimizer.addEdge(e);
        // }
    }

    maxGlobalOptimizationId += planeCount;

    // [GBA] Rooms
    for (const auto &room : rooms_in)
    {
        try
        {
            // Variables
            std::vector<vs_graphs::core::geometric::Plane *> walls{};
            if (room->getWalls(walls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWalls cannot fail; continue as before.
            }

            // No need to optimize if there are no walls
            if (walls.empty())
                continue;

            // Adding a vertex for each room
            g2o::VertexSE3Expmap *p_roomPoseVertex = new g2o::VertexSE3Expmap();

            // Setting the local optimization ID for the room
            int globalOptimizationId = maxGlobalOptimizationId + roomCount;
            p_roomPoseVertex->setId(globalOptimizationId);
            if (room->setOpIdG(globalOptimizationId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setOpIdG cannot fail; continue as before.
            }
            roomCount++;

            // Initialize the room vertex (centroid estimate)
            Eigen::Vector3d roomCentroid{};
            if (room->getCentroid(roomCentroid) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            p_roomPoseVertex->setEstimate(
                g2o::SE3Quat(Eigen::Quaterniond::Identity(),
                             roomCentroid.cast<double>()));
            p_roomPoseVertex->setFixed(true);
            optimizer.addVertex(p_roomPoseVertex);

            /*
             * The legacy multi-plane room projection factor averages closest
             * points from every wall. That estimate is not the center of a
             * bounded room and can pull otherwise valid wall planes during a
             * map re-merge. Keep the front-end room centroid fixed and retain
             * only the well-defined plane parallel/perpendicular constraints
             * below.
             */

            // Optimizing the parallel walls of the room
            for (size_t elementIndex = 0; elementIndex < walls.size();
                 elementIndex++)
                for (size_t secondElementIndex = elementIndex + 1;
                     secondElementIndex < walls.size();
                     secondElementIndex++)
                {
                    vs_graphs::core::geometric::Plane *p_firstWall =
                        walls[elementIndex];
                    vs_graphs::core::geometric::Plane *p_secondWall =
                        walls[secondElementIndex];

                    // If the same wall, skip
                    int firstWallGetId{};
                    if (p_firstWall->getId(firstWallGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int secondWallGetId{};
                    if (p_secondWall->getId(secondWallGetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    if (firstWallGetId == secondWallGetId)
                        continue;

                    // Check if the walls are parallel
                    bool arePlanesParallel2{};
                    if (utils::utils::Utils::arePlanesParallel(
                            p_firstWall,
                            p_secondWall,
                            arePlanesParallel2) !=
                        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                    {
                        // arePlanesParallel cannot fail; continue as before.
                    }
                    if (arePlanesParallel2)
                    {
                        // If they are parallel, check if they are facing each
                        // other
                        bool arePlanesFacingEachOther2{};
                        if (utils::utils::Utils::arePlanesFacingEachOther(
                                p_firstWall,
                                p_secondWall,
                                arePlanesFacingEachOther2) !=
                            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                        {
                            // arePlanesFacingEachOther cannot fail; continue as
                            // before.
                        }
                        if (arePlanesFacingEachOther2)
                        {
                            // Variables
                            int firstWallOptimizationId{};
                            if (p_firstWall->getOpIdG(
                                    firstWallOptimizationId) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                            {
                                // getOpIdG cannot fail; continue as before.
                            }
                            int secondWallOptimizationId{};
                            if (p_secondWall->getOpIdG(
                                    secondWallOptimizationId) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                            {
                                // getOpIdG cannot fail; continue as before.
                            }

                            if (optimizer.vertex(globalOptimizationId) &&
                                optimizer.vertex(firstWallOptimizationId) &&
                                optimizer.vertex(secondWallOptimizationId))
                            {
                                // std::cout << "[Optimizer] Parallelism
                                // constraint between walls "
                                //           << wall1->getId() << " and " <<
                                //           wall2->getId() << " of room " <<
                                //           pMapRoom->getId() << std::endl;

                                vs_graphs::core::EdgeVertexPlaneParallelism
                                    *p_edge = new vs_graphs::core::
                                        EdgeVertexPlaneParallelism();
                                p_edge->setVertex(
                                    0,
                                    dynamic_cast<g2o::OptimizableGraph::Vertex
                                                     *>(optimizer.vertex(
                                        firstWallOptimizationId)));
                                p_edge->setVertex(
                                    1,
                                    dynamic_cast<g2o::OptimizableGraph::Vertex
                                                     *>(optimizer.vertex(
                                        secondWallOptimizationId)));
                                p_edge->setMeasurement(
                                    0.0); // We want the angle between the
                                          // planes to be 0

                                // Information matrix
                                p_edge->setInformation(
                                    Eigen::Matrix<double, 1, 1>::Identity() *
                                    1e3);

                                // Adding the edge to the optimizer
                                g2o::RobustKernelHuber *p_robustKernel =
                                    new g2o::RobustKernelHuber;
                                p_edge->setRobustKernel(p_robustKernel);
                                p_robustKernel->setDelta(huberThreshold1D);
                                optimizer.addEdge(p_edge);
                            }
                        }
                    }

                    // Check if the walls are perpendicular
                    bool arePlanesPerpendicular2{};
                    if (utils::utils::Utils::arePlanesPerpendicular(
                            p_firstWall,
                            p_secondWall,
                            arePlanesPerpendicular2) !=
                        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                    {
                        // arePlanesPerpendicular cannot fail; continue as
                        // before.
                    }
                    if (arePlanesPerpendicular2)
                    {
                        // Variables
                        int firstWallOptimizationId{};
                        if (p_firstWall->getOpIdG(firstWallOptimizationId) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getOpIdG cannot fail; continue as before.
                        }
                        int secondWallOptimizationId{};
                        if (p_secondWall->getOpIdG(secondWallOptimizationId) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getOpIdG cannot fail; continue as before.
                        }

                        if (optimizer.vertex(globalOptimizationId) &&
                            optimizer.vertex(firstWallOptimizationId) &&
                            optimizer.vertex(secondWallOptimizationId))
                        {
                            // std::cout << "[Optimizer] Perpendicularity
                            // constraint between walls "
                            //           << wall1->getId() << " and " <<
                            //           wall2->getId() << " of room " <<
                            //           pMapRoom->getId() << std::endl;

                            vs_graphs::core::EdgeVertexPlanePerpendicularity
                                *p_edge = new vs_graphs::core::
                                    EdgeVertexPlanePerpendicularity();
                            p_edge->setVertex(
                                0,
                                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex(firstWallOptimizationId)));
                            p_edge->setVertex(
                                1,
                                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex(
                                        secondWallOptimizationId)));
                            p_edge->setMeasurement(
                                M_PI_2); // We want the angle between the planes
                                         // to be 90 degrees

                            // Information matrix
                            p_edge->setInformation(
                                Eigen::Matrix<double, 1, 1>::Identity() * 1e3);

                            // Adding the edge to the optimizer
                            g2o::RobustKernelHuber *p_robustKernel =
                                new g2o::RobustKernelHuber;
                            p_edge->setRobustKernel(p_robustKernel);
                            p_robustKernel->setDelta(huberThreshold1D);
                            optimizer.addEdge(p_edge);
                        }
                    }
                }
        }
        catch (std::exception &p_edge)
        {
            std::cerr << "[Optimizer] Error while globally optimizing room: "
                      << p_edge.what() << std::endl;
            continue;
        }
    }

    maxGlobalOptimizationId += roomCount;

    // [GBA] Floors
    // 🚧 Temporarily disabled to be synced with LBA: We moved the floor
    // optimization to the the front-end (SemanticsManager) as some rooms may
    // not be available in the local optimizer, causing wrong floor optimization
    // for (const auto &pMapFloor : vpFloors)
    // {
    //     try
    //     {
    //         // Variables
    //         bool allRoomsExist = true;
    //         std::vector<vs_graphs::core::semantic::Room *> rooms =
    //         pMapFloor->getRooms();

    //         // No need to optimize if there are no rooms
    //         if (rooms.empty())
    //             continue;

    //         // Adding a vertex for each floor
    //         g2o::VertexSE3Expmap *vrtxFloor = new g2o::VertexSE3Expmap();

    //         // Setting the local optimization ID for the floor
    //         int opIdG = maxOpId + nFloors;
    //         vrtxFloor->setId(opIdG);
    //         pMapFloor->setOpIdG(opIdG);
    //         nFloors++;

    //         // Initialize the floor vertex (centroid estimate)
    //         vrtxFloor->setEstimate(g2o::SE3Quat(Eigen::Quaterniond::Identity(),
    //                                             pMapFloor->getCentroid().cast<double>()));
    //         optimizer.addVertex(vrtxFloor);

    //         // Getting the rooms opIds
    //         std::vector<int> floorRoomOpIdGs;
    //         for (const auto &room : rooms)
    //             floorRoomOpIdGs.push_back(room->getOpIdG());

    //         for (int roomOpId : floorRoomOpIdGs)
    //         {
    //             if (!optimizer.vertex(roomOpId))
    //             {
    //                 allRoomsExist = false;
    //                 std::cerr << "[Optimizer] Room vertex with opId=" <<
    //                 roomOpId << " not found globally, skipping floor-room
    //                 edge." << std::endl; break;
    //             }
    //         }

    //         // Adding a single edge connecting the floor to all its rooms
    //         if (optimizer.vertex(opIdG) && allRoomsExist &&
    //         floorRoomOpIdGs.size() >= 1)
    //         {
    //             vs_graphs::core::EdgeVertexNSE3RoomProjectSE3Floor *e = new
    //             vs_graphs::core::EdgeVertexNSE3RoomProjectSE3Floor();

    //             // Resize edge to fit: 1 (floor vertex) + n (rooms)
    //             e->resize(1 + floorRoomOpIdGs.size());

    //             // Set floor vertex
    //             e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex
    //             *>(optimizer.vertex(opIdG)));

    //             // Set room vertices
    //             for (size_t i = 0; i < floorRoomOpIdGs.size(); i++)
    //                 if (optimizer.vertex(floorRoomOpIdGs[i]))
    //                     e->setVertex(i + 1,
    //                     dynamic_cast<g2o::OptimizableGraph::Vertex
    //                     *>(optimizer.vertex(floorRoomOpIdGs[i])));

    //             // Information matrix
    //             e->setInformation(Eigen::Matrix<double, 3, 3>::Identity());

    //             // Adding the edge to the optimizer
    //             g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
    //             e->setRobustKernel(rk);
    //             rk->setDelta(thHuber2D);
    //             optimizer.addEdge(e);
    //         }
    //     }
    //     catch (std::exception &e)
    //     {
    //         std::cerr << "[Optimizer] Error while globally optimizing floor:
    //         " << e.what() << std::endl; continue;
    //     }
    // }

    // maxOpId += nFloors;

    // Optimize!
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(iterationCount_in);
    optimizer.removePreIterationAction(&stopBridge);
    optimizer.removePostIterationAction(&stopBridge);
    Verbose::printMess("BA: End of the optimization",
                       Verbose::VERBOSITY_NORMAL);

    // [GBA] Globally optimized KeyFrames
    for (size_t elementIndex = 0; elementIndex < keyFrames_in.size();
         elementIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames_in[elementIndex];
        if (p_keyFrame->isBad())
            continue;
        g2o::VertexSE3Expmap *p_keyFramePoseVertex =
            static_cast<g2o::VertexSE3Expmap *>(
                optimizer.vertex(p_keyFrame->id));

        g2o::SE3Quat optimizedPoseQuat = p_keyFramePoseVertex->estimate();
        if (p_map->getOriginKeyFrame() &&
            loopKeyFrameId_in == p_map->getOriginKeyFrame()->id)
        {
            p_keyFrame->setPose(
                Sophus::SE3f(optimizedPoseQuat.rotation().cast<float>(),
                             optimizedPoseQuat.translation().cast<float>()));
        }
        else
        {
            p_keyFrame->tcwGBA = Sophus::SE3d(optimizedPoseQuat.rotation(),
                                              optimizedPoseQuat.translation())
                                     .cast<float>();
            p_keyFrame->baGlobalKeyFrameId = loopKeyFrameId_in;

            Sophus::SE3f    mTwc     = p_keyFrame->getPoseInverse();
            Sophus::SE3f    mTcGBA_c = p_keyFrame->tcwGBA * mTwc;
            Eigen::Vector3f poseCorrectionTranslation = mTcGBA_c.translation();
            double poseCorrectionDistance = poseCorrectionTranslation.norm();
            if (poseCorrectionDistance > 1)
            {
                int monoBadPointCount = 0, monoOptimizedPointCount = 0;
                int stereoBadPointCount = 0, stereoOptimizedPointCount = 0;
                vector<MapPoint *> monoOptimizedMapPoints,
                    stereoOptimizedMapPoints;

                for (size_t edgeIndex = 0, edgeCount = monoEdges.size();
                     edgeIndex < edgeCount;
                     edgeIndex++)
                {
                    vs_graphs::core::EdgeSE3ProjectXYZ *p_edge =
                        monoEdges[edgeIndex];
                    MapPoint *p_mapPoint = monoEdgeMapPoints[edgeIndex];
                    KeyFrame *p_edgeSourceKeyFrame = nullptr;
                    if (edgeSourceKeyFrame(monoEdgeKeyFrames,
                                           edgeIndex,
                                           p_edgeSourceKeyFrame) !=
                        OptimizerEdgeLookupStatus::
                            OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS)
                    {
                        // edgeSourceKeyFrame cannot fail; continue as before.
                    }

                    if (p_edgeSourceKeyFrame == nullptr ||
                        p_keyFrame != p_edgeSourceKeyFrame)
                    {
                        continue;
                    }

                    if (p_mapPoint->isBad())
                        continue;

                    if (p_edge->chi2() > 5.991 || !p_edge->isDepthPositive())
                    {
                        monoBadPointCount++;
                    }
                    else
                    {
                        monoOptimizedPointCount++;
                        monoOptimizedMapPoints.push_back(p_mapPoint);
                    }
                }

                for (size_t edgeIndex = 0, edgeCount = stereoEdges.size();
                     edgeIndex < edgeCount;
                     edgeIndex++)
                {
                    g2o::EdgeStereoSE3ProjectXYZ *p_edge =
                        stereoEdges[edgeIndex];
                    MapPoint *p_mapPoint = stereoEdgeMapPoints[edgeIndex];
                    KeyFrame *p_edgeSourceKeyFrame = nullptr;
                    if (edgeSourceKeyFrame(stereoEdgeKeyFrames,
                                           edgeIndex,
                                           p_edgeSourceKeyFrame) !=
                        OptimizerEdgeLookupStatus::
                            OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS)
                    {
                        // edgeSourceKeyFrame cannot fail; continue as before.
                    }

                    if (p_edgeSourceKeyFrame == nullptr ||
                        p_keyFrame != p_edgeSourceKeyFrame)
                    {
                        continue;
                    }

                    if (p_mapPoint->isBad())
                        continue;

                    if (p_edge->chi2() > 7.815 || !p_edge->isDepthPositive())
                    {
                        stereoBadPointCount++;
                    }
                    else
                    {
                        stereoOptimizedPointCount++;
                        stereoOptimizedMapPoints.push_back(p_mapPoint);
                    }
                }
            }
        }
    }

    // [GBA] Globally optimized MapPoints
    for (size_t elementIndex = 0; elementIndex < mapPoints_in.size();
         elementIndex++)
    {
        if (mapPointExcludedFlags[elementIndex])
            continue;

        MapPoint *p_mapPoint = mapPoints_in[elementIndex];

        if (p_mapPoint->isBad())
            continue;
        g2o::VertexSBAPointXYZ *p_mapPointVertex =
            static_cast<g2o::VertexSBAPointXYZ *>(
                optimizer.vertex(p_mapPoint->id + maxKeyFrameId + 1));

        if (loopKeyFrameId_in == p_map->getOriginKeyFrame()->id)
        {
            p_mapPoint->setWorldPos(p_mapPointVertex->estimate().cast<float>());
            p_mapPoint->updateNormalAndDepth();
        }
        else
        {
            p_mapPoint->posGBA = p_mapPointVertex->estimate().cast<float>();
            p_mapPoint->baGlobalKeyFrameId = loopKeyFrameId_in;
        }
    }

    /*
     * Apply semantic vertices immediately only for the initial-map BA. A
     * loop-triggered GBA runs on a worker and must not mutate semantic state
     * before LoopClosing validates its generation and acquires the semantic
     * transaction lock. The validated post-GBA deformation pass updates
     * markers and rooms for that case.
     */
    if (loopKeyFrameId_in == p_map->getOriginKeyFrame()->id)
    {
        // [GBA] Globally optimized markers
        for (semantic::Marker *p_marker : markers_in)
        {
            int markerOpIdG{};
            if (p_marker->getOpIdG(markerOpIdG) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // getOpIdG cannot fail; continue as before.
            }
            g2o::VertexSE3Expmap *p_markerVertex =
                static_cast<g2o::VertexSE3Expmap *>(
                    optimizer.vertex(markerOpIdG));

            if (p_markerVertex == nullptr)
            {
                continue;
            }

            const g2o::SE3Quat markerPose_MarkerToWorld =
                p_markerVertex->estimate();

            if (p_marker->setGlobalPose(Sophus::SE3f(
                    markerPose_MarkerToWorld.rotation().cast<float>(),
                    markerPose_MarkerToWorld.translation().cast<float>())) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // setGlobalPose cannot fail; continue as before.
            }
        }
    }

    // [GBA] Globally optimized planes
    for (auto &plane : planes_in)
    {
        int planeGetOpIdG{};
        if (plane->getOpIdG(planeGetOpIdG) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getOpIdG cannot fail; continue as before.
        }
        if (optimizer.vertex(planeGetOpIdG))
        {
            int planeGetOpIdG2{};
            if (plane->getOpIdG(planeGetOpIdG2) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getOpIdG cannot fail; continue as before.
            }
            g2o::VertexPlane *p_planeVertex = static_cast<g2o::VertexPlane *>(
                optimizer.vertex(planeGetOpIdG2));

            if (loopKeyFrameId_in == p_map->getOriginKeyFrame()->id)
            {
                /*
                 * Keep the finite wall cloud, centroid, bounds, octree, and
                 * optimized equation in one frame. Updating only the equation
                 * leaves the displayed wall and every geometric association at
                 * the pre-BA pose.
                 */
                if (plane->alignGeometryToEquation(p_planeVertex->estimate()) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // alignGeometryToEquation cannot fail; continue as before.
                }
            }
            else
            {
                plane->planeGBA           = p_planeVertex->estimate();
                plane->baGlobalKeyFrameId = loopKeyFrameId_in;
            }
        }
    }

    // [GBA] Globally optimized rooms
    if (loopKeyFrameId_in == p_map->getOriginKeyFrame()->id)
    {
        for (semantic::Room *p_room : rooms_in)
        {
            try
            {
                int roomOpIdG{};
                if (p_room->getOpIdG(roomOpIdG) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getOpIdG cannot fail; continue as before.
                }
                g2o::VertexSE3Expmap *p_roomVertex =
                    static_cast<g2o::VertexSE3Expmap *>(
                        optimizer.vertex(roomOpIdG));

                if (p_roomVertex != nullptr)
                {
                    if (p_room->setCentroid(
                            p_roomVertex->estimate().translation()) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // setCentroid cannot fail; continue as before.
                    }
                }
            }
            catch (const std::exception &caughtException)
            {
                std::cerr << "[Optimizer] Error while updating optimized room: "
                          << caughtException.what() << std::endl;
            }
        }
    }

    // [GBA] Globally optimized floors
    // 🚧 Temporarily disabled to be synced with LBA: We moved the floor
    // optimization to the the front-end (SemanticsManager) as some rooms may
    // not be available in the local optimizer, causing wrong floor optimization
    // for (const auto &vpFloor : vpFloors)
    // {
    //     try
    //     {
    //         // Variables
    //         int opIdG = vpFloor->getOpIdG();

    //         // If the opId is invalid, skip
    //         if (opIdG < 0)
    //             continue;

    //         g2o::OptimizableGraph::Vertex *vBase = optimizer.vertex(opIdG);
    //         if (!vBase)
    //         {
    //             std::cerr << "[Warning] Floor vertex with opIdG='" << opIdG
    //             << "' not found in optimizer!" << std::endl; continue;
    //         }

    //         // Safe downcast
    //         g2o::VertexSE3Expmap *vrtxFloor =
    //         dynamic_cast<g2o::VertexSE3Expmap *>(vBase); if (!vrtxFloor)
    //         {
    //             std::cerr << "[Warning] Vertex Floor with opIdG='" << opIdG
    //             << "' is not a VertexSE3Expmap!" << std::endl; continue;
    //         }

    //         vpFloor->setCentroid(vrtxFloor->estimate().translation());
    //     }
    //     catch (std::exception &e)
    //     {
    //         std::cerr << "[Optimizer] Error while locally updating optimized
    //         floor: " << e.what() << std::endl; continue;
    //     }
    // }
}

} // namespace core
} // namespace vs_graphs
