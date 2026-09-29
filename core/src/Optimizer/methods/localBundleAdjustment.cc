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
#include "System.h"
#include "Utils/Utils/objects/Utils.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void Optimizer::localBundleAdjustment(
    vs_graphs::core::KeyFrame *p_keyFrame_inout,
    bool                      *p_pbStopFlag_in,
    Map                       *p_map_inout,
    int                       &fixedKeyFrameCount_inout,
    int                       &optKeyFrameCount_out,
    int                       &mapPointCount_out,
    int                       &edgeCount_out,
    double                     markerImpact_in)
{
    // System parameters
    vs_graphs::core::types::SystemParams *p_sysParams = nullptr;
    if (vs_graphs::core::types::SystemParams::getParams(p_sysParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }

    // Variables
    fixedKeyFrameCount_inout = 0;
    optKeyFrameCount_out     = 0;
    mapPointCount_out        = 0;
    edgeCount_out            = 0;
    std::list<vs_graphs::core::semantic::Room *> localRoomList;
    vs_graphs::core::Map *p_currentMap = p_keyFrame_inout->getMap();
    std::list<vs_graphs::core::geometric::Plane *>  localPlaneList;
    std::list<vs_graphs::core::semantic::Marker *>  localMarkerList;
    std::list<vs_graphs::core::semantic::Passage *> localPassageList;
    std::list<vs_graphs::core::KeyFrame *>          localKeyFrameList;
    std::list<vs_graphs::core::MapPoint *>          localMapPointList;
    std::vector<vs_graphs::core::KeyFrame *>        neighborKeyFrameVector;
    std::vector<vs_graphs::core::semantic::Room *>  allRooms =
        p_currentMap->getAllRooms();
    std::vector<vs_graphs::core::semantic::Floor *> allFloors =
        p_currentMap->getAllFloors();

    // Unorderd maps to keep track of the local entities
    std::unordered_map<int, bool> localPlaneId;
    std::unordered_map<int, bool> localMarkerId;
    std::unordered_map<int, bool> localDoorwayId;
    std::unordered_map<int, bool> localMapPointId;
    std::unordered_map<int, bool> localKeyFrameId;

    // [LBA] Initialize the KeyFrame-related variables
    localKeyFrameList.push_back(p_keyFrame_inout);
    localKeyFrameId[p_keyFrame_inout->id] = true;
    p_keyFrame_inout->baLocalKeyFrameId   = p_keyFrame_inout->id;

    // [LBA] Fill in the neighbor KeyFrames
    if (p_sysParams->planeBasedCovisibility.enabled)
    {
        // Get the KeyFrames that see the same planes
        neighborKeyFrameVector = p_keyFrame_inout->getBestCovisibilityKeyFrames(
            p_sysParams->planeBasedCovisibility.maxKeyframes);
    }
    else
    {
        // Get the KeyFrames that see the same MapPoints
        neighborKeyFrameVector =
            p_keyFrame_inout->getVectorCovisibleKeyFrames();
    }

    // Iterate through all neighboring KeyFrames
    for (int markerIt = 0, indexEnd = neighborKeyFrameVector.size();
         markerIt < indexEnd;
         markerIt++)
    {
        // Get the current KeyFrame's neighbors
        vs_graphs::core::KeyFrame *p_keyFrame =
            neighborKeyFrameVector[markerIt];
        // Mark the KeyFrame as a part of the current LBA
        p_keyFrame->baLocalKeyFrameId = p_keyFrame_inout->id;
        // If the KeyFrame is proper, add it to the list of local KeyFrames for
        // LBA
        if (!p_keyFrame->isBad() && p_keyFrame->getMap() == p_currentMap)
        {
            localKeyFrameList.push_back(p_keyFrame);
            localKeyFrameId[p_keyFrame->id] = true;
        }
    }

    // [LBA] Loop through the local KeyFrames
    for (std::list<vs_graphs::core::KeyFrame *>::iterator
             lit  = localKeyFrameList.begin(),
             lend = localKeyFrameList.end();
         lit != lend;
         lit++)
    {
        // Variables
        vs_graphs::core::KeyFrame                       *p_keyFrame = *lit;
        std::vector<vs_graphs::core::geometric::Plane *> localPlanesVector =
            p_keyFrame->getMapPlanes();
        std::vector<vs_graphs::core::semantic::Marker *> localMarkersVector =
            p_keyFrame->getMapMarkers();
        std::vector<vs_graphs::core::semantic::Passage *> localDoorwaysVector =
            p_keyFrame->getMapPassages();
        std::vector<vs_graphs::core::MapPoint *> localMapPointsVector =
            p_keyFrame->getMapPointMatches();

        // If the KeyFrame is the initial KeyFrame of the map, mark that as a
        // fixed KeyFrame
        if (p_keyFrame->id == p_map_inout->getInitKeyFrameId())
        {
            fixedKeyFrameCount_inout = 1;
        }

        // [LBA] Loop through all the MapPoints and prepare them for LBA
        for (std::vector<vs_graphs::core::MapPoint *>::iterator
                 vit  = localMapPointsVector.begin(),
                 vend = localMapPointsVector.end();
             vit != vend;
             vit++)
        {
            // Variables
            vs_graphs::core::MapPoint *p_mapPoint = *vit;

            // If the MapPoint is proper, add it to the list of local MapPoints
            // for LBA
            if (p_mapPoint)
            {
                if (!p_mapPoint->isBad() &&
                    p_mapPoint->getMap() == p_currentMap)
                {
                    if (p_mapPoint->baLocalKeyFrameId != p_keyFrame_inout->id)
                    {
                        localMapPointList.push_back(p_mapPoint);
                        localMapPointId[p_mapPoint->id] = true;
                        p_mapPoint->baLocalKeyFrameId   = p_keyFrame_inout->id;
                    }
                }
            }
        }

        // [LBA] Loop through all the Markers and prepare them for LBA
        for (std::vector<vs_graphs::core::semantic::Marker *>::iterator
                 markerIt = localMarkersVector.begin(),
                 vend     = localMarkersVector.end();
             markerIt != vend;
             markerIt++)
        {
            int id2{};
            if ((*markerIt)->getId(id2) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (localMarkerId.find(id2) == localMarkerId.end())
            {
                vs_graphs::core::semantic::Marker *p_marker = *markerIt;
                localMarkerList.push_back(p_marker);
                int markerId{};
                if (p_marker->getId(markerId) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                localMarkerId[markerId] = true;
            }
        }

        // [LBA] Loop through all the Planes and prepare them for LBA
        for (std::vector<vs_graphs::core::geometric::Plane *>::iterator
                 markerIt = localPlanesVector.begin(),
                 vend     = localPlanesVector.end();
             markerIt != vend;
             markerIt++)
        {
            vs_graphs::core::geometric::Plane *p_localPlane = *markerIt;
            // If the plane does not exist, skip it
            if (!p_localPlane)
            {
                continue;
            }
            // If the plane is not known, do not add it to the local map
            geometric::Plane::PlaneVariant localPlanePlaneType{};
            if (p_localPlane->getPlaneType(localPlanePlaneType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getPlaneType cannot fail; continue as before.
            }
            if (localPlanePlaneType ==
                geometric::Plane::PlaneVariant::UNDEFINED)
            {
                continue;
            }
            // Otherwise, add the plane to the local map
            int localPlaneGetId{};
            if (p_localPlane->getId(localPlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (localPlaneId.find(localPlaneGetId) == localPlaneId.end())
            {
                localPlaneList.push_back(p_localPlane);
                int localPlaneGetId2{};
                if (p_localPlane->getId(localPlaneGetId2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                localPlaneId[localPlaneGetId2] = true;
            }
        }

        // [LBA] Loop through all the Doorways and prepare them for LBA
        for (std::vector<vs_graphs::core::semantic::Passage *>::iterator
                 markerIt = localDoorwaysVector.begin(),
                 vend     = localDoorwaysVector.end();
             markerIt != vend;
             markerIt++)
        {
            int id3{};
            if ((*markerIt)->getId(id3) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (localDoorwayId.find(id3) == localDoorwayId.end())
            {
                vs_graphs::core::semantic::Passage *p_doorway = *markerIt;
                localPassageList.push_back(p_doorway);
                int doorwayId{};
                if (p_doorway->getId(doorwayId) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                localDoorwayId[doorwayId] = true;
            }
        }
    }

    // [LBA] Among all rooms, filter only the ones with a wall in LBA
    for (const auto &room : allRooms)
    {
        // Get the walls of the room
        std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
        if (room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }
        // Add the room to the local map if any of the walls are in the local
        // map
        for (const auto &wall : roomWalls)
        {
            int wallGetId{};
            if (wall->getId(wallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (localPlaneId.find(wallGetId) != localPlaneId.end())
            {
                localRoomList.push_back(room);
                break;
            }
        }
    }

    // [LBA] Loop through all the local Rooms to add all their walls to LBA
    std::list<vs_graphs::core::geometric::Plane *> recentLocalMapPlanes;
    for (std::list<vs_graphs::core::semantic::Room *>::iterator
             markerIt = localRoomList.begin(),
             vend     = localRoomList.end();
         markerIt != vend;
         markerIt++)
    {
        std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
        if ((*markerIt)->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }
        for (const auto &roomWall : roomWalls)
        {
            int roomWallGetId{};
            if (roomWall->getId(roomWallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (localPlaneId.find(roomWallGetId) == localPlaneId.end())
            {
                localPlaneList.push_back(roomWall);
                int roomWallGetId2{};
                if (roomWall->getId(roomWallGetId2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                localPlaneId[roomWallGetId2] = true;
                recentLocalMapPlanes.push_back(roomWall);
            }
        }
    }

    // [LBA] Loop through the recently added planes, get all the KeyFrames and
    // add them
    std::list<vs_graphs::core::KeyFrame *> recentLocalMapKeyFrames;
    for (std::list<vs_graphs::core::geometric::Plane *>::iterator
             markerIt = recentLocalMapPlanes.begin(),
             vend     = recentLocalMapPlanes.end();
         markerIt != vend;
         markerIt++)
    {
        std::map<vs_graphs::core::KeyFrame *,
                 vs_graphs::core::geometric::Plane::Observation>
            planeObservations{};
        if ((*markerIt)->getObservations(planeObservations) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getObservations cannot fail; continue as before.
        }
        for (std::map<
                 vs_graphs::core::KeyFrame *,
                 vs_graphs::core::geometric::Plane::Observation>::const_iterator
                 observationId = planeObservations.begin(),
                 obLast        = planeObservations.end();
             observationId != obLast;
             observationId++)
        {
            vs_graphs::core::KeyFrame *p_keyFrame = observationId->first;
            if (!p_keyFrame->isBad() && p_keyFrame->getMap() == p_currentMap)
            {
                if (localKeyFrameId.find(p_keyFrame->id) ==
                    localKeyFrameId.end())
                {
                    localKeyFrameList.push_back(p_keyFrame);
                    localKeyFrameId[p_keyFrame->id] = true;
                    p_keyFrame->baLocalKeyFrameId   = p_keyFrame_inout->id;
                    recentLocalMapKeyFrames.push_back(p_keyFrame);
                }
            }
        }
    }

    // [LBA] Loop through the recently added keyframes, get all the map points
    // and add them
    for (std::list<vs_graphs::core::KeyFrame *>::iterator
             markerIt = recentLocalMapKeyFrames.begin(),
             vend     = recentLocalMapKeyFrames.end();
         markerIt != vend;
         markerIt++)
    {
        std::vector<vs_graphs::core::MapPoint *> vpMPs =
            (*markerIt)->getMapPointMatches();
        for (std::vector<vs_graphs::core::MapPoint *>::iterator
                 vit  = vpMPs.begin(),
                 vend = vpMPs.end();
             vit != vend;
             vit++)
        {
            vs_graphs::core::MapPoint *p_mapPoint = *vit;
            if (p_mapPoint)
            {
                if (!p_mapPoint->isBad() &&
                    p_mapPoint->getMap() == p_currentMap)
                {
                    if (p_mapPoint->baLocalKeyFrameId != p_keyFrame_inout->id)
                    {
                        localMapPointList.push_back(p_mapPoint);
                        localMapPointId[p_mapPoint->id] = true;
                        p_mapPoint->baLocalKeyFrameId   = p_keyFrame_inout->id;
                    }
                }
            }
        }
    }

    // [LBA] Fixed Keyframes for MPs (Keyframes that see Local MapPoints but
    // that are not Local Keyframes)
    std::list<vs_graphs::core::KeyFrame *> fixedCameras;
    for (std::list<vs_graphs::core::MapPoint *>::iterator
             lit  = localMapPointList.begin(),
             lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        std::map<vs_graphs::core::KeyFrame *, std::tuple<int, int>>
            observations = (*lit)->getObservations();
        for (std::map<vs_graphs::core::KeyFrame *,
                      std::tuple<int, int>>::iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            vs_graphs::core::KeyFrame *p_keyFrame = mit->first;

            if (p_keyFrame->baLocalKeyFrameId != p_keyFrame_inout->id &&
                p_keyFrame->baFixedKeyFrameId != p_keyFrame_inout->id)
            {
                p_keyFrame->baFixedKeyFrameId = p_keyFrame_inout->id;
                if (!p_keyFrame->isBad() &&
                    p_keyFrame->getMap() == p_currentMap)
                {
                    fixedCameras.push_back(p_keyFrame);
                }
            }
        }
    }

    // Count fixed KeyFrames
    fixedKeyFrameCount_inout = fixedCameras.size() + fixedKeyFrameCount_inout;
    if (fixedKeyFrameCount_inout == 0)
    {
        std::cout << "[Optimizer] No fixed KeyFrames found for LBA! Aborting..."
                  << std::endl;
        return;
    }

    // [LBA] Setup optimizer
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver;

    p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(p_linearSolver);

    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    if (p_map_inout->isInertial())
    {
        p_solver->setUserLambdaInit(100.0);
    }

    optimizer.setAlgorithm(p_solver);
    optimizer.setVerbose(false);

    if (p_pbStopFlag_in)
    {
        optimizer.setForceStopFlag(p_pbStopFlag_in);
    }

    unsigned long maximumKeyFrameId = 0;

    // Debug LBA
    p_currentMap->optKeyFrameIds.clear();
    p_currentMap->fixedKeyFrameIds.clear();

    // [LBA] Local KeyFrame vertices
    for (std::list<vs_graphs::core::KeyFrame *>::iterator
             lit  = localKeyFrameList.begin(),
             lend = localKeyFrameList.end();
         lit != lend;
         lit++)
    {
        vs_graphs::core::KeyFrame *p_keyFrame  = *lit;
        g2o::VertexSE3Expmap      *p_se3Vertex = new g2o::VertexSE3Expmap();
        Sophus::SE3<float>         Tcw         = p_keyFrame->getPose();
        p_se3Vertex->setEstimate(
            g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                         Tcw.translation().cast<double>()));
        p_se3Vertex->setId(p_keyFrame->id);
        p_se3Vertex->setFixed(p_keyFrame->id ==
                              p_map_inout->getInitKeyFrameId());
        optimizer.addVertex(p_se3Vertex);
        if (p_keyFrame->id > maximumKeyFrameId)
        {
            maximumKeyFrameId = p_keyFrame->id;
        }
        p_currentMap->optKeyFrameIds.insert(p_keyFrame->id);
    }
    optKeyFrameCount_out = localKeyFrameList.size();

    // [LBA] Fixed KeyFrame vertices
    for (std::list<vs_graphs::core::KeyFrame *>::iterator
             lit  = fixedCameras.begin(),
             lend = fixedCameras.end();
         lit != lend;
         lit++)
    {
        KeyFrame             *p_keyFrame  = *lit;
        g2o::VertexSE3Expmap *p_se3Vertex = new g2o::VertexSE3Expmap();
        Sophus::SE3<float>    Tcw         = p_keyFrame->getPose();
        p_se3Vertex->setEstimate(
            g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                         Tcw.translation().cast<double>()));
        p_se3Vertex->setId(p_keyFrame->id);
        p_se3Vertex->setFixed(true);
        optimizer.addVertex(p_se3Vertex);
        if (p_keyFrame->id > maximumKeyFrameId)
        {
            maximumKeyFrameId = p_keyFrame->id;
        }
        p_currentMap->fixedKeyFrameIds.insert(p_keyFrame->id);
    }

    // [LBA] MapPoint vertices
    const int expectedSizeCount =
        (localKeyFrameList.size() + fixedCameras.size()) *
        localMapPointList.size();

    vector<vs_graphs::core::EdgeSE3ProjectXYZ *> edgesMonos;
    edgesMonos.reserve(expectedSizeCount);

    vector<vs_graphs::core::EdgeSE3ProjectXYZToBody *> edgesBodies;
    edgesBodies.reserve(expectedSizeCount);

    vector<KeyFrame *> edgeKeyFrameMonos;
    edgeKeyFrameMonos.reserve(expectedSizeCount);

    vector<KeyFrame *> edgeKeyFrameBodies;
    edgeKeyFrameBodies.reserve(expectedSizeCount);

    vector<MapPoint *> mapPointEdgeMonos;
    mapPointEdgeMonos.reserve(expectedSizeCount);

    vector<MapPoint *> mapPointEdgeBodies;
    mapPointEdgeBodies.reserve(expectedSizeCount);

    vector<g2o::EdgeStereoSE3ProjectXYZ *> edgesStereos;
    edgesStereos.reserve(expectedSizeCount);

    vector<KeyFrame *> edgeKeyFrameStereos;
    edgeKeyFrameStereos.reserve(expectedSizeCount);

    vector<MapPoint *> mapPointEdgeStereos;
    mapPointEdgeStereos.reserve(expectedSizeCount);

    const int expectedSizePlaneCount =
        (localKeyFrameList.size() + fixedCameras.size()) *
        localPlaneList.size();
    vector<EdgeVertexPlaneProjectSE3KF *> edgesPlanes;
    edgesPlanes.reserve(expectedSizePlaneCount);

    vector<KeyFrame *> edgeKeyFramePlanes;
    edgeKeyFramePlanes.reserve(expectedSizePlaneCount);

    vector<geometric::Plane *> planeEdgePlanes;
    planeEdgePlanes.reserve(expectedSizePlaneCount);

    vector<EdgeSE3KFPointToPlane *> edgesPlanePoints;
    edgesPlanePoints.reserve(expectedSizePlaneCount);

    vector<KeyFrame *> edgeKeyFramePlanePoints;
    edgeKeyFramePlanePoints.reserve(expectedSizePlaneCount);

    vector<geometric::Plane *> planeEdgePlanePoints;
    planeEdgePlanePoints.reserve(expectedSizePlaneCount);

    const float thresholdHuber1d     = sqrt(3.841);
    const float thresholdHuberMono   = sqrt(5.991);
    const float thresholdHuberStereo = sqrt(7.815);

    int roomCount    = 1;
    int floorCount   = 1;
    int planeCount   = 1;
    int pointCount   = 0;
    int markerCount  = 1;
    int doorwayCount = 1;

    int edgeCount   = 0;
    int maximumOpId = 0;

    for (std::list<vs_graphs::core::MapPoint *>::iterator
             lit  = localMapPointList.begin(),
             lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        vs_graphs::core::MapPoint *p_mapPoint    = *lit;
        g2o::VertexSBAPointXYZ    *p_pointVertex = new g2o::VertexSBAPointXYZ();
        p_pointVertex->setEstimate(p_mapPoint->getWorldPos().cast<double>());
        int id = p_mapPoint->id + maximumKeyFrameId + 1;
        p_pointVertex->setId(id);
        p_pointVertex->setMarginalized(true);
        optimizer.addVertex(p_pointVertex);
        pointCount++;

        // Update the maxOpId to hold the biggest value
        if (id > maximumOpId)
        {
            maximumOpId = id;
        }

        const map<KeyFrame *, tuple<int, int>> observations =
            p_mapPoint->getObservations();

        // Set edges
        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            KeyFrame *p_keyFrame = mit->first;

            if (!p_keyFrame->isBad() && p_keyFrame->getMap() == p_currentMap)
            {
                const int leftIndex = get<0>(mit->second);

                // Monocular observation
                if (leftIndex != -1 &&
                    p_keyFrame->uRight[get<0>(mit->second)] < 0)
                {
                    const cv::KeyPoint &keyPointUn =
                        p_keyFrame->keyPointsUndistorted[leftIndex];
                    Eigen::Matrix<double, 2, 1> observation;
                    observation << keyPointUn.pt.x, keyPointUn.pt.y;

                    vs_graphs::core::EdgeSE3ProjectXYZ *e =
                        new vs_graphs::core::EdgeSE3ProjectXYZ();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(p_keyFrame->id)));
                    e->setMeasurement(observation);
                    const float &invSigma2 =
                        p_keyFrame->invLevelSigmaSquared[keyPointUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(thresholdHuberMono);

                    e->p_camera = p_keyFrame->p_camera;

                    optimizer.addEdge(e);
                    edgesMonos.push_back(e);
                    edgeKeyFrameMonos.push_back(p_keyFrame);
                    mapPointEdgeMonos.push_back(p_mapPoint);

                    edgeCount++;
                }
                else if (leftIndex != -1 &&
                         p_keyFrame->uRight[get<0>(mit->second)] >=
                             0) // Stereo observation
                {
                    const cv::KeyPoint &keyPointUn =
                        p_keyFrame->keyPointsUndistorted[leftIndex];
                    Eigen::Matrix<double, 3, 1> observation;
                    const float                 rightKeyPointU =
                        p_keyFrame->uRight[get<0>(mit->second)];
                    observation << keyPointUn.pt.x, keyPointUn.pt.y,
                        rightKeyPointU;

                    g2o::EdgeStereoSE3ProjectXYZ *e =
                        new g2o::EdgeStereoSE3ProjectXYZ();

                    if (optimizer.vertex(id) &&
                        optimizer.vertex(p_keyFrame->id))
                    {
                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(id)));
                        e->setVertex(
                            1,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(p_keyFrame->id)));
                        e->setMeasurement(observation);
                        const float &invSigma2 =
                            p_keyFrame->invLevelSigmaSquared[keyPointUn.octave];
                        Eigen::Matrix3d Info =
                            Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *p_robustKernel =
                            new g2o::RobustKernelHuber;
                        e->setRobustKernel(p_robustKernel);
                        p_robustKernel->setDelta(thresholdHuberStereo);

                        e->fx = p_keyFrame->fx;
                        e->fy = p_keyFrame->fy;
                        e->cx = p_keyFrame->cx;
                        e->cy = p_keyFrame->cy;
                        e->bf = p_keyFrame->mbf;

                        optimizer.addEdge(e);
                        edgesStereos.push_back(e);
                        edgeKeyFrameStereos.push_back(p_keyFrame);
                        mapPointEdgeStereos.push_back(p_mapPoint);

                        edgeCount++;
                    }
                }

                if (p_keyFrame->p_camera2)
                {
                    int rightIndex = get<1>(mit->second);

                    if (rightIndex != -1 &&
                        rightIndex < (int)p_keyFrame->keyPointsRight.size())
                    {
                        rightIndex -= p_keyFrame->leftKeyPointCount;

                        Eigen::Matrix<double, 2, 1> observation;
                        cv::KeyPoint                keyPoint =
                            p_keyFrame->keyPointsRight[rightIndex];
                        observation << keyPoint.pt.x, keyPoint.pt.y;

                        vs_graphs::core::EdgeSE3ProjectXYZToBody *e =
                            new vs_graphs::core::EdgeSE3ProjectXYZToBody();

                        if (optimizer.vertex(id) &&
                            optimizer.vertex(p_keyFrame->id))
                        {
                            e->setVertex(
                                0,
                                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex(id)));
                            e->setVertex(
                                1,
                                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex(p_keyFrame->id)));
                            e->setMeasurement(observation);
                            const float &invSigma2 =
                                p_keyFrame
                                    ->invLevelSigmaSquared[keyPoint.octave];
                            e->setInformation(Eigen::Matrix2d::Identity() *
                                              invSigma2);

                            g2o::RobustKernelHuber *p_robustKernel =
                                new g2o::RobustKernelHuber;
                            e->setRobustKernel(p_robustKernel);
                            p_robustKernel->setDelta(thresholdHuberMono);

                            Sophus::SE3f Trl = p_keyFrame->getRelativePoseTrl();
                            e->mTrl          = g2o::SE3Quat(
                                Trl.unit_quaternion().cast<double>(),
                                Trl.translation().cast<double>());

                            e->p_camera = p_keyFrame->p_camera2;

                            optimizer.addEdge(e);
                            edgesBodies.push_back(e);
                            edgeKeyFrameBodies.push_back(p_keyFrame);
                            mapPointEdgeBodies.push_back(p_mapPoint);

                            edgeCount++;
                        }
                    }
                }
            }
        }
    }
    edgeCount_out = edgeCount;

    // [LBA] Markers
    for (list<semantic::Marker *>::iterator markerIt = localMarkerList.begin(),
                                            lend     = localMarkerList.end();
         markerIt != lend;
         markerIt++)
    {
        // Adding a vertex for each marker
        semantic::Marker     *p_mapMarker    = *markerIt;
        g2o::VertexSE3Expmap *p_markerVertex = new g2o::VertexSE3Expmap();
        Sophus::SE3f          mapMarkerGlobalPose{};
        if (p_mapMarker->getGlobalPose(mapMarkerGlobalPose) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            // getGlobalPose cannot fail; continue as before.
        }
        Sophus::SE3f mapMarkerGlobalPose2{};
        if (p_mapMarker->getGlobalPose(mapMarkerGlobalPose2) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            // getGlobalPose cannot fail; continue as before.
        }
        p_markerVertex->setEstimate(
            g2o::SE3Quat(mapMarkerGlobalPose.unit_quaternion().cast<double>(),
                         mapMarkerGlobalPose2.translation().cast<double>()));
        int opId = maximumOpId + markerCount;
        p_markerVertex->setId(opId);
        optimizer.addVertex(p_markerVertex);
        markerCount++;

        // Setting the local optimization ID for the marker
        if (p_mapMarker->setOpId(opId) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            // setOpId cannot fail; continue as before.
        }

        // 🚧 [vS-Graphs v.2.0] in contrast with the first version of visual
        // S-Graphs, where there was an edge between the marker and the
        // keyframe, in this version we removed that edge and added an edge
        // between the plane and the keyframe, while still keeping the edge
        // between the marker and the plane.
    }

    maximumOpId += markerCount;

    // [LBA] Planes
    for (std::list<vs_graphs::core::geometric::Plane *>::iterator
             markerIt = localPlaneList.begin(),
             lend     = localPlaneList.end();
         markerIt != lend;
         markerIt++)
    {
        // Variables
        vs_graphs::core::geometric::Plane *p_mapPlane = *markerIt;
        g2o::VertexPlane *p_planeVertex               = new g2o::VertexPlane();

        // Adding a vertex for each plane
        int opId = maximumOpId + planeCount;
        p_planeVertex->setId(opId);

        if (p_sysParams->optimization.shouldMarginalizePlanes)
        {
            p_planeVertex->setMarginalized(true);
        }

        g2o::Plane3D planeGlobalEquation{};
        if (p_mapPlane->getGlobalEquation(planeGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getGlobalEquation cannot fail; continue as before.
        }
        p_planeVertex->setEstimate(planeGlobalEquation);
        optimizer.addVertex(p_planeVertex);
        planeCount++;

        // Setting the local optimization ID for the plane
        if (p_mapPlane->setOpId(opId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // setOpId cannot fail; continue as before.
        }

        // Adding edge between plane and MapPoints
        if (p_sysParams->optimization.planeMapPoint.enabled &&
            !p_sysParams->optimization.shouldMarginalizePlanes)
        {
            set<MapPoint *> mapPoints{};
            if (p_mapPlane->getMapPoints(mapPoints) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getMapPoints cannot fail; continue as before.
            }
            for (set<MapPoint *>::iterator lit  = mapPoints.begin(),
                                           lend = mapPoints.end();
                 lit != lend;
                 lit++)
            {
                MapPoint *p_mapPoint = *lit;

                if (!p_mapPoint || p_mapPoint->isBad())
                {
                    continue;
                }

                if (optimizer.vertex(opId) &&
                    optimizer.vertex(p_mapPoint->id + maximumKeyFrameId + 1))
                {
                    vs_graphs::core::EdgeVertexPlaneProjectPointXYZ *e =
                        new vs_graphs::core::EdgeVertexPlaneProjectPointXYZ();
                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(p_mapPoint->id +
                                                      maximumKeyFrameId + 1)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(opId)));
                    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                                      p_sysParams->optimization.planeMapPoint
                                          .informationGain);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(thresholdHuber1d);
                    optimizer.addEdge(e);
                    edgeCount++;
                }
            }
        }

        // Adding an edge between the plane and the keyframes
        map<KeyFrame *, vs_graphs::core::geometric::Plane::Observation>
            observations{};
        if (p_mapPlane->getObservations(observations) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // getObservations cannot fail; continue as before.
        }
        for (map<KeyFrame *,
                 vs_graphs::core::geometric::Plane::Observation>::const_iterator
                 observationId = observations.begin(),
                 obLast        = observations.end();
             observationId != obLast;
             observationId++)
        {
            KeyFrame *p_keyFrame = observationId->first;
            vs_graphs::core::geometric::Plane::Observation observation =
                observationId->second;

            if (p_keyFrame->isBad())
            {
                std::cout
                    << "[Optimizer] Bad KeyFrame detected for LBA! Skipping..."
                    << std::endl;
                if (p_mapPlane->eraseObservation(p_keyFrame) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // eraseObservation cannot fail; continue as before.
                }
                continue;
            }

            if (p_keyFrame->getMap() != p_currentMap)
            {
                std::cout << "[Optimizer] KeyFrame is not in the current map! "
                             "Skipping..."
                          << std::endl;
                continue;
            }

            if (optimizer.vertex(opId) && optimizer.vertex(p_keyFrame->id))
            {
                if (p_sysParams->optimization.planeKf.enabled)
                {
                    vs_graphs::core::EdgeVertexPlaneProjectSE3KF *e =
                        new vs_graphs::core::EdgeVertexPlaneProjectSE3KF();
                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(p_keyFrame->id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(opId)));
                    e->setInformation(
                        Eigen::Matrix<double, 3, 3>::Identity() *
                        observation.confidence *
                        p_sysParams->optimization.planeKf.informationGain);
                    e->setMeasurement(observation.localPlane);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(thresholdHuberStereo);
                    optimizer.addEdge(e);
                    edgeCount++;

                    edgesPlanes.push_back(e);
                    edgeKeyFramePlanes.push_back(p_keyFrame);
                    planeEdgePlanes.push_back(p_mapPlane);
                }

                // Adding plane-point constraints
                if (p_sysParams->optimization.planePoint.enabled)
                {
                    // Get the class index of the plane
                    int                            clsCloudIndex{};
                    geometric::Plane::PlaneVariant mapPlanePlaneType{};
                    if (p_mapPlane->getPlaneType(mapPlanePlaneType) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getPlaneType cannot fail; continue as before.
                    }
                    if (utils::utils::Utils::getClassIdFromPlaneType(
                            mapPlanePlaneType,
                            clsCloudIndex) !=
                        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                    {
                        // getClassIdFromPlaneType cannot fail; continue as
                        // before.
                    }
                    if (clsCloudIndex != -1)
                    {
                        // Add the plane-point constraint
                        vs_graphs::core::EdgeSE3KFPointToPlane *e =
                            new vs_graphs::core::EdgeSE3KFPointToPlane();
                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(p_keyFrame->id)));
                        e->setVertex(
                            1,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(opId)));
                        e->setInformation(
                            Eigen::Matrix<double, 1, 1>::Identity() *
                            observation.confidence *
                            p_sysParams->optimization.planePoint
                                .informationGain);
                        e->setMeasurement(
                            observation.pointPlaneConstraintMatrix);

                        g2o::RobustKernelHuber *p_robustKernel =
                            new g2o::RobustKernelHuber;
                        e->setRobustKernel(p_robustKernel);
                        p_robustKernel->setDelta(thresholdHuber1d);
                        optimizer.addEdge(e);
                        edgeCount++;

                        edgesPlanePoints.push_back(e);
                        edgeKeyFramePlanePoints.push_back(p_keyFrame);
                        planeEdgePlanePoints.push_back(p_mapPlane);
                    }
                }
            }
        }
    }

    maximumOpId += planeCount;

    // [LBA] Rooms
    for (std::list<vs_graphs::core::semantic::Room *>::iterator
             markerIt = localRoomList.begin(),
             lend     = localRoomList.end();
         markerIt != lend;
         markerIt++)
    {
        try
        {
            // Variables
            vs_graphs::core::semantic::Room *p_mapRoom = *markerIt;
            std::vector<vs_graphs::core::geometric::Plane *> walls{};
            if (p_mapRoom->getWalls(walls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWalls cannot fail; continue as before.
            }

            // No need to optimize if there are no walls
            if (walls.empty())
            {
                continue;
            }

            // Adding a vertex for each room
            g2o::VertexSE3Expmap *p_vertexRoom = new g2o::VertexSE3Expmap();

            // Setting the local optimization ID for the room
            int opId = maximumOpId + roomCount;
            p_vertexRoom->setId(opId);
            if (p_mapRoom->setOpId(opId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setOpId cannot fail; continue as before.
            }
            roomCount++;

            // Initialize the room vertex (centroid estimate)
            Eigen::Vector3d mapRoomCentroid{};
            if (p_mapRoom->getCentroid(mapRoomCentroid) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            p_vertexRoom->setEstimate(
                g2o::SE3Quat(Eigen::Quaterniond::Identity(),
                             mapRoomCentroid.cast<double>()));
            p_vertexRoom->setFixed(true);
            optimizer.addVertex(p_vertexRoom);

            /*
             * See the global BA path above: the legacy room-centering factor
             * is deliberately disabled because it can deform wall geometry.
             */

            // Optimizing the parallel walls of the room
            for (size_t edgeIndex = 0; edgeIndex < walls.size(); edgeIndex++)
            {
                for (size_t otherWallIndex = edgeIndex + 1;
                     otherWallIndex < walls.size();
                     otherWallIndex++)
                {
                    vs_graphs::core::geometric::Plane *p_wall1 =
                        walls[edgeIndex];
                    vs_graphs::core::geometric::Plane *p_wall2 =
                        walls[otherWallIndex];

                    // If the same wall, skip
                    int wall1GetId{};
                    if (p_wall1->getId(wall1GetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    int wall2GetId{};
                    if (p_wall2->getId(wall2GetId) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getId cannot fail; continue as before.
                    }
                    if (wall1GetId == wall2GetId)
                    {
                        continue;
                    }

                    // Check if the walls are parallel
                    bool arePlanesParallel2{};
                    if (utils::utils::Utils::arePlanesParallel(
                            p_wall1,
                            p_wall2,
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
                                p_wall1,
                                p_wall2,
                                arePlanesFacingEachOther2) !=
                            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                        {
                            // arePlanesFacingEachOther cannot fail; continue as
                            // before.
                        }
                        if (arePlanesFacingEachOther2)
                        {
                            // Variables
                            int opId1{};
                            if (p_wall1->getOpId(opId1) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                            {
                                // getOpId cannot fail; continue as before.
                            }
                            int opId2{};
                            if (p_wall2->getOpId(opId2) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                            {
                                // getOpId cannot fail; continue as before.
                            }

                            if (optimizer.vertex(opId) &&
                                optimizer.vertex(opId1) &&
                                optimizer.vertex(opId2))
                            {
                                vs_graphs::core::EdgeVertexPlaneParallelism *e =
                                    new vs_graphs::core::
                                        EdgeVertexPlaneParallelism();
                                e->setVertex(
                                    0,
                                    dynamic_cast<
                                        g2o::OptimizableGraph::Vertex *>(
                                        optimizer.vertex(opId1)));
                                e->setVertex(
                                    1,
                                    dynamic_cast<
                                        g2o::OptimizableGraph::Vertex *>(
                                        optimizer.vertex(opId2)));
                                e->setMeasurement(
                                    0.0); // We want the angle between the
                                          // planes to be 0

                                // Information matrix
                                e->setInformation(
                                    Eigen::Matrix<double, 1, 1>::Identity() *
                                    1e3);

                                // Adding the edge to the optimizer
                                g2o::RobustKernelHuber *p_robustKernel =
                                    new g2o::RobustKernelHuber;
                                e->setRobustKernel(p_robustKernel);
                                p_robustKernel->setDelta(thresholdHuber1d);
                                optimizer.addEdge(e);
                            }
                        }
                    }

                    // Check if the walls are perpendicular
                    bool arePlanesPerpendicular2{};
                    if (utils::utils::Utils::arePlanesPerpendicular(
                            p_wall1,
                            p_wall2,
                            arePlanesPerpendicular2) !=
                        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                    {
                        // arePlanesPerpendicular cannot fail; continue as
                        // before.
                    }
                    if (arePlanesPerpendicular2)
                    {
                        // Variables
                        int opId1{};
                        if (p_wall1->getOpId(opId1) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getOpId cannot fail; continue as before.
                        }
                        int opId2{};
                        if (p_wall2->getOpId(opId2) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getOpId cannot fail; continue as before.
                        }

                        if (optimizer.vertex(opId) && optimizer.vertex(opId1) &&
                            optimizer.vertex(opId2))
                        {
                            vs_graphs::core::EdgeVertexPlanePerpendicularity
                                *e = new vs_graphs::core::
                                    EdgeVertexPlanePerpendicularity();
                            e->setVertex(
                                0,
                                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex(opId1)));
                            e->setVertex(
                                1,
                                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex(opId2)));
                            e->setMeasurement(
                                M_PI_2); // We want the angle between the planes
                                         // to be 90 degrees

                            // Information matrix
                            e->setInformation(
                                Eigen::Matrix<double, 1, 1>::Identity() * 1e3);

                            // Adding the edge to the optimizer
                            g2o::RobustKernelHuber *p_robustKernel =
                                new g2o::RobustKernelHuber;
                            e->setRobustKernel(p_robustKernel);
                            p_robustKernel->setDelta(thresholdHuber1d);
                            optimizer.addEdge(e);
                        }
                    }
                }
            }
        }
        catch (std::exception &e)
        {
            std::cerr << "[Optimizer] Error while locally optimizing room: "
                      << e.what() << std::endl;
            continue;
        }
    }

    maximumOpId += roomCount;

    // abort if no edges
    if (edgeCount == 0)
    {
        Verbose::printMess(
            "LM-LBA: There are 0 edges in the optimizations, LBA aborted",
            Verbose::VERBOSITY_NORMAL);
        return;
    }

    if (p_pbStopFlag_in)
    {
        if (*p_pbStopFlag_in)
        {
            return;
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(edgesMonos.size() + edgesBodies.size() +
                     edgesStereos.size());

    vector<pair<KeyFrame *, geometric::Plane *>> vToErasePlane;
    vToErasePlane.reserve(edgesPlanes.size() * 2);

    // Check inlier observations
    for (size_t edgeIndex = 0, iend = edgesMonos.size(); edgeIndex < iend;
         edgeIndex++)
    {
        vs_graphs::core::EdgeSE3ProjectXYZ *e = edgesMonos[edgeIndex];
        MapPoint *p_mapPoint                  = mapPointEdgeMonos[edgeIndex];

        if (p_mapPoint->isBad())
        {
            continue;
        }

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *p_keyFrame = edgeKeyFrameMonos[edgeIndex];
            vToErase.push_back(make_pair(p_keyFrame, p_mapPoint));
        }
    }

    for (size_t edgeIndex = 0, iend = edgesBodies.size(); edgeIndex < iend;
         edgeIndex++)
    {
        vs_graphs::core::EdgeSE3ProjectXYZToBody *e = edgesBodies[edgeIndex];
        MapPoint *p_mapPoint = mapPointEdgeBodies[edgeIndex];

        if (p_mapPoint->isBad())
        {
            continue;
        }

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *p_keyFrame = edgeKeyFrameBodies[edgeIndex];
            vToErase.push_back(make_pair(p_keyFrame, p_mapPoint));
        }
    }

    for (size_t edgeIndex = 0, iend = edgesStereos.size(); edgeIndex < iend;
         edgeIndex++)
    {
        g2o::EdgeStereoSE3ProjectXYZ *e = edgesStereos[edgeIndex];
        MapPoint *p_mapPoint            = mapPointEdgeStereos[edgeIndex];

        if (p_mapPoint->isBad())
        {
            continue;
        }

        if (e->chi2() > 7.815 || !e->isDepthPositive())
        {
            KeyFrame *p_keyFrame = edgeKeyFrameStereos[edgeIndex];
            vToErase.push_back(make_pair(p_keyFrame, p_mapPoint));
        }
    }

    for (size_t edgeIndex = 0, iend = edgesPlanes.size(); edgeIndex < iend;
         edgeIndex++)
    {
        vs_graphs::core::EdgeVertexPlaneProjectSE3KF *e =
            edgesPlanes[edgeIndex];
        geometric::Plane *p_edgePlane = planeEdgePlanes[edgeIndex];

        if (e->chi2() > 7.815 || !e->isDistanceCorrect())
        {

            // if not already in ToErase, add it
            std::pair<KeyFrame *, geometric::Plane *> keyFramePlane =
                make_pair(edgeKeyFramePlanes[edgeIndex], p_edgePlane);
            if (std::find(vToErasePlane.begin(),
                          vToErasePlane.end(),
                          keyFramePlane) == vToErasePlane.end())
            {
                vToErasePlane.push_back(keyFramePlane);
            }
        }
    }

    for (size_t edgeIndex = 0, iend = edgesPlanePoints.size(); edgeIndex < iend;
         edgeIndex++)
    {
        vs_graphs::core::EdgeSE3KFPointToPlane *e = edgesPlanePoints[edgeIndex];
        geometric::Plane *p_edgePlane = planeEdgePlanePoints[edgeIndex];

        if (e->chi2() > 3.841 || !e->isDistanceCorrect())
        {
            // if not already in ToErase, add it
            std::pair<KeyFrame *, geometric::Plane *> keyFramePlane =
                make_pair(edgeKeyFramePlanePoints[edgeIndex], p_edgePlane);
            if (std::find(vToErasePlane.begin(),
                          vToErasePlane.end(),
                          keyFramePlane) == vToErasePlane.end())
            {
                vToErasePlane.push_back(keyFramePlane);
            }
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(p_map_inout->mapUpdateMutex);

    if (!vToErase.empty())
    {
        for (size_t edgeIndex = 0; edgeIndex < vToErase.size(); edgeIndex++)
        {
            KeyFrame *p_keyFrame        = vToErase[edgeIndex].first;
            MapPoint *p_mapPointToErase = vToErase[edgeIndex].second;
            p_keyFrame->eraseMapPointMatch(p_mapPointToErase);
            p_mapPointToErase->eraseObservation(p_keyFrame);
        }
    }

    if (!vToErasePlane.empty())
    {
        for (size_t edgeIndex = 0; edgeIndex < vToErasePlane.size();
             edgeIndex++)
        {
            KeyFrame         *p_keyFrame = vToErasePlane[edgeIndex].first;
            geometric::Plane *p_plane    = vToErasePlane[edgeIndex].second;
            if (p_plane->eraseObservation(p_keyFrame) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // eraseObservation cannot fail; continue as before.
            }
            p_keyFrame->removeMapPlane(p_plane);
        }
    }

    // [LBA] Locally optimized KeyFrames
    for (list<KeyFrame *>::iterator lit  = localKeyFrameList.begin(),
                                    lend = localKeyFrameList.end();
         lit != lend;
         lit++)
    {
        try
        {
            KeyFrame             *p_keyFrame = *lit;
            g2o::VertexSE3Expmap *p_se3Vertex =
                static_cast<g2o::VertexSE3Expmap *>(
                    optimizer.vertex(p_keyFrame->id));
            g2o::SE3Quat poseEstimate = p_se3Vertex->estimate();
            Sophus::SE3f Tiw(poseEstimate.rotation().cast<float>(),
                             poseEstimate.translation().cast<float>());
            p_keyFrame->setPose(Tiw);
        }
        catch (std::exception &e)
        {
            std::cerr << "[Optimizer] Error while locally updating optimized "
                         "KeyFrame: "
                      << e.what() << std::endl;
            continue;
        }
    }

    // [LBA] Locally optimized MapPoints
    for (list<MapPoint *>::iterator lit  = localMapPointList.begin(),
                                    lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        try
        {
            MapPoint               *p_mapPoint = *lit;
            g2o::VertexSBAPointXYZ *p_pointVertex =
                static_cast<g2o::VertexSBAPointXYZ *>(
                    optimizer.vertex(p_mapPoint->id + maximumKeyFrameId + 1));
            p_mapPoint->setWorldPos(p_pointVertex->estimate().cast<float>());
            p_mapPoint->updateNormalAndDepth();
        }
        catch (std::exception &e)
        {
            std::cerr << "[Optimizer] Error while locally updating optimized "
                         "MapPoint: "
                      << e.what() << std::endl;
            continue;
        }
    }

    // [LBA] Locally optimized markers
    for (list<semantic::Marker *>::iterator markerIt = localMarkerList.begin(),
                                            lend     = localMarkerList.end();
         markerIt != lend;
         markerIt++)
    {
        try
        {
            semantic::Marker *p_mapMarker = *markerIt;
            int               mapMarkerOpId{};
            if (p_mapMarker->getOpId(mapMarkerOpId) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // getOpId cannot fail; continue as before.
            }
            g2o::VertexSE3Expmap *p_markerVertex =
                static_cast<g2o::VertexSE3Expmap *>(
                    optimizer.vertex(mapMarkerOpId));
            g2o::SE3Quat poseEstimate = p_markerVertex->estimate();
            Sophus::SE3f Tiw(poseEstimate.rotation().cast<float>(),
                             poseEstimate.translation().cast<float>());
            if (p_mapMarker->setGlobalPose(Tiw) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // setGlobalPose cannot fail; continue as before.
            }
        }
        catch (std::exception &e)
        {
            std::cerr
                << "[Optimizer] Error while locally updating optimized marker: "
                << e.what() << std::endl;
            continue;
        }
    }

    // [LBA] Locally optimized planes
    for (std::list<vs_graphs::core::geometric::Plane *>::iterator
             markerIt = localPlaneList.begin(),
             lend     = localPlaneList.end();
         markerIt != lend;
         markerIt++)
    {
        try
        {
            vs_graphs::core::geometric::Plane *p_mapPlane = *markerIt;
            int                                mapPlaneGetOpId{};
            if (p_mapPlane->getOpId(mapPlaneGetOpId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getOpId cannot fail; continue as before.
            }
            g2o::VertexPlane *p_planeVertex = static_cast<g2o::VertexPlane *>(
                optimizer.vertex(mapPlaneGetOpId));
            g2o::Plane3D planePlane = p_planeVertex->estimate();
            if (p_mapPlane->setGlobalEquation(planePlane) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // setGlobalEquation cannot fail; continue as before.
            }
        }
        catch (std::exception &e)
        {
            std::cerr
                << "[Optimizer] Error while locally updating optimized plane: "
                << e.what() << std::endl;
            continue;
        }
    }

    // [LBA] Locally optimized rooms
    // 🚧 Temporarily disabled: The reason is to avoid getting room centroid
    // dragged into the wall equation centroid for
    // (std::list<vs_graphs::core::semantic::Room
    // *>::iterator idx = localRoomList.begin(), lend = localRoomList.end(); idx
    // != lend; idx++)
    // {
    //     try
    //     {
    //         vs_graphs::core::semantic::Room *pMapRoom = *idx;
    //         g2o::VertexSE3Expmap *vrtxRoom = static_cast<g2o::VertexSE3Expmap
    //         *>(optimizer.vertex(pMapRoom->getOpId())); g2o::SE3Quat SE3quat =
    //         vrtxRoom->estimate();
    //         pMapRoom->setCentroid(SE3quat.translation());

    //         // Locally Optimized Doorways
    //         // for (const auto doorway : pMapRoom->getPassages())
    //         // {
    //         //     vs_graphs::core::semantic::Passage *pMapDoorway = doorway;
    //         //     g2o::VertexSE3Expmap *vDoorway =
    //         static_cast<g2o::VertexSE3Expmap
    //         *>(optimizer.vertex(pMapDoorway->getOpId()));
    //         //     g2o::SE3Quat SE3quat = vDoorway->estimate();
    //         //     Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
    //         SE3quat.translation().cast<float>());
    //         //     pMapDoorway->setGlobalPose(Tiw);
    //         // }
    //     }
    //     catch (std::exception &e)
    //     {
    //         std::cerr << "[Optimizer] Error while locally updating optimized
    //         room: " << e.what() << std::endl; continue;
    //     }
    // }

    // [LBA] Locally optimized floors
    // 🚧 Temporarily disabled: We moved the floor optimization to the the
    // front-end (SemanticsManager) as some rooms may not be available in the
    // local optimizer, causing wrong floor optimization for (const auto
    // &pMapFloor : allFloors)
    // {
    //     try
    //     {
    //         int opId = pMapFloor->getOpId();

    //         if (opId < 0)
    //             continue;

    //         g2o::OptimizableGraph::Vertex *vBase = optimizer.vertex(opId);
    //         if (!vBase)
    //         {
    //             std::cerr << "[Warning] Floor vertex with opId='" << opId <<
    //             "' not found in optimizer!" << std::endl; continue;
    //         }

    //         // Safe downcast
    //         g2o::VertexSE3Expmap *vrtxFloor =
    //         dynamic_cast<g2o::VertexSE3Expmap *>(vBase); if (!vrtxFloor)
    //         {
    //             std::cerr << "[Warning] Vertex Floor with opId='" << opId <<
    //             "' is not a VertexSE3Expmap!" << std::endl; continue;
    //         }

    //         pMapFloor->setCentroid(vrtxFloor->estimate().translation());
    //     }
    //     catch (std::exception &e)
    //     {
    //         std::cerr << "[Optimizer] Error while locally updating optimized
    //         floor: " << e.what() << std::endl; continue;
    //     }
    // }

    p_map_inout->increaseChangeIndex();
}

} // namespace core
} // namespace vs_graphs
