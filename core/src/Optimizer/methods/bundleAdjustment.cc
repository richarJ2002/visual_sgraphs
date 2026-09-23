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

namespace vs_graphs
{
namespace core
{

void Optimizer::bundleAdjustment(
    const std::vector<vs_graphs::core::KeyFrame *>          &vpKFs,
    const std::vector<vs_graphs::core::MapPoint *>          &vpMP,
    const std::vector<vs_graphs::core::semantic::Marker *>  &allMarkersVec,
    const std::vector<vs_graphs::core::geometric::Plane *>  &allPlanesVec,
    const std::vector<vs_graphs::core::semantic::Passage *> &allDoorwaysVec,
    const std::vector<vs_graphs::core::semantic::Room *>    &vpRooms,
    const std::vector<vs_graphs::core::semantic::Floor *>   &vpFloors,
    int                                                      nIterations,
    bool                                                    *pbStopFlag,
    const unsigned long                                      nLoopKF,
    const bool                                               bRobust,
    double                                                   markerImpact,
    const std::atomic_bool                                  *pStopRequested_in)
{
    // System parameters
    vs_graphs::core::types::SystemParams *p_sysParams =
        vs_graphs::core::types::SystemParams::getParams();

    // Variables
    std::vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    if (vpKFs.empty())
        return;

    vs_graphs::core::Map *pMap = vpKFs[0]->getMap();

    AtomicOptimizerStopBridge stopBridge(pStopRequested_in, pbStopFlag);

    // Set up the solver
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    if (pStopRequested_in != nullptr && pbStopFlag != nullptr)
    {
        optimizer.addPreIterationAction(&stopBridge);
        optimizer.addPostIterationAction(&stopBridge);
    }

    long unsigned int maxKFid = 0;

    const int nExpectedSize = (vpKFs.size()) * vpMP.size();

    vector<vs_graphs::core::EdgeSE3ProjectXYZ *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<vs_graphs::core::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
    vpEdgesBody.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFBody;
    vpEdgeKFBody.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    // [GBA] KeyFrames
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float>    Tcw  = pKF->getPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId == pMap->getInitKeyFrameId());
        optimizer.addVertex(vSE3);
        if (pKF->mnId > maxKFid)
            maxKFid = pKF->mnId;
    }

    const float thHuber1D = sqrt(3.841);
    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    int nPlanes   = 1;
    int nRooms    = 1;
    int nFloors   = 1;
    int maxOpId   = 0;
    int nMarkers  = 1;
    int nDoorways = 1;

    // [GBA] MapPoints
    for (size_t i = 0; i < vpMP.size(); i++)
    {
        MapPoint *pMP = vpMP[i];
        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->getWorldPos().cast<double>());
        const int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        // Update the maxOpId to hold the biggest value
        if (id > maxOpId)
            maxOpId = id;

        const map<KeyFrame *, tuple<int, int>> observations =
            pMP->getObservations();

        int nEdges = 0;
        // SET EDGES
        for (map<KeyFrame *, tuple<int, int>>::const_iterator mit =
                 observations.begin();
             mit != observations.end();
             mit++)
        {
            KeyFrame *pKF = mit->first;
            if (pKF->isBad() || pKF->mnId > maxKFid)
                continue;
            if (optimizer.vertex(id) == nullptr ||
                optimizer.vertex(pKF->mnId) == nullptr)
                continue;
            nEdges++;

            const int leftIndex = get<0>(mit->second);

            if (leftIndex != -1 && pKF->uRight[get<0>(mit->second)] < 0)
            {
                const cv::KeyPoint &kpUn = pKF->keyPointsUndistorted[leftIndex];

                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                vs_graphs::core::EdgeSE3ProjectXYZ *e =
                    new vs_graphs::core::EdgeSE3ProjectXYZ();

                e->setVertex(0,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(id)));
                e->setVertex(1,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->invLevelSigmaSquared[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                if (bRobust)
                {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->pCamera = pKF->p_camera;

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMP);
            }
            else if (leftIndex != -1 &&
                     pKF->uRight[leftIndex] >= 0) // Stereo observation
            {
                const cv::KeyPoint &kpUn = pKF->keyPointsUndistorted[leftIndex];

                Eigen::Matrix<double, 3, 1> obs;
                const float kp_ur = pKF->uRight[get<0>(mit->second)];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ *e =
                    new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(id)));
                e->setVertex(1,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->invLevelSigmaSquared[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                e->setInformation(Info);

                if (bRobust)
                {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMP);
            }

            if (pKF->p_camera2)
            {
                int rightIndex = get<1>(mit->second);

                if (rightIndex != -1 && static_cast<size_t>(rightIndex) <
                                            pKF->keyPointsRight.size())
                {
                    rightIndex -= pKF->Nleft;

                    Eigen::Matrix<double, 2, 1> obs;
                    cv::KeyPoint kp = pKF->keyPointsRight[rightIndex];
                    obs << kp.pt.x, kp.pt.y;

                    vs_graphs::core::EdgeSE3ProjectXYZToBody *e =
                        new vs_graphs::core::EdgeSE3ProjectXYZToBody();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 =
                        pKF->invLevelSigmaSquared[kp.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);

                    Sophus::SE3f Trl = pKF->getRelativePoseTrl();
                    e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(),
                                           Trl.translation().cast<double>());

                    e->pCamera = pKF->p_camera2;

                    optimizer.addEdge(e);
                    vpEdgesBody.push_back(e);
                    vpEdgeKFBody.push_back(pKF);
                    vpMapPointEdgeBody.push_back(pMP);
                }
            }
        }

        if (nEdges == 0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i] = true;
        }
        else
        {
            vbNotIncludedMP[i] = false;
        }
    }

    // [GBA] Markers
    for (const auto &vpMarker : allMarkersVec)
    {
        // Adding a vertex for each marker
        g2o::VertexSE3Expmap *vMarker = new g2o::VertexSE3Expmap();
        vMarker->setEstimate(g2o::SE3Quat(
            vpMarker->getGlobalPose().unit_quaternion().cast<double>(),
            vpMarker->getGlobalPose().translation().cast<double>()));
        int opIdG = maxOpId + nMarkers;
        vMarker->setId(opIdG);
        optimizer.addVertex(vMarker);
        nMarkers++;

        // Setting the Global Optimization ID for the marker
        vpMarker->setOpIdG(opIdG);

        /*!
         * The edge used to connect a Marker vertex (SE3) to a KeyFrame vertex
         * (SE3) 🚧 [vS-Graphs v.2.0] This edge is not used anymore, in contrast
         * to the previous version. [Note]: it creates constraint for six
         * measurements, i.e., (x, y, z, roll, pitch, yaw)
         */
    }

    maxOpId += nMarkers;

    // [GBA] Planes
    for (const auto &vpPlane : allPlanesVec)
    {
        // Skip undefined planes (if not wall for now)
        if (vpPlane->getPlaneType() ==
            geometric::Plane::PlaneVariant::UNDEFINED)
            continue;
        // Adding a vertex for each plane
        g2o::VertexPlane *vPlane = new g2o::VertexPlane();
        int               opIdG  = maxOpId + nPlanes;
        vPlane->setId(opIdG);
        vPlane->setEstimate(vpPlane->getGlobalEquation());
        if (p_sysParams->optimization.marginalizePlanes)
            vPlane->setMarginalized(true);
        optimizer.addVertex(vPlane);
        nPlanes++;

        // Setting the global optimization ID for the plane
        vpPlane->setOpIdG(opIdG);

        // Adding an edge between the plane and the keyframes
        const map<KeyFrame *, vs_graphs::core::geometric::Plane::Observation>
            observations = vpPlane->getObservations();
        for (map<KeyFrame *,
                 vs_graphs::core::geometric::Plane::Observation>::const_iterator
                 obsId  = observations.begin(),
                 obLast = observations.end();
             obsId != obLast;
             obsId++)
        {
            KeyFrame                                      *pKFi = obsId->first;
            vs_graphs::core::geometric::Plane::Observation obs  = obsId->second;

            if (pKFi->isBad())
            {
                std::cout
                    << "[Optimizer] Bad KeyFrame detected for GBA! Skipping..."
                    << std::endl;
                vpPlane->eraseObservation(pKFi);
                continue;
            }

            g2o::Plane3D planeLocalEquation = obs.localPlane;

            if (optimizer.vertex(opIdG) && optimizer.vertex(pKFi->mnId))
            {
                if (p_sysParams->optimization.planeKf.enabled)
                {
                    vs_graphs::core::EdgeVertexPlaneProjectSE3KF *e =
                        new vs_graphs::core::EdgeVertexPlaneProjectSE3KF();
                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(opIdG)));
                    e->setInformation(
                        Eigen::Matrix<double, 3, 3>::Identity() *
                        obs.confidence *
                        p_sysParams->optimization.planeKf.informationGain);
                    e->setMeasurement(planeLocalEquation);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                    optimizer.addEdge(e);
                }

                // adding plane-point constraints
                if (p_sysParams->optimization.planePoint.enabled)
                {
                    // get the class index of the plane
                    int clsCloudIdx =
                        utils::utils::Utils::getClassIdFromPlaneType(
                            vpPlane->getPlaneType());
                    if (clsCloudIdx != -1)
                    {
                        // add the plane-point constraint
                        vs_graphs::core::EdgeSE3KFPointToPlane *e =
                            new vs_graphs::core::EdgeSE3KFPointToPlane();
                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(pKFi->mnId)));
                        e->setVertex(
                            1,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(opIdG)));
                        e->setInformation(
                            Eigen::Matrix<double, 1, 1>::Identity() *
                            obs.confidence *
                            p_sysParams->optimization.planePoint
                                .informationGain);
                        e->setMeasurement(obs.pointPlaneConstraintMatrix);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber1D);
                        optimizer.addEdge(e);
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

    maxOpId += nPlanes;

    // [GBA] Rooms
    for (const auto &pMapRoom : vpRooms)
    {
        try
        {
            // Variables
            std::vector<vs_graphs::core::geometric::Plane *> walls =
                pMapRoom->getWalls();

            // No need to optimize if there are no walls
            if (walls.empty())
                continue;

            // Adding a vertex for each room
            g2o::VertexSE3Expmap *vrtxRoom = new g2o::VertexSE3Expmap();

            // Setting the local optimization ID for the room
            int opIdG = maxOpId + nRooms;
            vrtxRoom->setId(opIdG);
            pMapRoom->setOpIdG(opIdG);
            nRooms++;

            // Initialize the room vertex (centroid estimate)
            vrtxRoom->setEstimate(
                g2o::SE3Quat(Eigen::Quaterniond::Identity(),
                             pMapRoom->getCentroid().cast<double>()));
            vrtxRoom->setFixed(true);
            optimizer.addVertex(vrtxRoom);

            /*
             * The legacy multi-plane room projection factor averages closest
             * points from every wall. That estimate is not the center of a
             * bounded room and can pull otherwise valid wall planes during a
             * map re-merge. Keep the front-end room centroid fixed and retain
             * only the well-defined plane parallel/perpendicular constraints
             * below.
             */

            // Optimizing the parallel walls of the room
            for (size_t i = 0; i < walls.size(); i++)
                for (size_t j = i + 1; j < walls.size(); j++)
                {
                    vs_graphs::core::geometric::Plane *wall1 = walls[i];
                    vs_graphs::core::geometric::Plane *wall2 = walls[j];

                    // If the same wall, skip
                    if (wall1->getId() == wall2->getId())
                        continue;

                    // Check if the walls are parallel
                    if (utils::utils::Utils::arePlanesParallel(wall1, wall2))
                    {
                        // If they are parallel, check if they are facing each
                        // other
                        if (utils::utils::Utils::arePlanesFacingEachOther(
                                wall1,
                                wall2))
                        {
                            // Variables
                            int opId1 = wall1->getOpIdG();
                            int opId2 = wall2->getOpIdG();

                            if (optimizer.vertex(opIdG) &&
                                optimizer.vertex(opId1) &&
                                optimizer.vertex(opId2))
                            {
                                // std::cout << "[Optimizer] Parallelism
                                // constraint between walls "
                                //           << wall1->getId() << " and " <<
                                //           wall2->getId() << " of room " <<
                                //           pMapRoom->getId() << std::endl;

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
                                g2o::RobustKernelHuber *rk =
                                    new g2o::RobustKernelHuber;
                                e->setRobustKernel(rk);
                                rk->setDelta(thHuber1D);
                                optimizer.addEdge(e);
                            }
                        }
                    }

                    // Check if the walls are perpendicular
                    if (utils::utils::Utils::arePlanesPerpendicular(wall1,
                                                                    wall2))
                    {
                        // Variables
                        int opId1 = wall1->getOpIdG();
                        int opId2 = wall2->getOpIdG();

                        if (optimizer.vertex(opIdG) &&
                            optimizer.vertex(opId1) && optimizer.vertex(opId2))
                        {
                            // std::cout << "[Optimizer] Perpendicularity
                            // constraint between walls "
                            //           << wall1->getId() << " and " <<
                            //           wall2->getId() << " of room " <<
                            //           pMapRoom->getId() << std::endl;

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
                            g2o::RobustKernelHuber *rk =
                                new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuber1D);
                            optimizer.addEdge(e);
                        }
                    }
                }
        }
        catch (std::exception &e)
        {
            std::cerr << "[Optimizer] Error while globally optimizing room: "
                      << e.what() << std::endl;
            continue;
        }
    }

    maxOpId += nRooms;

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
    optimizer.optimize(nIterations);
    optimizer.removePreIterationAction(&stopBridge);
    optimizer.removePostIterationAction(&stopBridge);
    Verbose::printMess("BA: End of the optimization",
                       Verbose::VERBOSITY_NORMAL);

    // [GBA] Globally optimized KeyFrames
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        g2o::VertexSE3Expmap *vSE3 =
            static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));

        g2o::SE3Quat SE3quat = vSE3->estimate();
        if (pMap->getOriginKeyFrame() &&
            nLoopKF == pMap->getOriginKeyFrame()->mnId)
        {
            pKF->setPose(Sophus::SE3f(SE3quat.rotation().cast<float>(),
                                      SE3quat.translation().cast<float>()));
        }
        else
        {
            pKF->tcwGBA =
                Sophus::SE3d(SE3quat.rotation(), SE3quat.translation())
                    .cast<float>();
            pKF->baGlobalKeyFrameId = nLoopKF;

            Sophus::SE3f    mTwc        = pKF->getPoseInverse();
            Sophus::SE3f    mTcGBA_c    = pKF->tcwGBA * mTwc;
            Eigen::Vector3f vector_dist = mTcGBA_c.translation();
            double          dist        = vector_dist.norm();
            if (dist > 1)
            {
                int numMonoBadPoints = 0, numMonoOptPoints = 0;
                int numStereoBadPoints = 0, numStereoOptPoints = 0;
                vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;

                for (size_t i2 = 0, iend = vpEdgesMono.size(); i2 < iend; i2++)
                {
                    vs_graphs::core::EdgeSE3ProjectXYZ *e = vpEdgesMono[i2];
                    MapPoint *pMP     = vpMapPointEdgeMono[i2];
                    KeyFrame *pKFedge = edgeSourceKeyFrame(vpEdgeKFMono, i2);

                    if (pKFedge == nullptr || pKF != pKFedge)
                    {
                        continue;
                    }

                    if (pMP->isBad())
                        continue;

                    if (e->chi2() > 5.991 || !e->isDepthPositive())
                    {
                        numMonoBadPoints++;
                    }
                    else
                    {
                        numMonoOptPoints++;
                        vpMonoMPsOpt.push_back(pMP);
                    }
                }

                for (size_t i2 = 0, iend = vpEdgesStereo.size(); i2 < iend;
                     i2++)
                {
                    g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i2];
                    MapPoint *pMP                   = vpMapPointEdgeStereo[i2];
                    KeyFrame *pKFedge = edgeSourceKeyFrame(vpEdgeKFStereo, i2);

                    if (pKFedge == nullptr || pKF != pKFedge)
                    {
                        continue;
                    }

                    if (pMP->isBad())
                        continue;

                    if (e->chi2() > 7.815 || !e->isDepthPositive())
                    {
                        numStereoBadPoints++;
                    }
                    else
                    {
                        numStereoOptPoints++;
                        vpStereoMPsOpt.push_back(pMP);
                    }
                }
            }
        }
    }

    // [GBA] Globally optimized MapPoints
    for (size_t i = 0; i < vpMP.size(); i++)
    {
        if (vbNotIncludedMP[i])
            continue;

        MapPoint *pMP = vpMP[i];

        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(
            optimizer.vertex(pMP->mnId + maxKFid + 1));

        if (nLoopKF == pMap->getOriginKeyFrame()->mnId)
        {
            pMP->setWorldPos(vPoint->estimate().cast<float>());
            pMP->updateNormalAndDepth();
        }
        else
        {
            pMP->posGBA             = vPoint->estimate().cast<float>();
            pMP->baGlobalKeyFrameId = nLoopKF;
        }
    }

    /*
     * Apply semantic vertices immediately only for the initial-map BA. A
     * loop-triggered GBA runs on a worker and must not mutate semantic state
     * before LoopClosing validates its generation and acquires the semantic
     * transaction lock. The validated post-GBA deformation pass updates
     * markers and rooms for that case.
     */
    if (nLoopKF == pMap->getOriginKeyFrame()->mnId)
    {
        // [GBA] Globally optimized markers
        for (semantic::Marker *p_marker : allMarkersVec)
        {
            g2o::VertexSE3Expmap *p_markerVertex =
                static_cast<g2o::VertexSE3Expmap *>(
                    optimizer.vertex(p_marker->getOpIdG()));

            if (p_markerVertex == nullptr)
            {
                continue;
            }

            const g2o::SE3Quat markerPose_MarkerToWorld =
                p_markerVertex->estimate();

            p_marker->setGlobalPose(Sophus::SE3f(
                markerPose_MarkerToWorld.rotation().cast<float>(),
                markerPose_MarkerToWorld.translation().cast<float>()));
        }
    }

    // [GBA] Globally optimized planes
    for (auto &vpPlane : allPlanesVec)
    {
        if (optimizer.vertex(vpPlane->getOpIdG()))
        {
            g2o::VertexPlane *vPlane = static_cast<g2o::VertexPlane *>(
                optimizer.vertex(vpPlane->getOpIdG()));

            if (nLoopKF == pMap->getOriginKeyFrame()->mnId)
            {
                /*
                 * Keep the finite wall cloud, centroid, bounds, octree, and
                 * optimized equation in one frame. Updating only the equation
                 * leaves the displayed wall and every geometric association at
                 * the pre-BA pose.
                 */
                vpPlane->alignGeometryToEquation(vPlane->estimate());
            }
            else
            {
                vpPlane->planeGBA           = vPlane->estimate();
                vpPlane->baGlobalKeyFrameId = nLoopKF;
            }
        }
    }

    // [GBA] Globally optimized rooms
    if (nLoopKF == pMap->getOriginKeyFrame()->mnId)
    {
        for (semantic::Room *p_room : vpRooms)
        {
            try
            {
                g2o::VertexSE3Expmap *p_roomVertex =
                    static_cast<g2o::VertexSE3Expmap *>(
                        optimizer.vertex(p_room->getOpIdG()));

                if (p_roomVertex != nullptr)
                {
                    p_room->setCentroid(p_roomVertex->estimate().translation());
                }
            }
            catch (const std::exception &exception)
            {
                std::cerr << "[Optimizer] Error while updating optimized room: "
                          << exception.what() << std::endl;
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
