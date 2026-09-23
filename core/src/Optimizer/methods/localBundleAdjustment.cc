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
#include "Utils/Utils/objects/Utils.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void Optimizer::localBundleAdjustment(vs_graphs::core::KeyFrame *pKF,
                                      bool                      *pbStopFlag,
                                      Map                       *pMap,
                                      int                       &countFixedKF,
                                      int                       &num_OptKF,
                                      int                       &num_MPs,
                                      int                       &num_edges,
                                      double                     markerImpact)
{
    // System parameters
    vs_graphs::core::types::SystemParams *p_sysParams =
        vs_graphs::core::types::SystemParams::getParams();

    // Variables
    countFixedKF = 0;
    num_OptKF    = 0;
    num_MPs      = 0;
    num_edges    = 0;
    std::list<vs_graphs::core::semantic::Room *>    localRoomList;
    vs_graphs::core::Map                           *pCurrentMap = pKF->getMap();
    std::list<vs_graphs::core::geometric::Plane *>  localPlaneList;
    std::list<vs_graphs::core::semantic::Marker *>  localMarkerList;
    std::list<vs_graphs::core::semantic::Passage *> localPassageList;
    std::list<vs_graphs::core::KeyFrame *>          localKeyFrameList;
    std::list<vs_graphs::core::MapPoint *>          localMapPointList;
    std::vector<vs_graphs::core::KeyFrame *>        neighborKeyFrameVec;
    std::vector<vs_graphs::core::semantic::Room *>  allRooms =
        pCurrentMap->getAllRooms();
    std::vector<vs_graphs::core::semantic::Floor *> allFloors =
        pCurrentMap->getAllFloors();

    // Unorderd maps to keep track of the local entities
    std::unordered_map<int, bool> localPlaneId;
    std::unordered_map<int, bool> localMarkerId;
    std::unordered_map<int, bool> localDoorwayId;
    std::unordered_map<int, bool> localMapPointId;
    std::unordered_map<int, bool> localKeyFrameId;

    // [LBA] Initialize the KeyFrame-related variables
    localKeyFrameList.push_back(pKF);
    localKeyFrameId[pKF->mnId] = true;
    pKF->baLocalKeyFrameId     = pKF->mnId;

    // [LBA] Fill in the neighbor KeyFrames
    if (p_sysParams->planeBasedCovisibility.enabled)
        // Get the KeyFrames that see the same planes
        neighborKeyFrameVec = pKF->getBestCovisibilityKeyFrames(
            p_sysParams->planeBasedCovisibility.maxKeyframes);
    else
        // Get the KeyFrames that see the same MapPoints
        neighborKeyFrameVec = pKF->getVectorCovisibleKeyFrames();

    // Iterate through all neighboring KeyFrames
    for (int idx = 0, idxEnd = neighborKeyFrameVec.size(); idx < idxEnd; idx++)
    {
        // Get the current KeyFrame's neighbors
        vs_graphs::core::KeyFrame *pKFi = neighborKeyFrameVec[idx];
        // Mark the KeyFrame as a part of the current LBA
        pKFi->baLocalKeyFrameId = pKF->mnId;
        // If the KeyFrame is proper, add it to the list of local KeyFrames for
        // LBA
        if (!pKFi->isBad() && pKFi->getMap() == pCurrentMap)
        {
            localKeyFrameList.push_back(pKFi);
            localKeyFrameId[pKFi->mnId] = true;
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
        vs_graphs::core::KeyFrame                       *pKFi = *lit;
        std::vector<vs_graphs::core::geometric::Plane *> localPlanesVec =
            pKFi->getMapPlanes();
        std::vector<vs_graphs::core::semantic::Marker *> localMarkersVec =
            pKFi->getMapMarkers();
        std::vector<vs_graphs::core::semantic::Passage *> localDoorwaysVec =
            pKFi->getMapPassages();
        std::vector<vs_graphs::core::MapPoint *> localMapPointsVec =
            pKFi->getMapPointMatches();

        // If the KeyFrame is the initial KeyFrame of the map, mark that as a
        // fixed KeyFrame
        if (pKFi->mnId == pMap->getInitKeyFrameId())
            countFixedKF = 1;

        // [LBA] Loop through all the MapPoints and prepare them for LBA
        for (std::vector<vs_graphs::core::MapPoint *>::iterator
                 vit  = localMapPointsVec.begin(),
                 vend = localMapPointsVec.end();
             vit != vend;
             vit++)
        {
            // Variables
            vs_graphs::core::MapPoint *pMP = *vit;

            // If the MapPoint is proper, add it to the list of local MapPoints
            // for LBA
            if (pMP)
                if (!pMP->isBad() && pMP->getMap() == pCurrentMap)
                {
                    if (pMP->baLocalKeyFrameId != pKF->mnId)
                    {
                        localMapPointList.push_back(pMP);
                        localMapPointId[pMP->mnId] = true;
                        pMP->baLocalKeyFrameId     = pKF->mnId;
                    }
                }
        }

        // [LBA] Loop through all the Markers and prepare them for LBA
        for (std::vector<vs_graphs::core::semantic::Marker *>::iterator
                 idx  = localMarkersVec.begin(),
                 vend = localMarkersVec.end();
             idx != vend;
             idx++)
        {
            if (localMarkerId.find((*idx)->getId()) == localMarkerId.end())
            {
                vs_graphs::core::semantic::Marker *marker = *idx;
                localMarkerList.push_back(marker);
                localMarkerId[marker->getId()] = true;
            }
        }

        // [LBA] Loop through all the Planes and prepare them for LBA
        for (std::vector<vs_graphs::core::geometric::Plane *>::iterator
                 idx  = localPlanesVec.begin(),
                 vend = localPlanesVec.end();
             idx != vend;
             idx++)
        {
            vs_graphs::core::geometric::Plane *plane = *idx;
            // If the plane does not exist, skip it
            if (!plane)
                continue;
            // If the plane is not known, do not add it to the local map
            if (plane->getPlaneType() ==
                geometric::Plane::PlaneVariant::UNDEFINED)
                continue;
            // Otherwise, add the plane to the local map
            if (localPlaneId.find(plane->getId()) == localPlaneId.end())
            {
                localPlaneList.push_back(plane);
                localPlaneId[plane->getId()] = true;
            }
        }

        // [LBA] Loop through all the Doorways and prepare them for LBA
        for (std::vector<vs_graphs::core::semantic::Passage *>::iterator
                 idx  = localDoorwaysVec.begin(),
                 vend = localDoorwaysVec.end();
             idx != vend;
             idx++)
        {
            if (localDoorwayId.find((*idx)->getId()) == localDoorwayId.end())
            {
                vs_graphs::core::semantic::Passage *doorway = *idx;
                localPassageList.push_back(doorway);
                localDoorwayId[doorway->getId()] = true;
            }
        }
    }

    // [LBA] Among all rooms, filter only the ones with a wall in LBA
    for (const auto &room : allRooms)
    {
        // Get the walls of the room
        std::vector<vs_graphs::core::geometric::Plane *> roomWalls =
            room->getWalls();
        // Add the room to the local map if any of the walls are in the local
        // map
        for (const auto &wall : roomWalls)
            if (localPlaneId.find(wall->getId()) != localPlaneId.end())
            {
                localRoomList.push_back(room);
                break;
            }
    }

    // [LBA] Loop through all the local Rooms to add all their walls to LBA
    std::list<vs_graphs::core::geometric::Plane *> lRecentLocalMapPlanes;
    for (std::list<vs_graphs::core::semantic::Room *>::iterator
             idx  = localRoomList.begin(),
             vend = localRoomList.end();
         idx != vend;
         idx++)
    {
        std::vector<vs_graphs::core::geometric::Plane *> roomWalls =
            (*idx)->getWalls();
        for (const auto &roomWall : roomWalls)
        {
            if (localPlaneId.find(roomWall->getId()) == localPlaneId.end())
            {
                localPlaneList.push_back(roomWall);
                localPlaneId[roomWall->getId()] = true;
                lRecentLocalMapPlanes.push_back(roomWall);
            }
        }
    }

    // [LBA] Loop through the recently added planes, get all the KeyFrames and
    // add them
    std::list<vs_graphs::core::KeyFrame *> lRecentLocalMapKeyFrames;
    for (std::list<vs_graphs::core::geometric::Plane *>::iterator
             idx  = lRecentLocalMapPlanes.begin(),
             vend = lRecentLocalMapPlanes.end();
         idx != vend;
         idx++)
    {
        std::map<vs_graphs::core::KeyFrame *,
                 vs_graphs::core::geometric::Plane::Observation>
            planeObservations = (*idx)->getObservations();
        for (std::map<
                 vs_graphs::core::KeyFrame *,
                 vs_graphs::core::geometric::Plane::Observation>::const_iterator
                 obsId  = planeObservations.begin(),
                 obLast = planeObservations.end();
             obsId != obLast;
             obsId++)
        {
            vs_graphs::core::KeyFrame *pKFi = obsId->first;
            if (!pKFi->isBad() && pKFi->getMap() == pCurrentMap)
            {
                if (localKeyFrameId.find(pKFi->mnId) == localKeyFrameId.end())
                {
                    localKeyFrameList.push_back(pKFi);
                    localKeyFrameId[pKFi->mnId] = true;
                    pKFi->baLocalKeyFrameId     = pKF->mnId;
                    lRecentLocalMapKeyFrames.push_back(pKFi);
                }
            }
        }
    }

    // [LBA] Loop through the recently added keyframes, get all the map points
    // and add them
    for (std::list<vs_graphs::core::KeyFrame *>::iterator
             idx  = lRecentLocalMapKeyFrames.begin(),
             vend = lRecentLocalMapKeyFrames.end();
         idx != vend;
         idx++)
    {
        std::vector<vs_graphs::core::MapPoint *> vpMPs =
            (*idx)->getMapPointMatches();
        for (std::vector<vs_graphs::core::MapPoint *>::iterator
                 vit  = vpMPs.begin(),
                 vend = vpMPs.end();
             vit != vend;
             vit++)
        {
            vs_graphs::core::MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad() && pMP->getMap() == pCurrentMap)
                {
                    if (pMP->baLocalKeyFrameId != pKF->mnId)
                    {
                        localMapPointList.push_back(pMP);
                        localMapPointId[pMP->mnId] = true;
                        pMP->baLocalKeyFrameId     = pKF->mnId;
                    }
                }
        }
    }

    // [LBA] Fixed Keyframes for MPs (Keyframes that see Local MapPoints but
    // that are not Local Keyframes)
    std::list<vs_graphs::core::KeyFrame *> lFixedCameras;
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
            vs_graphs::core::KeyFrame *pKFi = mit->first;

            if (pKFi->baLocalKeyFrameId != pKF->mnId &&
                pKFi->baFixedKeyFrameId != pKF->mnId)
            {
                pKFi->baFixedKeyFrameId = pKF->mnId;
                if (!pKFi->isBad() && pKFi->getMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Count fixed KeyFrames
    countFixedKF = lFixedCameras.size() + countFixedKF;
    if (countFixedKF == 0)
    {
        std::cout << "[Optimizer] No fixed KeyFrames found for LBA! Aborting..."
                  << std::endl;
        return;
    }

    // [LBA] Setup optimizer
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    if (pMap->isInertial())
        solver->setUserLambdaInit(100.0);

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Debug LBA
    pCurrentMap->optKeyFrameIds.clear();
    pCurrentMap->fixedKeyFrameIds.clear();

    // [LBA] Local KeyFrame vertices
    for (std::list<vs_graphs::core::KeyFrame *>::iterator
             lit  = localKeyFrameList.begin(),
             lend = localKeyFrameList.end();
         lit != lend;
         lit++)
    {
        vs_graphs::core::KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap      *vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float>         Tcw  = pKFi->getPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == pMap->getInitKeyFrameId());
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
        pCurrentMap->optKeyFrameIds.insert(pKFi->mnId);
    }
    num_OptKF = localKeyFrameList.size();

    // [LBA] Fixed KeyFrame vertices
    for (std::list<vs_graphs::core::KeyFrame *>::iterator
             lit  = lFixedCameras.begin(),
             lend = lFixedCameras.end();
         lit != lend;
         lit++)
    {
        KeyFrame             *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float>    Tcw  = pKFi->getPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
        pCurrentMap->fixedKeyFrameIds.insert(pKFi->mnId);
    }

    // [LBA] MapPoint vertices
    const int nExpectedSize =
        (localKeyFrameList.size() + lFixedCameras.size()) *
        localMapPointList.size();

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

    const int nExpectedSizePlane =
        (localKeyFrameList.size() + lFixedCameras.size()) *
        localPlaneList.size();
    vector<EdgeVertexPlaneProjectSE3KF *> vpEdgesPlane;
    vpEdgesPlane.reserve(nExpectedSizePlane);

    vector<KeyFrame *> vpEdgeKFPlane;
    vpEdgeKFPlane.reserve(nExpectedSizePlane);

    vector<geometric::Plane *> vpPlaneEdgePlane;
    vpPlaneEdgePlane.reserve(nExpectedSizePlane);

    vector<EdgeSE3KFPointToPlane *> vpEdgesPlanePoint;
    vpEdgesPlanePoint.reserve(nExpectedSizePlane);

    vector<KeyFrame *> vpEdgeKFPlanePoint;
    vpEdgeKFPlanePoint.reserve(nExpectedSizePlane);

    vector<geometric::Plane *> vpPlaneEdgePlanePoint;
    vpPlaneEdgePlanePoint.reserve(nExpectedSizePlane);

    const float thHuber1D     = sqrt(3.841);
    const float thHuberMono   = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    int nRooms    = 1;
    int nFloors   = 1;
    int nPlanes   = 1;
    int nPoints   = 0;
    int nMarkers  = 1;
    int nDoorways = 1;

    int nEdges  = 0;
    int maxOpId = 0;

    for (std::list<vs_graphs::core::MapPoint *>::iterator
             lit  = localMapPointList.begin(),
             lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        vs_graphs::core::MapPoint *pMP    = *lit;
        g2o::VertexSBAPointXYZ    *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->getWorldPos().cast<double>());
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        nPoints++;

        // Update the maxOpId to hold the biggest value
        if (id > maxOpId)
            maxOpId = id;

        const map<KeyFrame *, tuple<int, int>> observations =
            pMP->getObservations();

        // Set edges
        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (!pKFi->isBad() && pKFi->getMap() == pCurrentMap)
            {
                const int leftIndex = get<0>(mit->second);

                // Monocular observation
                if (leftIndex != -1 && pKFi->uRight[get<0>(mit->second)] < 0)
                {
                    const cv::KeyPoint &kpUn =
                        pKFi->keyPointsUndistorted[leftIndex];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    vs_graphs::core::EdgeSE3ProjectXYZ *e =
                        new vs_graphs::core::EdgeSE3ProjectXYZ();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 =
                        pKFi->invLevelSigmaSquared[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->pCamera = pKFi->p_camera;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    nEdges++;
                }
                else if (leftIndex != -1 && pKFi->uRight[get<0>(mit->second)] >=
                                                0) // Stereo observation
                {
                    const cv::KeyPoint &kpUn =
                        pKFi->keyPointsUndistorted[leftIndex];
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKFi->uRight[get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e =
                        new g2o::EdgeStereoSE3ProjectXYZ();

                    if (optimizer.vertex(id) && optimizer.vertex(pKFi->mnId))
                    {
                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(id)));
                        e->setVertex(
                            1,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 =
                            pKFi->invLevelSigmaSquared[kpUn.octave];
                        Eigen::Matrix3d Info =
                            Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;
                        e->bf = pKFi->mbf;

                        optimizer.addEdge(e);
                        vpEdgesStereo.push_back(e);
                        vpEdgeKFStereo.push_back(pKFi);
                        vpMapPointEdgeStereo.push_back(pMP);

                        nEdges++;
                    }
                }

                if (pKFi->p_camera2)
                {
                    int rightIndex = get<1>(mit->second);

                    if (rightIndex != -1 &&
                        rightIndex < (int)pKFi->keyPointsRight.size())
                    {
                        rightIndex -= pKFi->Nleft;

                        Eigen::Matrix<double, 2, 1> obs;
                        cv::KeyPoint kp = pKFi->keyPointsRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        vs_graphs::core::EdgeSE3ProjectXYZToBody *e =
                            new vs_graphs::core::EdgeSE3ProjectXYZToBody();

                        if (optimizer.vertex(id) &&
                            optimizer.vertex(pKFi->mnId))
                        {
                            e->setVertex(
                                0,
                                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex(id)));
                            e->setVertex(
                                1,
                                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                    optimizer.vertex(pKFi->mnId)));
                            e->setMeasurement(obs);
                            const float &invSigma2 =
                                pKFi->invLevelSigmaSquared[kp.octave];
                            e->setInformation(Eigen::Matrix2d::Identity() *
                                              invSigma2);

                            g2o::RobustKernelHuber *rk =
                                new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuberMono);

                            Sophus::SE3f Trl = pKFi->getRelativePoseTrl();
                            e->mTrl          = g2o::SE3Quat(
                                Trl.unit_quaternion().cast<double>(),
                                Trl.translation().cast<double>());

                            e->pCamera = pKFi->p_camera2;

                            optimizer.addEdge(e);
                            vpEdgesBody.push_back(e);
                            vpEdgeKFBody.push_back(pKFi);
                            vpMapPointEdgeBody.push_back(pMP);

                            nEdges++;
                        }
                    }
                }
            }
        }
    }
    num_edges = nEdges;

    // [LBA] Markers
    for (list<semantic::Marker *>::iterator idx  = localMarkerList.begin(),
                                            lend = localMarkerList.end();
         idx != lend;
         idx++)
    {
        // Adding a vertex for each marker
        semantic::Marker     *pMapMarker = *idx;
        g2o::VertexSE3Expmap *vMarker    = new g2o::VertexSE3Expmap();
        vMarker->setEstimate(g2o::SE3Quat(
            pMapMarker->getGlobalPose().unit_quaternion().cast<double>(),
            pMapMarker->getGlobalPose().translation().cast<double>()));
        int opId = maxOpId + nMarkers;
        vMarker->setId(opId);
        optimizer.addVertex(vMarker);
        nMarkers++;

        // Setting the local optimization ID for the marker
        pMapMarker->setOpId(opId);

        // 🚧 [vS-Graphs v.2.0] in contrast with the first version of visual
        // S-Graphs, where there was an edge between the marker and the
        // keyframe, in this version we removed that edge and added an edge
        // between the plane and the keyframe, while still keeping the edge
        // between the marker and the plane.
    }

    maxOpId += nMarkers;

    // [LBA] Planes
    for (std::list<vs_graphs::core::geometric::Plane *>::iterator
             idx  = localPlaneList.begin(),
             lend = localPlaneList.end();
         idx != lend;
         idx++)
    {
        // Variables
        vs_graphs::core::geometric::Plane *pMapPlane = *idx;
        g2o::VertexPlane                  *vPlane    = new g2o::VertexPlane();

        // Adding a vertex for each plane
        int opId = maxOpId + nPlanes;
        vPlane->setId(opId);

        if (p_sysParams->optimization.marginalizePlanes)
            vPlane->setMarginalized(true);

        g2o::Plane3D planeGlobalEquation = pMapPlane->getGlobalEquation();
        vPlane->setEstimate(planeGlobalEquation);
        optimizer.addVertex(vPlane);
        nPlanes++;

        // Setting the local optimization ID for the plane
        pMapPlane->setOpId(opId);

        // Adding edge between plane and MapPoints
        if (p_sysParams->optimization.planeMapPoint.enabled &&
            !p_sysParams->optimization.marginalizePlanes)
        {
            set<MapPoint *> sMPs = pMapPlane->getMapPoints();
            for (set<MapPoint *>::iterator lit  = sMPs.begin(),
                                           lend = sMPs.end();
                 lit != lend;
                 lit++)
            {
                MapPoint *pMP = *lit;

                if (!pMP || pMP->isBad())
                    continue;

                if (optimizer.vertex(opId) &&
                    optimizer.vertex(pMP->mnId + maxKFid + 1))
                {
                    vs_graphs::core::EdgeVertexPlaneProjectPointXYZ *e =
                        new vs_graphs::core::EdgeVertexPlaneProjectPointXYZ();
                    e->setVertex(
                        0,
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(pMP->mnId + maxKFid + 1)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(opId)));
                    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                                      p_sysParams->optimization.planeMapPoint
                                          .informationGain);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber1D);
                    optimizer.addEdge(e);
                    nEdges++;
                }
            }
        }

        // Adding an edge between the plane and the keyframes
        const map<KeyFrame *, vs_graphs::core::geometric::Plane::Observation>
            observations = pMapPlane->getObservations();
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
                    << "[Optimizer] Bad KeyFrame detected for LBA! Skipping..."
                    << std::endl;
                pMapPlane->eraseObservation(pKFi);
                continue;
            }

            if (pKFi->getMap() != pCurrentMap)
            {
                std::cout << "[Optimizer] KeyFrame is not in the current map! "
                             "Skipping..."
                          << std::endl;
                continue;
            }

            if (optimizer.vertex(opId) && optimizer.vertex(pKFi->mnId))
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
                                     optimizer.vertex(opId)));
                    e->setInformation(
                        Eigen::Matrix<double, 3, 3>::Identity() *
                        obs.confidence *
                        p_sysParams->optimization.planeKf.informationGain);
                    e->setMeasurement(obs.localPlane);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);
                    optimizer.addEdge(e);
                    nEdges++;

                    vpEdgesPlane.push_back(e);
                    vpEdgeKFPlane.push_back(pKFi);
                    vpPlaneEdgePlane.push_back(pMapPlane);
                }

                // Adding plane-point constraints
                if (p_sysParams->optimization.planePoint.enabled)
                {
                    // Get the class index of the plane
                    int clsCloudIdx =
                        utils::utils::Utils::getClassIdFromPlaneType(
                            pMapPlane->getPlaneType());
                    if (clsCloudIdx != -1)
                    {
                        // Add the plane-point constraint
                        vs_graphs::core::EdgeSE3KFPointToPlane *e =
                            new vs_graphs::core::EdgeSE3KFPointToPlane();
                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(pKFi->mnId)));
                        e->setVertex(
                            1,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(opId)));
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
                        nEdges++;

                        vpEdgesPlanePoint.push_back(e);
                        vpEdgeKFPlanePoint.push_back(pKFi);
                        vpPlaneEdgePlanePoint.push_back(pMapPlane);
                    }
                }
            }
        }
    }

    maxOpId += nPlanes;

    // [LBA] Rooms
    for (std::list<vs_graphs::core::semantic::Room *>::iterator
             idx  = localRoomList.begin(),
             lend = localRoomList.end();
         idx != lend;
         idx++)
    {
        try
        {
            // Variables
            vs_graphs::core::semantic::Room                 *pMapRoom = *idx;
            std::vector<vs_graphs::core::geometric::Plane *> walls =
                pMapRoom->getWalls();

            // No need to optimize if there are no walls
            if (walls.empty())
                continue;

            // Adding a vertex for each room
            g2o::VertexSE3Expmap *vrtxRoom = new g2o::VertexSE3Expmap();

            // Setting the local optimization ID for the room
            int opId = maxOpId + nRooms;
            vrtxRoom->setId(opId);
            pMapRoom->setOpId(opId);
            nRooms++;

            // Initialize the room vertex (centroid estimate)
            vrtxRoom->setEstimate(
                g2o::SE3Quat(Eigen::Quaterniond::Identity(),
                             pMapRoom->getCentroid().cast<double>()));
            vrtxRoom->setFixed(true);
            optimizer.addVertex(vrtxRoom);

            /*
             * See the global BA path above: the legacy room-centering factor
             * is deliberately disabled because it can deform wall geometry.
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
                            int opId1 = wall1->getOpId();
                            int opId2 = wall2->getOpId();

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
                        int opId1 = wall1->getOpId();
                        int opId2 = wall2->getOpId();

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
            std::cerr << "[Optimizer] Error while locally optimizing room: "
                      << e.what() << std::endl;
            continue;
        }
    }

    maxOpId += nRooms;

    // abort if no edges
    if (nEdges == 0)
    {
        Verbose::printMess(
            "LM-LBA: There are 0 edges in the optimizations, LBA aborted",
            Verbose::VERBOSITY_NORMAL);
        return;
    }

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() +
                     vpEdgesStereo.size());

    vector<pair<KeyFrame *, geometric::Plane *>> vToErasePlane;
    vToErasePlane.reserve(vpEdgesPlane.size() * 2);

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        vs_graphs::core::EdgeSE3ProjectXYZ *e   = vpEdgesMono[i];
        MapPoint                           *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++)
    {
        vs_graphs::core::EdgeSE3ProjectXYZToBody *e   = vpEdgesBody[i];
        MapPoint                                 *pMP = vpMapPointEdgeBody[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFBody[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ *e   = vpEdgesStereo[i];
        MapPoint                     *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for (size_t i = 0, iend = vpEdgesPlane.size(); i < iend; i++)
    {
        vs_graphs::core::EdgeVertexPlaneProjectSE3KF *e = vpEdgesPlane[i];
        geometric::Plane *vpPlane                       = vpPlaneEdgePlane[i];

        if (e->chi2() > 7.815 || !e->isDistanceCorrect())
        {

            // if not already in ToErase, add it
            std::pair<KeyFrame *, geometric::Plane *> pKFPlane =
                make_pair(vpEdgeKFPlane[i], vpPlane);
            if (std::find(vToErasePlane.begin(),
                          vToErasePlane.end(),
                          pKFPlane) == vToErasePlane.end())
                vToErasePlane.push_back(pKFPlane);
        }
    }

    for (size_t i = 0, iend = vpEdgesPlanePoint.size(); i < iend; i++)
    {
        vs_graphs::core::EdgeSE3KFPointToPlane *e = vpEdgesPlanePoint[i];
        geometric::Plane *vpPlane                 = vpPlaneEdgePlanePoint[i];

        if (e->chi2() > 3.841 || !e->isDistanceCorrect())
        {
            // if not already in ToErase, add it
            std::pair<KeyFrame *, geometric::Plane *> pKFPlane =
                make_pair(vpEdgeKFPlanePoint[i], vpPlane);
            if (std::find(vToErasePlane.begin(),
                          vToErasePlane.end(),
                          pKFPlane) == vToErasePlane.end())
                vToErasePlane.push_back(pKFPlane);
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty())
    {
        for (size_t i = 0; i < vToErase.size(); i++)
        {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->eraseMapPointMatch(pMPi);
            pMPi->eraseObservation(pKFi);
        }
    }

    if (!vToErasePlane.empty())
    {
        for (size_t i = 0; i < vToErasePlane.size(); i++)
        {
            KeyFrame         *pKFi   = vToErasePlane[i].first;
            geometric::Plane *pPlane = vToErasePlane[i].second;
            pPlane->eraseObservation(pKFi);
            pKFi->removeMapPlane(pPlane);
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
            KeyFrame             *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(
                optimizer.vertex(pKFi->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                             SE3quat.translation().cast<float>());
            pKFi->setPose(Tiw);
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
            MapPoint               *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint =
                static_cast<g2o::VertexSBAPointXYZ *>(
                    optimizer.vertex(pMP->mnId + maxKFid + 1));
            pMP->setWorldPos(vPoint->estimate().cast<float>());
            pMP->updateNormalAndDepth();
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
    for (list<semantic::Marker *>::iterator idx  = localMarkerList.begin(),
                                            lend = localMarkerList.end();
         idx != lend;
         idx++)
    {
        try
        {
            semantic::Marker     *pMapMarker = *idx;
            g2o::VertexSE3Expmap *vMarker = static_cast<g2o::VertexSE3Expmap *>(
                optimizer.vertex(pMapMarker->getOpId()));
            g2o::SE3Quat SE3quat = vMarker->estimate();
            Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                             SE3quat.translation().cast<float>());
            pMapMarker->setGlobalPose(Tiw);
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
             idx  = localPlaneList.begin(),
             lend = localPlaneList.end();
         idx != lend;
         idx++)
    {
        try
        {
            vs_graphs::core::geometric::Plane *pMapPlane = *idx;
            g2o::VertexPlane *vPlane = static_cast<g2o::VertexPlane *>(
                optimizer.vertex(pMapPlane->getOpId()));
            g2o::Plane3D planePlane = vPlane->estimate();
            pMapPlane->setGlobalEquation(planePlane);
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

    pMap->increaseChangeIndex();
}

} // namespace core
} // namespace vs_graphs
