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

#include "LoopClosing.h"

#include "Optimizer.h"
#include "Semantic/SemanticVerify.h"

#include "../private_functions.h"

#include <chrono>
#include <mutex>
#include <thread>

namespace vs_graphs
{
namespace core
{

semantic::SemanticMergeDecision LoopClosing::mergeLocalInertial()
{
    /* Reject stale place-recognition candidates before stopping workers */
    if (p_currentKF == nullptr || p_mergeMatchedKF == nullptr ||
        p_currentKF->isBad() || p_mergeMatchedKF->isBad())
    {
        return semantic::SemanticMergeDecision::REJECT;
    }

    Map *pCurrentMap = p_currentKF->getMap();
    Map *pMergeMap   = p_mergeMatchedKF->getMap();

    if (pCurrentMap == nullptr || pMergeMap == nullptr ||
        pCurrentMap == pMergeMap || pCurrentMap->isBad() ||
        pMergeMap->isBad() || !p_atlas->isActiveMap(pCurrentMap) ||
        !p_atlas->isActiveMap(pMergeMap))
    {
        return semantic::SemanticMergeDecision::REJECT;
    }

    int numTemporalKFs = 11; // [TODO] Set by parameter

    // Relationship to rebuild the essential graph, it is used two times, first
    // in the local window and later in the rest of the map
    KeyFrame *pNewChild;
    KeyFrame *pNewParent;

    vector<KeyFrame *> vpLocalCurrentWindowKFs;
    vector<KeyFrame *> vpMergeConnectedKFs;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;

    // Flag that is true only when we stopped a running BA, in this case we need
    // relaunch at the end of the merge
    bool bRelaunchBA = false;

    /* Stop and reclaim GBA before either map changes frame or ownership. */
    bRelaunchBA = stopGlobalBundleAdjustment();

    p_localMapper->requestStop();

    // Wait until Local Mapping has effectively stopped
    while (!p_localMapper->isStopped())
    {
        usleep(1000);
    }

    /* Keep the inertial semantic transfer atomic for the complete merge. */
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();

    if (p_currentKF->isBad() || p_mergeMatchedKF->isBad() ||
        p_currentKF->getMap() != pCurrentMap ||
        p_mergeMatchedKF->getMap() != pMergeMap ||
        !p_atlas->isActiveMap(pCurrentMap) || !p_atlas->isActiveMap(pMergeMap))
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        return semantic::SemanticMergeDecision::REJECT;
    }

    const semantic::SemanticMergeGateResult semanticMergeGate =
        semantic::SemanticVerify::evaluateMapMergeGate(
            pCurrentMap,
            pMergeMap,
            oldCorrectedPose.inverse(),
            semantic::SemanticVerify::configFromSystemParams());
    const std::string floorVerificationResult = semanticMergeGate.floorDecision;
    std::cout << "[SemanticMergeGate] surviving_map=" << pCurrentMap->getId()
              << " absorbed_map=" << pMergeMap->getId() << " decision="
              << semantic::SemanticVerify::mergeDecisionName(
                     semanticMergeGate.decision)
              << " reason="
              << semantic::SemanticVerify::mergeReasonName(
                     semanticMergeGate.reason)
              << " shared_rooms=" << semanticMergeGate.sharedRoomCount
              << " aligned_rooms=" << semanticMergeGate.alignedRoomCount
              << " matched_walls=" << semanticMergeGate.matchedWallCount
              << " matched_passages=" << semanticMergeGate.matchedPassageCount
              << " committed=0" << std::endl;
    if (semanticMergeGate.decision != semantic::SemanticMergeDecision::ACCEPT)
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        if (bRelaunchBA)
        {
            relaunchGlobalBundleAdjustment(pCurrentMap);
        }
        return semanticMergeGate.decision;
    }

    {
        float        s_on = oldCorrectedPose.scale();
        Sophus::SE3f T_on(oldCorrectedPose.rotation().cast<float>(),
                          oldCorrectedPose.translation().cast<float>());

        std::unique_lock<std::mutex> currentMapUpdateLock(
            pCurrentMap->mMutexMapUpdate);

        p_localMapper->emptyQueue();

        std::chrono::steady_clock::time_point t2 =
            std::chrono::steady_clock::now();
        bool bScaleVel = false;
        if (s_on != 1)
            bScaleVel = true;
        pCurrentMap->applyScaledRotation(T_on, s_on, bScaleVel);
        p_tracker->updateFrameIMU(s_on,
                                  p_currentKF->getImuBias(),
                                  p_tracker->getLastKeyFrame());

        std::chrono::steady_clock::time_point t3 =
            std::chrono::steady_clock::now();
    }

    const int numKFnew = pCurrentMap->getKeyFrameCount();

    if ((p_tracker->sensor == System::IMU_MONOCULAR ||
         p_tracker->sensor == System::IMU_STEREO ||
         p_tracker->sensor == System::IMU_RGBD) &&
        !pCurrentMap->getInertialBA2())
    {
        /* Map is not completly initialized */
        Eigen::Vector3d bg, ba;
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        Optimizer::inertialOptimization(pCurrentMap, bg, ba);
        IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
        std::unique_lock<std::mutex> currentMapUpdateLock(
            pCurrentMap->mMutexMapUpdate);
        p_tracker->updateFrameIMU(1.0f, b, p_tracker->getLastKeyFrame());

        /* Set map initialized */
        pCurrentMap->setInertialBA2();
        pCurrentMap->setInertialBA1();
        pCurrentMap->setImuInitialized();
    }

    /* Retain imported room identities for post-optimization reconciliation. */
    std::vector<semantic::Room *> importedRooms;

    /* Load KFs and MPs from merge map */
    {
        /*!
         * Acquire both map-update mutexes without imposing an unsafe order.
         *
         * @note        Get Merge Map Mutex and stop tracking.
         */
        std::scoped_lock mapUpdateLocks(pCurrentMap->mMutexMapUpdate,
                                        pMergeMap->mMutexMapUpdate);

        vector<KeyFrame *>         vpMergeMapKFs = pMergeMap->getAllKeyFrames();
        vector<MapPoint *>         vpMergeMapMPs = pMergeMap->getAllMapPoints();
        vector<geometric::Plane *> vpMergeMapPlanes = pMergeMap->getAllPlanes();
        vector<semantic::Marker *> vpMergeMapMarkers =
            pMergeMap->getAllMarkers();
        vector<vs_graphs::core::semantic::Passage *> vpMergeMapPassages =
            pMergeMap->getAllPassages();
        vector<semantic::Room *> vpMergeMapDetectedRooms =
            pMergeMap->getAllDetectedMapRooms();
        vector<semantic::Room *> vpMergeMapMarkerRooms =
            pMergeMap->getAllMarkerBasedMapRooms();
        vector<semantic::Floor *> vpMergeMapFloors = pMergeMap->getAllFloors();

        importedRooms = vpMergeMapDetectedRooms;
        importedRooms.insert(importedRooms.end(),
                             vpMergeMapMarkerRooms.begin(),
                             vpMergeMapMarkerRooms.end());

        for (KeyFrame *pKFi : vpMergeMapKFs)
        {
            if (!pKFi || pKFi->isBad() || pKFi->getMap() != pMergeMap)
            {
                continue;
            }

            // Make sure connections are updated
            pKFi->updateMap(pCurrentMap);
            pCurrentMap->addKeyFrame(pKFi);
            pMergeMap->eraseKeyFrame(pKFi);
        }

        for (MapPoint *pMPi : vpMergeMapMPs)
        {
            if (!pMPi || pMPi->isBad() || pMPi->getMap() != pMergeMap)
                continue;

            pMPi->updateMap(pCurrentMap);
            pCurrentMap->addMapPoint(pMPi);
            pMergeMap->eraseMapPoint(pMPi);
        }

        int nextPlaneId = 0;
        for (geometric::Plane *p_existingPlane : pCurrentMap->getAllPlanes())
        {
            if (p_existingPlane != nullptr)
            {
                nextPlaneId =
                    std::max(nextPlaneId, p_existingPlane->getId() + 1);
            }
        }

        for (geometric::Plane *p_plane : vpMergeMapPlanes)
        {
            if (p_plane == nullptr || p_plane->isBad())
            {
                continue;
            }

            p_plane->setMap(pCurrentMap);
            p_plane->setId(nextPlaneId++);
            pCurrentMap->addMapPlane(p_plane);
            pMergeMap->eraseMapPlane(p_plane);
        }

        int nextMarkerId = 0;
        for (semantic::Marker *p_existingMarker : pCurrentMap->getAllMarkers())
        {
            if (p_existingMarker != nullptr)
            {
                nextMarkerId =
                    std::max(nextMarkerId, p_existingMarker->getId() + 1);
            }
        }

        for (semantic::Marker *p_marker : vpMergeMapMarkers)
        {
            if (p_marker == nullptr)
            {
                continue;
            }

            p_marker->setMap(pCurrentMap);
            p_marker->setId(nextMarkerId++);
            pCurrentMap->addMapMarker(p_marker);
            pMergeMap->eraseMapMarker(p_marker);
        }

        for (vs_graphs::core::semantic::Passage *p_passage : vpMergeMapPassages)
        {
            if (p_passage == nullptr)
            {
                continue;
            }

            semantic::Passage *p_retainedPassage = nullptr;
            for (semantic::Passage *p_existingPassage :
                 pCurrentMap->getAllPassages())
            {
                if (p_existingPassage != nullptr &&
                    p_existingPassage->getId() == p_passage->getId())
                {
                    p_retainedPassage = p_existingPassage;
                    break;
                }
            }

            pMergeMap->eraseMapPassage(p_passage);
            if (p_retainedPassage != nullptr)
            {
                p_retainedPassage->mergeFromDuplicate(p_passage);
                for (semantic::Room *p_room : vpMergeMapDetectedRooms)
                {
                    if (p_room != nullptr)
                    {
                        p_room->replacePassageAssociation(p_passage,
                                                          p_retainedPassage);
                    }
                }
                for (semantic::Room *p_room : vpMergeMapMarkerRooms)
                {
                    if (p_room != nullptr)
                    {
                        p_room->replacePassageAssociation(p_passage,
                                                          p_retainedPassage);
                    }
                }
                p_passage->setBad();
                continue;
            }

            p_passage->setMap(pCurrentMap);
            pCurrentMap->addMapPassage(p_passage);
        }

        for (semantic::Room *p_room : vpMergeMapDetectedRooms)
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            p_room->setMap(pCurrentMap);
            pCurrentMap->addDetectedMapRoom(p_room);
            pMergeMap->eraseDetectedMapRoom(p_room);
        }

        for (semantic::Room *p_room : vpMergeMapMarkerRooms)
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            p_room->setMap(pCurrentMap);
            pCurrentMap->addCandidateMapRoom(p_room);
            pMergeMap->eraseMarkerBasedMapRoom(p_room);
        }

        for (semantic::Floor *p_floor : vpMergeMapFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }

            pMergeMap->eraseMapFloor(p_floor);
            semantic::Floor *p_retainedFloor = nullptr;
            for (semantic::Floor *p_existingFloor : pCurrentMap->getAllFloors())
            {
                if (p_existingFloor != nullptr &&
                    p_existingFloor->getId() == p_floor->getId())
                {
                    p_retainedFloor = p_existingFloor;
                    break;
                }
            }
            if (p_retainedFloor != nullptr)
            {
                mergeFloorEvidenceAndRooms(p_retainedFloor, p_floor);
                continue;
            }
            p_floor->setMap(pCurrentMap);
            pCurrentMap->addMapFloor(p_floor);
        }

        collapseMergedFloors(pCurrentMap);

        /* Rebuild derived free-space topology in the corrected map frame. */
        pCurrentMap->setSkeletonClusterPoints({});
        pCurrentMap->setSkeletonEdges({});

        for (semantic::Room *p_room : pCurrentMap->getAllRooms())
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            for (geometric::Plane *p_wall : p_room->getWalls())
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    pCurrentMap->addRoomWallPlane(p_wall);
                }
            }
        }

        // Save non corrected poses (already merged maps)
        vector<KeyFrame *> vpKFs = pCurrentMap->getAllKeyFrames();
        for (KeyFrame *pKFi : vpKFs)
        {
            Sophus::SE3d Tiw = (pKFi->getPose()).cast<double>();
            g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            NonCorrectedSim3[pKFi] = g2oSiw;
        }
    }

    if (pMergeMap->getOriginKeyFrame() != nullptr)
    {
        pMergeMap->getOriginKeyFrame()->setFirstConnection(false);
    }
    pNewChild =
        p_mergeMatchedKF
            ->getParent(); // Old parent, it will be the new child of this KF
    pNewParent = p_mergeMatchedKF; // Old child, now it will be the parent of
                                   // its own parent(we need eliminate this KF
                                   // from children list in its old parent)
    p_mergeMatchedKF->changeParent(p_currentKF);
    while (pNewChild)
    {
        pNewChild->eraseChild(
            pNewParent); // We remove the relation between the old parent and
                         // the new for avoid loop
        KeyFrame *pOldParent = pNewChild->getParent();
        pNewChild->changeParent(pNewParent);
        pNewParent = pNewChild;
        pNewChild  = pOldParent;
    }

    vector<MapPoint *>
        vpCheckFuseMapPoint; // MapPoint vector from current map to allow to
                             // fuse duplicated points with the old map (merge)
    vector<KeyFrame *> vpCurrentConnectedKFs;

    mergeConnectedKFs.clear();
    mergeConnectedKFs.push_back(p_mergeMatchedKF);
    vector<KeyFrame *> aux = p_mergeMatchedKF->getVectorCovisibleKeyFrames();
    mergeConnectedKFs.insert(mergeConnectedKFs.end(), aux.begin(), aux.end());
    if (mergeConnectedKFs.size() > 6)
        mergeConnectedKFs.erase(mergeConnectedKFs.begin() + 6,
                                mergeConnectedKFs.end());

    p_currentKF->updateConnections();
    vpCurrentConnectedKFs.push_back(p_currentKF);
    aux = p_currentKF->getVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(),
                                 aux.begin(),
                                 aux.end());
    if (vpCurrentConnectedKFs.size() > 6)
        vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin() + 6,
                                    vpCurrentConnectedKFs.end());

    set<MapPoint *> spMapPointMerge;
    for (KeyFrame *pKFi : mergeConnectedKFs)
    {
        set<MapPoint *> vpMPs = pKFi->getMapPoints();
        spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
        if (spMapPointMerge.size() > 1000)
            break;
    }

    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(),
              spMapPointMerge.end(),
              std::back_inserter(vpCheckFuseMapPoint));
    searchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

    for (KeyFrame *pKFi : vpCurrentConnectedKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->updateConnections();
    }

    const auto finalizeInertialMerge = [this, pCurrentMap, pMergeMap]()
    {
        p_mergeMatchedKF->addMergeEdge(p_currentKF);
        p_currentKF->addMergeEdge(p_mergeMatchedKF);
        pCurrentMap->increaseChangeIndex();

        /*!
         * Inertial welding changes the same derived-map coordinate contract
         * as a visual merge. Increment the externally observed revision after
         * all corrected poses and semantic entities have become authoritative.
         */
        pCurrentMap->informNewBigChange();

        p_atlas->changeMap(pCurrentMap);
        p_atlas->setMapBad(pMergeMap);
        p_atlas->removeBadMaps();
    };
    for (KeyFrame *pKFi : mergeConnectedKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->updateConnections();
    }

    /* A sufficiently established current map can support inertial welding BA.
     */
    bool inertialBundleAdjustmentRan = false;

    if (numKFnew >= 10)
    {
        bool      bStopFlag         = false;
        KeyFrame *p_currentKeyFrame = p_tracker->getLastKeyFrame();

        if (p_currentKeyFrame != nullptr)
        {
            Optimizer::mergeInertialBA(p_currentKeyFrame,
                                       p_mergeMatchedKF,
                                       &bStopFlag,
                                       pCurrentMap,
                                       CorrectedSim3);
            inertialBundleAdjustmentRan = true;
        }
    }

    if (inertialBundleAdjustmentRan)
    {
        /* Complete the post-BA pose map, including fixed deformation nodes. */
        for (const auto &[p_keyFrame, poseBefore_WorldToCamera] :
             NonCorrectedSim3)
        {
            (void)poseBefore_WorldToCamera;

            if (p_keyFrame == nullptr || p_keyFrame->isBad() ||
                p_keyFrame->getMap() != pCurrentMap)
            {
                continue;
            }

            const Sophus::SE3d poseAfter_WorldToCamera =
                p_keyFrame->getPose().cast<double>();

            CorrectedSim3.insert_or_assign(
                p_keyFrame,
                g2o::Sim3(poseAfter_WorldToCamera.unit_quaternion(),
                          poseAfter_WorldToCamera.translation(),
                          1.0));
        }

        const g2o::Sim3 identityTransform_WorldToWorld(
            Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero(),
            1.0);

        utils::utils::Utils::propagateSemanticPoseCorrections(
            pCurrentMap,
            NonCorrectedSim3,
            CorrectedSim3,
            identityTransform_WorldToWorld);
    }

    /* Fuse semantic hypotheses only after the final inertial pose correction.
     */
    if (types::SystemParams::getParams()->semSeg.reassociate.enabled)
    {
        utils::utils::Utils::reAssociateSemanticPlanes(p_atlas);
    }

    /* Matching stable room identities must collapse even when optional
     * geometry reassociation is disabled. */
    utils::utils::Utils::fuseDuplicateRoomsAfterMerge(pCurrentMap,
                                                      importedRooms);

    if (types::SystemParams::getParams()->semSeg.reassociate.enabled)
    {
        utils::utils::Utils::reAssociateRooms(p_atlas);
        utils::utils::Utils::reAssociatePassages(p_atlas);
    }

    finalizeInertialMerge();

    std::cout << "[SemanticMergeGate] surviving_map=" << pCurrentMap->getId()
              << " absorbed_map=" << pMergeMap->getId()
              << " decision=ACCEPT reason=ALIGNED"
              << " floor=" << floorVerificationResult << " committed=1"
              << std::endl;

    /* The semantic graph and Atlas ownership are now stable for other workers.
     */
    semanticUpdateLock.unlock();

    p_localMapper->release();

    if (bRelaunchBA &&
        (!pCurrentMap->isImuInitialized() ||
         (pCurrentMap->getKeyFrameCount() < 200 && p_atlas->countMaps() == 1)))
    {
        relaunchGlobalBundleAdjustment(pCurrentMap);
    }

    return semantic::SemanticMergeDecision::ACCEPT;
}

} // namespace core
} // namespace vs_graphs
