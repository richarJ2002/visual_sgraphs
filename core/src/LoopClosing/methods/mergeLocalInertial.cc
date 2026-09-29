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
#include "LocalMapping.h"
#include "System.h"
#include "Tracking.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"

#include <chrono>
#include <mutex>
#include <rclcpp/logging.hpp>
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

    Map *p_currentMap = p_currentKF->getMap();
    Map *p_mergeMap   = p_mergeMatchedKF->getMap();

    if (p_currentMap == nullptr || p_mergeMap == nullptr ||
        p_currentMap == p_mergeMap || p_currentMap->isBad() ||
        p_mergeMap->isBad() || !p_atlas->isActiveMap(p_currentMap) ||
        !p_atlas->isActiveMap(p_mergeMap))
    {
        return semantic::SemanticMergeDecision::REJECT;
    }

    int temporalKeyFrameCount = 11; // [TODO] Set by parameter

    // Relationship to rebuild the essential graph, it is used two times, first
    // in the local window and later in the rest of the map
    KeyFrame *p_newChild;
    KeyFrame *p_newParent;

    vector<KeyFrame *> localCurrentWindowKeyFrames;
    vector<KeyFrame *> mergeConnectedKeyFrames;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;

    // Flag that is true only when we stopped a running BA, in this case we need
    // relaunch at the end of the merge
    bool shouldRelaunchBa = false;

    /* Stop and reclaim GBA before either map changes frame or ownership. */
    shouldRelaunchBa = stopGlobalBundleAdjustment();

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
        p_currentKF->getMap() != p_currentMap ||
        p_mergeMatchedKF->getMap() != p_mergeMap ||
        !p_atlas->isActiveMap(p_currentMap) ||
        !p_atlas->isActiveMap(p_mergeMap))
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        return semantic::SemanticMergeDecision::REJECT;
    }

    semantic::SemanticVerifyConfig configuration{};
    if (semantic::SemanticVerify::configFromSystemParams(configuration) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: configFromSystemParams returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    semantic::SemanticMergeGateResult semanticMergeGate{};
    if (semantic::SemanticVerify::evaluateMapMergeGate(
            p_currentMap,
            p_mergeMap,
            oldCorrectedPose.inverse(),
            semanticMergeGate,
            configuration) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateMapMergeGate returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const std::string floorVerificationResult = semanticMergeGate.floorDecision;
    const char       *p_name                  = nullptr;
    if (semantic::SemanticVerify::mergeDecisionName(semanticMergeGate.decision,
                                                    p_name) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: mergeDecisionName returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const char *p_name2 = nullptr;
    if (semantic::SemanticVerify::mergeReasonName(semanticMergeGate.reason,
                                                  p_name2) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: mergeReasonName returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "[SemanticMergeGate] surviving_map=" << p_currentMap->getId()
              << " absorbed_map=" << p_mergeMap->getId()
              << " decision=" << p_name << " reason=" << p_name2
              << " shared_rooms=" << semanticMergeGate.sharedRoomCount
              << " aligned_rooms=" << semanticMergeGate.alignedRoomCount
              << " matched_walls=" << semanticMergeGate.matchedWallCount
              << " matched_passages=" << semanticMergeGate.matchedPassageCount
              << " committed=0" << std::endl;
    if (semanticMergeGate.decision != semantic::SemanticMergeDecision::ACCEPT)
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        if (shouldRelaunchBa)
        {
            relaunchGlobalBundleAdjustment(p_currentMap);
        }
        return semanticMergeGate.decision;
    }

    {
        float        s_on = oldCorrectedPose.scale();
        Sophus::SE3f T_on(oldCorrectedPose.rotation().cast<float>(),
                          oldCorrectedPose.translation().cast<float>());

        std::unique_lock<std::mutex> currentMapUpdateLock(
            p_currentMap->mapUpdateMutex);

        p_localMapper->emptyQueue();

        std::chrono::steady_clock::time_point t2 =
            std::chrono::steady_clock::now();
        bool shouldScaleVelocity = false;
        if (s_on != 1)
            shouldScaleVelocity = true;
        p_currentMap->applyScaledRotation(T_on, s_on, shouldScaleVelocity);
        p_tracker->updateFrameIMU(s_on,
                                  p_currentKF->getImuBias(),
                                  p_tracker->getLastKeyFrame());

        std::chrono::steady_clock::time_point t3 =
            std::chrono::steady_clock::now();
    }

    const int keyFrameNewCount = p_currentMap->getKeyFrameCount();

    if ((p_tracker->sensor == System::IMU_MONOCULAR ||
         p_tracker->sensor == System::IMU_STEREO ||
         p_tracker->sensor == System::IMU_RGBD) &&
        !p_currentMap->getInertialBA2())
    {
        /* Map is not completly initialized */
        Eigen::Vector3d bg, ba;
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        Optimizer::inertialOptimization(p_currentMap, bg, ba);
        IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
        std::unique_lock<std::mutex> currentMapUpdateLock(
            p_currentMap->mapUpdateMutex);
        p_tracker->updateFrameIMU(1.0f, b, p_tracker->getLastKeyFrame());

        /* Set map initialized */
        p_currentMap->setInertialBA2();
        p_currentMap->setInertialBA1();
        p_currentMap->setImuInitialized();
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
        std::scoped_lock mapUpdateLocks(p_currentMap->mapUpdateMutex,
                                        p_mergeMap->mapUpdateMutex);

        vector<KeyFrame *> mergeMapKeyFrames = p_mergeMap->getAllKeyFrames();
        vector<MapPoint *> mergeMapMapPoints = p_mergeMap->getAllMapPoints();
        vector<geometric::Plane *> mergeMapPlanes = p_mergeMap->getAllPlanes();
        vector<semantic::Marker *> mergeMapMarkers =
            p_mergeMap->getAllMarkers();
        vector<vs_graphs::core::semantic::Passage *> mergeMapPassages =
            p_mergeMap->getAllPassages();
        vector<semantic::Room *> mergeMapDetectedRooms =
            p_mergeMap->getAllDetectedMapRooms();
        vector<semantic::Room *> mergeMapMarkerRooms =
            p_mergeMap->getAllMarkerBasedMapRooms();
        vector<semantic::Floor *> mergeMapFloors = p_mergeMap->getAllFloors();

        importedRooms = mergeMapDetectedRooms;
        importedRooms.insert(importedRooms.end(),
                             mergeMapMarkerRooms.begin(),
                             mergeMapMarkerRooms.end());

        for (KeyFrame *p_keyFrame : mergeMapKeyFrames)
        {
            if (!p_keyFrame || p_keyFrame->isBad() ||
                p_keyFrame->getMap() != p_mergeMap)
            {
                continue;
            }

            // Make sure connections are updated
            p_keyFrame->updateMap(p_currentMap);
            p_currentMap->addKeyFrame(p_keyFrame);
            p_mergeMap->eraseKeyFrame(p_keyFrame);
        }

        for (MapPoint *p_mapPoint : mergeMapMapPoints)
        {
            if (!p_mapPoint || p_mapPoint->isBad() ||
                p_mapPoint->getMap() != p_mergeMap)
                continue;

            p_mapPoint->updateMap(p_currentMap);
            p_currentMap->addMapPoint(p_mapPoint);
            p_mergeMap->eraseMapPoint(p_mapPoint);
        }

        int nextPlaneId = 0;
        for (geometric::Plane *p_existingPlane : p_currentMap->getAllPlanes())
        {
            if (p_existingPlane != nullptr)
            {
                int existingPlaneGetId{};
                if (p_existingPlane->getId(existingPlaneGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                nextPlaneId = std::max(nextPlaneId, existingPlaneGetId + 1);
            }
        }

        for (geometric::Plane *p_plane : mergeMapPlanes)
        {
            bool planeIsBad{};
            if (!(p_plane == nullptr) &&
                p_plane->isBad(planeIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_plane == nullptr || planeIsBad)
            {
                continue;
            }

            if (p_plane->setMap(p_currentMap) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_plane->setId(nextPlaneId++) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            p_currentMap->addMapPlane(p_plane);
            p_mergeMap->eraseMapPlane(p_plane);
        }

        int nextMarkerId = 0;
        for (semantic::Marker *p_existingMarker : p_currentMap->getAllMarkers())
        {
            if (p_existingMarker != nullptr)
            {
                int existingMarkerId{};
                if (p_existingMarker->getId(existingMarkerId) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                nextMarkerId = std::max(nextMarkerId, existingMarkerId + 1);
            }
        }

        for (semantic::Marker *p_marker : mergeMapMarkers)
        {
            if (p_marker == nullptr)
            {
                continue;
            }

            if (p_marker->setMap(p_currentMap) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_marker->setId(nextMarkerId++) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            p_currentMap->addMapMarker(p_marker);
            p_mergeMap->eraseMapMarker(p_marker);
        }

        for (vs_graphs::core::semantic::Passage *p_passage : mergeMapPassages)
        {
            if (p_passage == nullptr)
            {
                continue;
            }

            semantic::Passage *p_retainedPassage = nullptr;
            for (semantic::Passage *p_existingPassage :
                 p_currentMap->getAllPassages())
            {
                int existingPassageId{};
                if ((p_existingPassage != nullptr) &&
                    p_existingPassage->getId(existingPassageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int passageId{};
                if ((p_existingPassage != nullptr) &&
                    p_passage->getId(passageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_existingPassage != nullptr &&
                    existingPassageId == passageId)
                {
                    p_retainedPassage = p_existingPassage;
                    break;
                }
            }

            p_mergeMap->eraseMapPassage(p_passage);
            if (p_retainedPassage != nullptr)
            {
                bool retainedPassageWasGeometryReplaced{};
                if (p_retainedPassage->mergeFromDuplicate(
                        p_passage,
                        retainedPassageWasGeometryReplaced) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    retainedPassageWasGeometryReplaced = false;
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: mergeFromDuplicate rejected its input; "
                                "continuing as before.",
                                __func__);
                }
                for (semantic::Room *p_room : mergeMapDetectedRooms)
                {
                    if (p_room != nullptr)
                    {
                        bool roomWasAssociationReplaced{};
                        if (p_room->replacePassageAssociation(
                                p_passage,
                                p_retainedPassage,
                                roomWasAssociationReplaced) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            roomWasAssociationReplaced = false;
                            RCLCPP_WARN(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: replacePassageAssociation rejected its "
                                "input; continuing as before.",
                                __func__);
                        }
                    }
                }
                for (semantic::Room *p_room : mergeMapMarkerRooms)
                {
                    if (p_room != nullptr)
                    {
                        bool roomWasAssociationReplaced2{};
                        if (p_room->replacePassageAssociation(
                                p_passage,
                                p_retainedPassage,
                                roomWasAssociationReplaced2) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            roomWasAssociationReplaced2 = false;
                            RCLCPP_WARN(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: replacePassageAssociation rejected its "
                                "input; continuing as before.",
                                __func__);
                        }
                    }
                }
                if (p_passage->setBad() !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setBad returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                continue;
            }

            if (p_passage->setMap(p_currentMap) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            p_currentMap->addMapPassage(p_passage);
        }

        for (semantic::Room *p_room : mergeMapDetectedRooms)
        {
            bool roomIsBad{};
            if (!(p_room == nullptr) &&
                p_room->isBad(roomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room == nullptr || roomIsBad)
            {
                continue;
            }

            if (p_room->setMap(p_currentMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            p_currentMap->addDetectedMapRoom(p_room);
            p_mergeMap->eraseDetectedMapRoom(p_room);
        }

        for (semantic::Room *p_room : mergeMapMarkerRooms)
        {
            bool roomIsBad2{};
            if (!(p_room == nullptr) &&
                p_room->isBad(roomIsBad2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room == nullptr || roomIsBad2)
            {
                continue;
            }

            if (p_room->setMap(p_currentMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            p_currentMap->addCandidateMapRoom(p_room);
            p_mergeMap->eraseMarkerBasedMapRoom(p_room);
        }

        for (semantic::Floor *p_floor : mergeMapFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }

            p_mergeMap->eraseMapFloor(p_floor);
            semantic::Floor *p_retainedFloor = nullptr;
            for (semantic::Floor *p_existingFloor :
                 p_currentMap->getAllFloors())
            {
                int existingFloorId{};
                if ((p_existingFloor != nullptr) &&
                    p_existingFloor->getId(existingFloorId) !=
                        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int floorId{};
                if ((p_existingFloor != nullptr) &&
                    p_floor->getId(floorId) !=
                        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_existingFloor != nullptr && existingFloorId == floorId)
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
            if (p_floor->setMap(p_currentMap) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            p_currentMap->addMapFloor(p_floor);
        }

        collapseMergedFloors(p_currentMap);

        /* Rebuild derived free-space topology in the corrected map frame. */
        p_currentMap->setSkeletonClusterPoints({});
        p_currentMap->setSkeletonEdges({});

        for (semantic::Room *p_room : p_currentMap->getAllRooms())
        {
            bool roomIsBad3{};
            if (!(p_room == nullptr) &&
                p_room->isBad(roomIsBad3) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room == nullptr || roomIsBad3)
            {
                continue;
            }

            std::vector<geometric::Plane *> roomWalls{};
            if (p_room->getWalls(roomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_wall : roomWalls)
            {
                bool wallIsBad{};
                if ((p_wall != nullptr) &&
                    p_wall->isBad(wallIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_wall != nullptr && !wallIsBad)
                {
                    p_currentMap->addRoomWallPlane(p_wall);
                }
            }
        }

        // Save non corrected poses (already merged maps)
        vector<KeyFrame *> keyFrames = p_currentMap->getAllKeyFrames();
        for (KeyFrame *p_keyFrame : keyFrames)
        {
            Sophus::SE3d Tiw = (p_keyFrame->getPose()).cast<double>();
            g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            NonCorrectedSim3[p_keyFrame] = g2oSiw;
        }
    }

    if (p_mergeMap->getOriginKeyFrame() != nullptr)
    {
        p_mergeMap->getOriginKeyFrame()->setFirstConnection(false);
    }
    p_newChild =
        p_mergeMatchedKF
            ->getParent(); // Old parent, it will be the new child of this KF
    p_newParent = p_mergeMatchedKF; // Old child, now it will be the parent of
                                    // its own parent(we need eliminate this KF
                                    // from children list in its old parent)
    p_mergeMatchedKF->changeParent(p_currentKF);
    while (p_newChild)
    {
        p_newChild->eraseChild(
            p_newParent); // We remove the relation between the old parent and
                          // the new for avoid loop
        KeyFrame *p_oldParent = p_newChild->getParent();
        p_newChild->changeParent(p_newParent);
        p_newParent = p_newChild;
        p_newChild  = p_oldParent;
    }

    vector<MapPoint *>
        checkFuseMapPoints; // MapPoint vector from current map to allow to
                            // fuse duplicated points with the old map (merge)
    vector<KeyFrame *> currentConnectedKeyFrames;

    mergeConnectedKFs.clear();
    mergeConnectedKFs.push_back(p_mergeMatchedKF);
    vector<KeyFrame *> aux = p_mergeMatchedKF->getVectorCovisibleKeyFrames();
    mergeConnectedKFs.insert(mergeConnectedKFs.end(), aux.begin(), aux.end());
    if (mergeConnectedKFs.size() > 6)
        mergeConnectedKFs.erase(mergeConnectedKFs.begin() + 6,
                                mergeConnectedKFs.end());

    p_currentKF->updateConnections();
    currentConnectedKeyFrames.push_back(p_currentKF);
    aux = p_currentKF->getVectorCovisibleKeyFrames();
    currentConnectedKeyFrames.insert(currentConnectedKeyFrames.end(),
                                     aux.begin(),
                                     aux.end());
    if (currentConnectedKeyFrames.size() > 6)
        currentConnectedKeyFrames.erase(currentConnectedKeyFrames.begin() + 6,
                                        currentConnectedKeyFrames.end());

    set<MapPoint *> mapPointMerges;
    for (KeyFrame *p_keyFrame : mergeConnectedKFs)
    {
        set<MapPoint *> mapPoints = p_keyFrame->getMapPoints();
        mapPointMerges.insert(mapPoints.begin(), mapPoints.end());
        if (mapPointMerges.size() > 1000)
            break;
    }

    checkFuseMapPoints.reserve(mapPointMerges.size());
    std::copy(mapPointMerges.begin(),
              mapPointMerges.end(),
              std::back_inserter(checkFuseMapPoints));
    searchAndFuse(currentConnectedKeyFrames, checkFuseMapPoints);

    for (KeyFrame *p_keyFrame : currentConnectedKeyFrames)
    {
        if (!p_keyFrame || p_keyFrame->isBad())
            continue;

        p_keyFrame->updateConnections();
    }

    const auto finalizeInertialMerge = [this, p_currentMap, p_mergeMap]()
    {
        p_mergeMatchedKF->addMergeEdge(p_currentKF);
        p_currentKF->addMergeEdge(p_mergeMatchedKF);
        p_currentMap->increaseChangeIndex();

        /*!
         * Inertial welding changes the same derived-map coordinate contract
         * as a visual merge. Increment the externally observed revision after
         * all corrected poses and semantic entities have become authoritative.
         */
        p_currentMap->informNewBigChange();

        p_atlas->changeMap(p_currentMap);
        p_atlas->setMapBad(p_mergeMap);
        p_atlas->removeBadMaps();
    };
    for (KeyFrame *p_keyFrame : mergeConnectedKFs)
    {
        if (!p_keyFrame || p_keyFrame->isBad())
            continue;

        p_keyFrame->updateConnections();
    }

    /* A sufficiently established current map can support inertial welding BA.
     */
    bool inertialBundleAdjustmentRan = false;

    if (keyFrameNewCount >= 10)
    {
        bool      isStopRequested   = false;
        KeyFrame *p_currentKeyFrame = p_tracker->getLastKeyFrame();

        if (p_currentKeyFrame != nullptr)
        {
            Optimizer::mergeInertialBA(p_currentKeyFrame,
                                       p_mergeMatchedKF,
                                       &isStopRequested,
                                       p_currentMap,
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
                p_keyFrame->getMap() != p_currentMap)
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

        if (utils::utils::Utils::propagateSemanticPoseCorrections(
                p_currentMap,
                NonCorrectedSim3,
                CorrectedSim3,
                identityTransform_WorldToWorld) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: propagateSemanticPoseCorrections returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }
    }

    /* Fuse semantic hypotheses only after the final inertial pose correction.
     */
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_params->semSeg.reassociate.enabled)
    {
        if (utils::utils::Utils::reAssociateSemanticPlanes(p_atlas) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: reAssociateSemanticPlanes returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }

    /* Matching stable room identities must collapse even when optional
     * geometry reassociation is disabled. */
    if (utils::utils::Utils::fuseDuplicateRoomsAfterMerge(p_currentMap,
                                                          importedRooms) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: fuseDuplicateRoomsAfterMerge returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    types::SystemParams *p_params2 = nullptr;
    if (types::SystemParams::getParams(p_params2) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_params2->semSeg.reassociate.enabled)
    {
        if (utils::utils::Utils::reAssociateRooms(p_atlas) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reAssociateRooms returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (utils::utils::Utils::reAssociatePassages(p_atlas) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reAssociatePassages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    finalizeInertialMerge();

    std::cout << "[SemanticMergeGate] surviving_map=" << p_currentMap->getId()
              << " absorbed_map=" << p_mergeMap->getId()
              << " decision=ACCEPT reason=ALIGNED"
              << " floor=" << floorVerificationResult << " committed=1"
              << std::endl;

    /* The semantic graph and Atlas ownership are now stable for other workers.
     */
    semanticUpdateLock.unlock();

    p_localMapper->release();

    if (shouldRelaunchBa &&
        (!p_currentMap->isImuInitialized() ||
         (p_currentMap->getKeyFrameCount() < 200 && p_atlas->countMaps() == 1)))
    {
        relaunchGlobalBundleAdjustment(p_currentMap);
    }

    return semantic::SemanticMergeDecision::ACCEPT;
}

} // namespace core
} // namespace vs_graphs
