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

/*!
 * @file            mergeLocalInertial.cc
 *
 * @brief           Implements LoopClosing::mergeLocalInertial(), declared in
 *                  LoopClosing.h.
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

LoopClosingStatus LoopClosing::mergeLocalInertial(
    semantic::SemanticMergeDecision &localInertial_out)
{
    /* Reject stale place-recognition candidates before stopping workers */
    bool currentKFIsBad{};
    if (!(p_currentKF == nullptr || p_mergeMatchedKF == nullptr) &&
        p_currentKF->isBad(currentKFIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool mergeMatchedKFIsBad{};
    if (!(p_currentKF == nullptr || p_mergeMatchedKF == nullptr ||
          currentKFIsBad) &&
        p_mergeMatchedKF->isBad(mergeMatchedKFIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_currentKF == nullptr || p_mergeMatchedKF == nullptr ||
        currentKFIsBad || mergeMatchedKFIsBad)
    {
        localInertial_out = semantic::SemanticMergeDecision::REJECT;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    Map *p_currentMap = nullptr;
    if (p_currentKF->getMap(p_currentMap) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Map *p_mergeMap = nullptr;
    if (p_mergeMatchedKF->getMap(p_mergeMap) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    bool currentMapIsBad{};
    if (!(p_currentMap == nullptr || p_mergeMap == nullptr ||
          p_currentMap == p_mergeMap) &&
        p_currentMap->isBad(currentMapIsBad) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool mergeMapIsBad{};
    if (!(p_currentMap == nullptr || p_mergeMap == nullptr ||
          p_currentMap == p_mergeMap || currentMapIsBad) &&
        p_mergeMap->isBad(mergeMapIsBad) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool atlasIsActiveMap{};
    if (!(p_currentMap == nullptr || p_mergeMap == nullptr ||
          p_currentMap == p_mergeMap || currentMapIsBad || mergeMapIsBad) &&
        p_atlas->isActiveMap(p_currentMap, atlasIsActiveMap) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isActiveMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool atlasIsActiveMap2{};
    if (!(p_currentMap == nullptr || p_mergeMap == nullptr ||
          p_currentMap == p_mergeMap || currentMapIsBad || mergeMapIsBad ||
          !atlasIsActiveMap) &&
        p_atlas->isActiveMap(p_mergeMap, atlasIsActiveMap2) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isActiveMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_currentMap == nullptr || p_mergeMap == nullptr ||
        p_currentMap == p_mergeMap || currentMapIsBad || mergeMapIsBad ||
        !atlasIsActiveMap || !atlasIsActiveMap2)
    {
        localInertial_out = semantic::SemanticMergeDecision::REJECT;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    // Relationship to rebuild the essential graph, it is used two times, first
    // in the local window and later in the rest of the map
    KeyFrame *p_newChild;
    KeyFrame *p_newParent;

    std::vector<KeyFrame *> localCurrentWindowKeyFrames;
    std::vector<KeyFrame *> mergeConnectedKeyFrames;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;

    // Flag that is true only when we stopped a running BA, in this case we need
    // relaunch at the end of the merge
    bool shouldRelaunchBa = false;

    /* Stop and reclaim GBA before either map changes frame or ownership. */
    bool wasRunning{};
    if (stopGlobalBundleAdjustment(wasRunning) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: stopGlobalBundleAdjustment returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    shouldRelaunchBa = wasRunning;

    if (p_localMapper->requestStop() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestStop returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // Wait until Local Mapping has effectively stopped
    for (;;)
    {
        bool localMapperIsStopped{};
        if (p_localMapper->isStopped(localMapperIsStopped) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isStopped returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (localMapperIsStopped)
        {
            break;
        }
        usleep(1000);
    }

    /* Keep the inertial semantic transfer atomic for the complete merge. */
    std::unique_lock<std::mutex> semanticUpdateLock{};
    if (p_atlas->acquireSemanticUpdateLock(semanticUpdateLock) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: acquireSemanticUpdateLock returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    bool currentKFIsBad2{};
    if (p_currentKF->isBad(currentKFIsBad2) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool mergeMatchedKFIsBad2{};
    if (!(currentKFIsBad2) && p_mergeMatchedKF->isBad(mergeMatchedKFIsBad2) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Map *p_currentKFMap = nullptr;
    if (!(currentKFIsBad2 || mergeMatchedKFIsBad2) &&
        p_currentKF->getMap(p_currentKFMap) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Map *p_mergeMatchedKFMap = nullptr;
    if (!(currentKFIsBad2 || mergeMatchedKFIsBad2 ||
          p_currentKFMap != p_currentMap) &&
        p_mergeMatchedKF->getMap(p_mergeMatchedKFMap) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool atlasIsActiveMap3{};
    if (!(currentKFIsBad2 || mergeMatchedKFIsBad2 ||
          p_currentKFMap != p_currentMap ||
          p_mergeMatchedKFMap != p_mergeMap) &&
        p_atlas->isActiveMap(p_currentMap, atlasIsActiveMap3) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isActiveMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool atlasIsActiveMap4{};
    if (!(currentKFIsBad2 || mergeMatchedKFIsBad2 ||
          p_currentKFMap != p_currentMap || p_mergeMatchedKFMap != p_mergeMap ||
          !atlasIsActiveMap3) &&
        p_atlas->isActiveMap(p_mergeMap, atlasIsActiveMap4) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isActiveMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (currentKFIsBad2 || mergeMatchedKFIsBad2 ||
        p_currentKFMap != p_currentMap || p_mergeMatchedKFMap != p_mergeMap ||
        !atlasIsActiveMap3 || !atlasIsActiveMap4)
    {
        semanticUpdateLock.unlock();
        if (p_localMapper->release() !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: release returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        localInertial_out = semantic::SemanticMergeDecision::REJECT;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
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
    unsigned long currentMapId{};
    if (p_currentMap->getId(currentMapId) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    unsigned long mergeMapId{};
    if (p_mergeMap->getId(mergeMapId) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::cout << "[SemanticMergeGate] surviving_map=" << currentMapId
              << " absorbed_map=" << mergeMapId << " decision=" << p_name
              << " reason=" << p_name2
              << " shared_rooms=" << semanticMergeGate.sharedRoomCount
              << " aligned_rooms=" << semanticMergeGate.alignedRoomCount
              << " matched_walls=" << semanticMergeGate.matchedWallCount
              << " matched_passages=" << semanticMergeGate.matchedPassageCount
              << " committed=0" << std::endl;
    if (semanticMergeGate.decision != semantic::SemanticMergeDecision::ACCEPT)
    {
        semanticUpdateLock.unlock();
        if (p_localMapper->release() !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: release returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (shouldRelaunchBa)
        {
            if (relaunchGlobalBundleAdjustment(p_currentMap) !=
                LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: relaunchGlobalBundleAdjustment returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        localInertial_out = semanticMergeGate.decision;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    {
        float        s_on = oldCorrectedPose.scale();
        Sophus::SE3f T_on(oldCorrectedPose.rotation().cast<float>(),
                          oldCorrectedPose.translation().cast<float>());

        std::unique_lock<std::mutex> currentMapUpdateLock(
            p_currentMap->mapUpdateMutex);

        if (p_localMapper->emptyQueue() !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: emptyQueue returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        bool shouldScaleVelocity = false;
        if (s_on != 1)
            shouldScaleVelocity = true;
        if (p_currentMap->applyScaledRotation(T_on,
                                              s_on,
                                              shouldScaleVelocity) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyScaledRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        IMU::Bias currentKFImuBias{};
        if (p_currentKF->getImuBias(currentKFImuBias) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        KeyFrame *p_trackerLastKeyFrame = nullptr;
        if (p_tracker->getLastKeyFrame(p_trackerLastKeyFrame) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLastKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_tracker->updateFrameIMU(s_on,
                                      currentKFImuBias,
                                      p_trackerLastKeyFrame) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateFrameIMU returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    unsigned long keyFrameNewCountValue{};
    if (p_currentMap->getKeyFrameCount(keyFrameNewCountValue) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKeyFrameCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const int keyFrameNewCount = static_cast<int>(keyFrameNewCountValue);

    bool currentMapInertialBA2{};
    if (((p_tracker->sensor == System::IMU_MONOCULAR ||
          p_tracker->sensor == System::IMU_STEREO ||
          p_tracker->sensor == System::IMU_RGBD)) &&
        p_currentMap->getInertialBA2(currentMapInertialBA2) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getInertialBA2 returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if ((p_tracker->sensor == System::IMU_MONOCULAR ||
         p_tracker->sensor == System::IMU_STEREO ||
         p_tracker->sensor == System::IMU_RGBD) &&
        !currentMapInertialBA2)
    {
        /* Map is not completly initialized */
        Eigen::Vector3d bg, ba;
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        if (Optimizer::inertialOptimization(p_currentMap, bg, ba) !=
            OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: inertialOptimization returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
        std::unique_lock<std::mutex> currentMapUpdateLock(
            p_currentMap->mapUpdateMutex);
        KeyFrame *p_trackerLastKeyFrame2 = nullptr;
        if (p_tracker->getLastKeyFrame(p_trackerLastKeyFrame2) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLastKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_tracker->updateFrameIMU(1.0f, b, p_trackerLastKeyFrame2) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateFrameIMU returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Set map initialized */
        if (p_currentMap->setInertialBA2() != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setInertialBA2 returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap->setInertialBA1() != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setInertialBA1 returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap->setImuInitialized() != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setImuInitialized returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Retain imported room identities for post-optimization reconciliation. */
    std::vector<semantic::Room *> importedRooms;

    /* Load KFs and MPs from merge map */
    {
        /*!
         * Acquire both map-update mutexes without imposing an unsafe order.
         *
         * @note            Get Merge Map Mutex and stop tracking.
         */
        std::scoped_lock mapUpdateLocks(p_currentMap->mapUpdateMutex,
                                        p_mergeMap->mapUpdateMutex);

        std::vector<KeyFrame *> mergeMapKeyFrames{};
        if (p_mergeMap->getAllKeyFrames(mergeMapKeyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<MapPoint *> mergeMapMapPoints{};
        if (p_mergeMap->getAllMapPoints(mergeMapMapPoints) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllMapPoints returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<geometric::Plane *> mergeMapPlanes{};
        if (p_mergeMap->getAllPlanes(mergeMapPlanes) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPlanes returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<semantic::Marker *> mergeMapMarkers{};
        if (p_mergeMap->getAllMarkers(mergeMapMarkers) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllMarkers returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<vs_graphs::core::semantic::Passage *> mergeMapPassages{};
        if (p_mergeMap->getAllPassages(mergeMapPassages) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPassages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<semantic::Room *> mergeMapDetectedRooms{};
        if (p_mergeMap->getAllDetectedMapRooms(mergeMapDetectedRooms) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllDetectedMapRooms returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<semantic::Room *> mergeMapMarkerRooms{};
        if (p_mergeMap->getAllMarkerBasedMapRooms(mergeMapMarkerRooms) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getAllMarkerBasedMapRooms returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        std::vector<semantic::Floor *> mergeMapFloors{};
        if (p_mergeMap->getAllFloors(mergeMapFloors) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllFloors returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        importedRooms = mergeMapDetectedRooms;
        importedRooms.insert(importedRooms.end(),
                             mergeMapMarkerRooms.begin(),
                             mergeMapMarkerRooms.end());

        for (KeyFrame *p_keyFrame : mergeMapKeyFrames)
        {
            bool keyFrameIsBad{};
            if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad) !=
                                      KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_keyFrameMap = nullptr;
            if (!(!p_keyFrame || keyFrameIsBad) &&
                p_keyFrame->getMap(p_keyFrameMap) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!p_keyFrame || keyFrameIsBad || p_keyFrameMap != p_mergeMap)
            {
                continue;
            }

            // Make sure connections are updated
            if (p_keyFrame->updateMap(p_currentMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updateMap returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap->addKeyFrame(p_keyFrame) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mergeMap->eraseKeyFrame(p_keyFrame) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        for (MapPoint *p_mapPoint : mergeMapMapPoints)
        {
            bool mapPointIsBad{};
            if (!(!p_mapPoint) && p_mapPoint->isBad(mapPointIsBad) !=
                                      MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_mapPointMap = nullptr;
            if (!(!p_mapPoint || mapPointIsBad) &&
                p_mapPoint->getMap(p_mapPointMap) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!p_mapPoint || mapPointIsBad || p_mapPointMap != p_mergeMap)
                continue;

            if (p_mapPoint->updateMap(p_currentMap) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updateMap returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap->addMapPoint(p_mapPoint) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mergeMap->eraseMapPoint(p_mapPoint) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        int                             nextPlaneId = 0;
        std::vector<geometric::Plane *> currentMapAllPlanes{};
        if (p_currentMap->getAllPlanes(currentMapAllPlanes) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllPlanes returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_existingPlane : currentMapAllPlanes)
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
            if (p_currentMap->addMapPlane(p_plane) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mergeMap->eraseMapPlane(p_plane) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        int                             nextMarkerId = 0;
        std::vector<semantic::Marker *> currentMapAllMarkers{};
        if (p_currentMap->getAllMarkers(currentMapAllMarkers) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllMarkers returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Marker *p_existingMarker : currentMapAllMarkers)
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
            if (p_currentMap->addMapMarker(p_marker) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapMarker returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mergeMap->eraseMapMarker(p_marker) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapMarker returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        for (vs_graphs::core::semantic::Passage *p_passage : mergeMapPassages)
        {
            if (p_passage == nullptr)
            {
                continue;
            }

            semantic::Passage *p_retainedPassage = nullptr;
            std::vector<vs_graphs::core::semantic::Passage *>
                currentMapAllPassages{};
            if (p_currentMap->getAllPassages(currentMapAllPassages) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (semantic::Passage *p_existingPassage : currentMapAllPassages)
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

            if (p_mergeMap->eraseMapPassage(p_passage) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPassage returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
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
            if (p_currentMap->addMapPassage(p_passage) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPassage returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
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
            if (p_currentMap->addDetectedMapRoom(p_room) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addDetectedMapRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mergeMap->eraseDetectedMapRoom(p_room) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: eraseDetectedMapRoom returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
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
            if (p_currentMap->addCandidateMapRoom(p_room) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: addCandidateMapRoom returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_mergeMap->eraseMarkerBasedMapRoom(p_room) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: eraseMarkerBasedMapRoom returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        for (semantic::Floor *p_floor : mergeMapFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }

            if (p_mergeMap->eraseMapFloor(p_floor) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapFloor returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Floor               *p_retainedFloor = nullptr;
            std::vector<semantic::Floor *> currentMapAllFloors{};
            if (p_currentMap->getAllFloors(currentMapAllFloors) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllFloors returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (semantic::Floor *p_existingFloor : currentMapAllFloors)
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
                if (mergeFloorEvidenceAndRooms(p_retainedFloor, p_floor) !=
                    LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: mergeFloorEvidenceAndRooms returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
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
            if (p_currentMap->addMapFloor(p_floor) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapFloor returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        if (collapseMergedFloors(p_currentMap) !=
            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: collapseMergedFloors returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Rebuild derived free-space topology in the corrected map frame. */
        if (p_currentMap->setSkeletonClusterPoints({}) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: setSkeletonClusterPoints returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_currentMap->setSkeletonEdges({}) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setSkeletonEdges returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        std::vector<semantic::Room *> currentMapAllRooms{};
        if (p_currentMap->getAllRooms(currentMapAllRooms) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllRooms returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Room *p_room : currentMapAllRooms)
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
                    if (p_currentMap->addRoomWallPlane(p_wall) !=
                        MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addRoomWallPlane returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }
        }

        // Save non corrected poses (already merged maps)
        std::vector<KeyFrame *> keyFrames{};
        if (p_currentMap->getAllKeyFrames(keyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (KeyFrame *p_keyFrame : keyFrames)
        {
            Sophus::SE3f keyFramePose{};
            if (p_keyFrame->getPose(keyFramePose) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            Sophus::SE3d keyFramePose_worldToCamera =
                (keyFramePose).cast<double>();
            g2o::Sim3 g2oSiw(keyFramePose_worldToCamera.unit_quaternion(),
                             keyFramePose_worldToCamera.translation(),
                             1.0);
            NonCorrectedSim3[p_keyFrame] = g2oSiw;
        }
    }

    KeyFrame *p_mergeMapOriginKeyFrame = nullptr;
    if (p_mergeMap->getOriginKeyFrame(p_mergeMapOriginKeyFrame) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getOriginKeyFrame returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_mergeMapOriginKeyFrame != nullptr)
    {
        KeyFrame *p_mergeMapOriginKeyFrame2 = nullptr;
        if (p_mergeMap->getOriginKeyFrame(p_mergeMapOriginKeyFrame2) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getOriginKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mergeMapOriginKeyFrame2->setFirstConnection(false) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setFirstConnection returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    KeyFrame *p_mergeMatchedKFParent = nullptr;
    if (p_mergeMatchedKF->getParent(p_mergeMatchedKFParent) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParent returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_newChild = p_mergeMatchedKFParent; // Old parent, it will be the new child
                                         // of this KF
    p_newParent = p_mergeMatchedKF; // Old child, now it will be the parent of
                                    // its own parent(we need eliminate this KF
                                    // from children list in its old parent)
    if (p_mergeMatchedKF->changeParent(p_currentKF) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: changeParent returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    while (p_newChild)
    {
        if (p_newChild->eraseChild(p_newParent) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseChild returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        } // We remove the relation between the old parent and
          // the new for avoid loop
        KeyFrame *p_oldParent = nullptr;
        if (p_newChild->getParent(p_oldParent) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParent returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_newChild->changeParent(p_newParent) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: changeParent returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        p_newParent = p_newChild;
        p_newChild  = p_oldParent;
    }

    std::vector<MapPoint *>
        checkFuseMapPoints; // MapPoint vector from current map to allow to
                            // fuse duplicated points with the old map (merge)
    std::vector<KeyFrame *> currentConnectedKeyFrames;

    mergeConnectedKFs.clear();
    mergeConnectedKFs.push_back(p_mergeMatchedKF);
    std::vector<KeyFrame *> aux{};
    if (p_mergeMatchedKF->getVectorCovisibleKeyFrames(aux) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getVectorCovisibleKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    mergeConnectedKFs.insert(mergeConnectedKFs.end(), aux.begin(), aux.end());
    if (mergeConnectedKFs.size() > 6)
        mergeConnectedKFs.erase(mergeConnectedKFs.begin() + 6,
                                mergeConnectedKFs.end());

    if (p_currentKF->updateConnections() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    currentConnectedKeyFrames.push_back(p_currentKF);
    std::vector<KeyFrame *> currentKFVectorCovisibleKeyFrames{};
    if (p_currentKF->getVectorCovisibleKeyFrames(
            currentKFVectorCovisibleKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getVectorCovisibleKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    aux = currentKFVectorCovisibleKeyFrames;
    currentConnectedKeyFrames.insert(currentConnectedKeyFrames.end(),
                                     aux.begin(),
                                     aux.end());
    if (currentConnectedKeyFrames.size() > 6)
        currentConnectedKeyFrames.erase(currentConnectedKeyFrames.begin() + 6,
                                        currentConnectedKeyFrames.end());

    std::set<MapPoint *> mapPointMerges;
    for (KeyFrame *p_keyFrame : mergeConnectedKFs)
    {
        std::set<MapPoint *> mapPoints{};
        if (p_keyFrame->getMapPoints(mapPoints) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPoints returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        mapPointMerges.insert(mapPoints.begin(), mapPoints.end());
        if (mapPointMerges.size() > 1000)
            break;
    }

    checkFuseMapPoints.reserve(mapPointMerges.size());
    std::copy(mapPointMerges.begin(),
              mapPointMerges.end(),
              std::back_inserter(checkFuseMapPoints));
    if (searchAndFuse(currentConnectedKeyFrames, checkFuseMapPoints) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: searchAndFuse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (KeyFrame *p_keyFrame : currentConnectedKeyFrames)
    {
        bool keyFrameIsBad2{};
        if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad2) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad2)
            continue;

        if (p_keyFrame->updateConnections() !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateConnections returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    const auto finalizeInertialMerge = [this, p_currentMap, p_mergeMap]()
    {
        if (p_mergeMatchedKF->addMergeEdge(p_currentKF) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMergeEdge returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentKF->addMergeEdge(p_mergeMatchedKF) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMergeEdge returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap->increaseChangeIndex() !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: increaseChangeIndex returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /*!
         * Inertial welding changes the same derived-map coordinate contract
         * as a visual merge. Increment the externally observed revision after
         * all corrected poses and semantic entities have become authoritative.
         */
        if (p_currentMap->informNewBigChange() != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: informNewBigChange returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_atlas->changeMap(p_currentMap) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: changeMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_atlas->setMapBad(p_mergeMap) != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMapBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_atlas->removeBadMaps() != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: removeBadMaps returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    };
    for (KeyFrame *p_keyFrame : mergeConnectedKFs)
    {
        bool keyFrameIsBad3{};
        if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad3) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad3)
            continue;

        if (p_keyFrame->updateConnections() !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateConnections returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* A sufficiently established current map can support inertial welding BA.
     */
    bool inertialBundleAdjustmentRan = false;

    if (keyFrameNewCount >= 10)
    {
        bool      isStopRequested   = false;
        KeyFrame *p_currentKeyFrame = nullptr;
        if (p_tracker->getLastKeyFrame(p_currentKeyFrame) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLastKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_currentKeyFrame != nullptr)
        {
            if (Optimizer::mergeInertialBA(p_currentKeyFrame,
                                           p_mergeMatchedKF,
                                           &isStopRequested,
                                           p_currentMap,
                                           CorrectedSim3) !=
                OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: mergeInertialBA returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            inertialBundleAdjustmentRan = true;
        }
    }

    if (inertialBundleAdjustmentRan)
    {
        /* Complete the post-BA pose map, including fixed deformation nodes. */
        for (const auto &[p_keyFrame, poseBefore_worldToCamera] :
             NonCorrectedSim3)
        {
            (void)poseBefore_worldToCamera;

            bool keyFrameIsBad4{};
            if (!(p_keyFrame == nullptr) &&
                p_keyFrame->isBad(keyFrameIsBad4) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_keyFrameMap2 = nullptr;
            if (!(p_keyFrame == nullptr || keyFrameIsBad4) &&
                p_keyFrame->getMap(p_keyFrameMap2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame == nullptr || keyFrameIsBad4 ||
                p_keyFrameMap2 != p_currentMap)
            {
                continue;
            }

            Sophus::SE3f keyFramePose2{};
            if (p_keyFrame->getPose(keyFramePose2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            const Sophus::SE3d poseAfter_worldToCamera =
                keyFramePose2.cast<double>();

            CorrectedSim3.insert_or_assign(
                p_keyFrame,
                g2o::Sim3(poseAfter_worldToCamera.unit_quaternion(),
                          poseAfter_worldToCamera.translation(),
                          1.0));
        }

        const g2o::Sim3 identityTransform_worldToWorld(
            Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero(),
            1.0);

        if (utils::utils::Utils::propagateSemanticPoseCorrections(
                p_currentMap,
                NonCorrectedSim3,
                CorrectedSim3,
                identityTransform_worldToWorld) !=
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

    unsigned long currentMapId2{};
    if (p_currentMap->getId(currentMapId2) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    unsigned long mergeMapId2{};
    if (p_mergeMap->getId(mergeMapId2) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::cout << "[SemanticMergeGate] surviving_map=" << currentMapId2
              << " absorbed_map=" << mergeMapId2
              << " decision=ACCEPT reason=ALIGNED"
              << " floor=" << floorVerificationResult << " committed=1"
              << std::endl;

    /* The semantic graph and Atlas ownership are now stable for other workers.
     */
    semanticUpdateLock.unlock();

    if (p_localMapper->release() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: release returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    bool currentMapIsImuInitialized{};
    if ((shouldRelaunchBa) &&
        p_currentMap->isImuInitialized(currentMapIsImuInitialized) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    unsigned long currentMapKeyFrameCount{};
    if ((shouldRelaunchBa) && !(!currentMapIsImuInitialized) &&
        p_currentMap->getKeyFrameCount(currentMapKeyFrameCount) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKeyFrameCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    int atlasMaps{};
    if ((shouldRelaunchBa) && !(!currentMapIsImuInitialized) &&
        (currentMapKeyFrameCount < 200) &&
        p_atlas->countMaps(atlasMaps) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: countMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (shouldRelaunchBa && (!currentMapIsImuInitialized ||
                             (currentMapKeyFrameCount < 200 && atlasMaps == 1)))
    {
        if (relaunchGlobalBundleAdjustment(p_currentMap) !=
            LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: relaunchGlobalBundleAdjustment returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }

    localInertial_out = semantic::SemanticMergeDecision::ACCEPT;
    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
