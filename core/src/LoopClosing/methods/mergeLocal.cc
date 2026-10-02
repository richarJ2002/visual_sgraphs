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
 * @file            mergeLocal.cc
 *
 * @brief           Implements LoopClosing::mergeLocal(), declared in
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

#include <mutex>
#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

LoopClosingStatus
    LoopClosing::mergeLocal(semantic::SemanticMergeDecision &local_out)
{
    /* ---------------------------------------------------------------------- *
     * SECTION 1 - INITIALISATION
     *
     * Merge Policy
     * ------------------------------------------------------------------------
     * The current map is treated as the authoritative map and therefore
     * survives the merge. The matched map is transformed into the coordinate
     * frame of the current map before all of its contents are appended into
     * the current map. Once all objects have been transferred, the merge map
     * is marked as bad and removed from the atlas.
     * ---------------------------------------------------------------------- */

    /* Constant used to determine the number of temporal keyframes */
    constexpr int COUNT_TEMPORAL_KEY_FRAMES = 25;

    /* Extract the system parameters */
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_sysParams = p_params;

    /* Reject stale place-recognition candidates before stopping other workers.
     */
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
        local_out = semantic::SemanticMergeDecision::REJECT;
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
        local_out = semantic::SemanticMergeDecision::REJECT;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 2 - STOP GLOBAL BUNDLE ADJUSTMENT
     *
     * The merge modifies map ownership, poses and graph connectivity.
     * Therefore no optimisation thread is allowed to access these objects
     * while the merge is taking place.
     * ---------------------------------------------------------------------- */

    /* Flag to indicate if bundle adjustment should be relaunched */
    bool shouldRelaunchBa = false;

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

    /* ---------------------------------------------------------------------- *
     * SECTION 3 - STOP LOCAL MAPPING
     * ---------------------------------------------------------------------- */

    /* Request stop */
    if (p_localMapper->requestStop() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestStop returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Wait until local mapper stops */
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

    /* ---------------------------------------------------------------------- *
     * SECTION 4 - IDENTIFY SOURCE AND DESTINATION MAPS
     *
     * pCurrentMap
     *      Survives the merge.
     *
     * pMergeMap
     *      Is transformed into the current-map frame before being appended
     *      into pCurrentMap.
     * ---------------------------------------------------------------------- */

    /*
     * Prevent semantic worker threads from modifying either graph while map
     * frames, ownership, and cross-entity references are being changed.
     */
    std::unique_lock<std::mutex> semanticUpdateLock{};
    if (p_atlas->acquireSemanticUpdateLock(semanticUpdateLock) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: acquireSemanticUpdateLock returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Revalidate after quiescing workers; retained retired maps keep stale
     * raw pointers alive, so pointer non-nullness alone is insufficient. */
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
        local_out = semantic::SemanticMergeDecision::REJECT;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    Sophus::SE3f currentKFPoseInverse{};
    if (p_currentKF->getPoseInverse(currentKFPoseInverse) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPoseInverse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const Sophus::SE3d cameraPose_cameraToWorld =
        currentKFPoseInverse.cast<double>();
    const g2o::Sim3 g2oNonCorrectedSwc(
        cameraPose_cameraToWorld.unit_quaternion(),
        cameraPose_cameraToWorld.translation(),
        1.0);
    const g2o::Sim3 g2oSwCurrentWMerge = g2oNonCorrectedSwc * mg2oMergeScw;
    const g2o::Sim3 g2oSwMergeWCurrent = g2oSwCurrentWMerge.inverse();

    semantic::SemanticVerifyConfig configuration2{};
    if (semantic::SemanticVerify::configFromSystemParams(configuration2) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: configFromSystemParams returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    semantic::SemanticMergeGateResult semanticMergeGate{};
    if (semantic::SemanticVerify::evaluateMapMergeGate(p_currentMap,
                                                       p_mergeMap,
                                                       g2oSwCurrentWMerge,
                                                       semanticMergeGate,
                                                       configuration2) !=
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
        local_out = semanticMergeGate.decision;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    /* Discard queued keyframes only after every merge rejection gate passed. */
    if (p_localMapper->emptyQueue() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: emptyQueue returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Update the connections of the current keyframe */
    if (p_currentKF->updateConnections() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 5 - BUILD THE CURRENT-MAP LOCAL WINDOW
     *
     * This window forms the fixed reference side of the weld.
     * ---------------------------------------------------------------------- */

    std::set<KeyFrame *> localWindowKeyFrames;
    std::set<MapPoint *> localWindowMapPoints;

    /*!
     * If using IMU, construct temporal inertial chain. Otherwise, start with
     * current keyframe for local window.
     */
    bool currentMapIsInertial{};
    if (p_currentMap->isInertial(currentMapIsInertial) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isInertial returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool mergeMapIsInertial{};
    if ((currentMapIsInertial) && p_mergeMap->isInertial(mergeMapIsInertial) !=
                                      MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isInertial returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (currentMapIsInertial && mergeMapIsInertial)
    {
        /* ------------------------------------------------------------------ *
         * Walk backwards through the temporal chain
         * ------------------------------------------------------------------ */

        KeyFrame *p_keyFrame    = p_currentKF;
        int       insertedCount = 0;

        while (p_keyFrame && insertedCount < COUNT_TEMPORAL_KEY_FRAMES)
        {
            localWindowKeyFrames.insert(p_keyFrame);

            std::set<MapPoint *> mapPoints{};
            if (p_keyFrame->getMapPoints(mapPoints) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPoints returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            localWindowMapPoints.insert(mapPoints.begin(), mapPoints.end());

            p_keyFrame = p_keyFrame->p_prevKF;
            insertedCount++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards through the temporal chain
         * ------------------------------------------------------------------ */

        p_keyFrame    = p_currentKF->p_nextKF;
        insertedCount = 0;

        while (p_keyFrame && insertedCount < COUNT_TEMPORAL_KEY_FRAMES)
        {
            localWindowKeyFrames.insert(p_keyFrame);

            std::set<MapPoint *> mapPoints{};
            if (p_keyFrame->getMapPoints(mapPoints) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPoints returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            localWindowMapPoints.insert(mapPoints.begin(), mapPoints.end());

            p_keyFrame = p_keyFrame->p_nextKF;
            insertedCount++;
        }
    }
    else
    {
        localWindowKeyFrames.insert(p_currentKF);
    }

    /* ---------------------------------------------------------------------- *
     * Expand the local window using covisibility.
     * ---------------------------------------------------------------------- */

    /* Create list of strongest covisibility connectsion to current keyframe */
    std::vector<KeyFrame *> covisibleKeyFrames{};
    if (p_currentKF->getBestCovisibilityKeyFrames(COUNT_TEMPORAL_KEY_FRAMES,
                                                  covisibleKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBestCovisibilityKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Insert keyframes with best connections into local window */
    localWindowKeyFrames.insert(covisibleKeyFrames.begin(),
                                covisibleKeyFrames.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    localWindowKeyFrames.insert(p_currentKF);

    constexpr int MAXIMUM_EXPANSION_ITERATIONS = 5;

    int expansionCount = 0;

    /*!
     * If there is not enough keyframes in the local window, then we look at the
     * keyframes currently in spLocalWindowKF and find there best covisibility
     * keyframes. We do this to gaurantee there is enough keyframes in the
     * `spLocalWindowKFs`. We attmemp to reach that amount
     * `kMaxExpansionIterations`. In other words, we are willing to expand the
     * window with `kMaxExpansionIterations` iterations to reach
     * `kNumTemporalKFs` in `spLocalWindowKFs`, if the current keyframe doesn't
     * have enough covisible keyframes attached to it.
     */
    while (localWindowKeyFrames.size() < COUNT_TEMPORAL_KEY_FRAMES &&
           expansionCount < MAXIMUM_EXPANSION_ITERATIONS)
    {
        std::vector<KeyFrame *> newCovisibles;

        for (KeyFrame *p_keyFrame : localWindowKeyFrames)
        {
            std::vector<KeyFrame *> covisibles{};
            if (p_keyFrame->getBestCovisibilityKeyFrames(
                    COUNT_TEMPORAL_KEY_FRAMES / 2,
                    covisibles) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getBestCovisibilityKeyFrames returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }

            for (KeyFrame *p_covisibleKeyFrame : covisibles)
            {
                if (!p_covisibleKeyFrame)
                    continue;

                bool covisibleKeyFrameIsBad{};
                if (p_covisibleKeyFrame->isBad(covisibleKeyFrameIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (covisibleKeyFrameIsBad)
                    continue;

                if (localWindowKeyFrames.count(p_covisibleKeyFrame))
                    continue;

                newCovisibles.push_back(p_covisibleKeyFrame);
            }
        }

        localWindowKeyFrames.insert(newCovisibles.begin(), newCovisibles.end());

        ++expansionCount;
    }

    /* ---------------------------------------------------------------------- *
     * Collect all landmarks observed by the current-map welding window.
     * ---------------------------------------------------------------------- */

    for (KeyFrame *p_keyFrame : localWindowKeyFrames)
    {
        /* Skip invalid keyframes. (Shouldn't need this but good for safety) */
        bool keyFrameIsBad{};
        if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad)
        {
            continue;
        }

        /* Extract the map points from the keyframe */
        std::set<MapPoint *> mapPoints{};
        if (p_keyFrame->getMapPoints(mapPoints) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPoints returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Insert all the map points into the map point local window */
        localWindowMapPoints.insert(mapPoints.begin(), mapPoints.end());
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 6 - BUILD THE MERGE-MAP LOCAL WINDOW
     *
     * These keyframes will be transformed into the current-map frame before
     * being transferred into the surviving map. Essentially, repeat above
     * steps but with the merge map.
     * ---------------------------------------------------------------------- */

    std::set<KeyFrame *> mergeConnectedKeyFrames;
    std::set<MapPoint *> mapPointMerges;

    /*!
     * If using IMU, construct temporal inertial chain. Otherwise, start with
     * current keyframe for local window.
     */
    bool currentMapIsInertial2{};
    if (p_currentMap->isInertial(currentMapIsInertial2) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isInertial returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool mergeMapIsInertial2{};
    if ((currentMapIsInertial2) &&
        p_mergeMap->isInertial(mergeMapIsInertial2) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isInertial returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (currentMapIsInertial2 && mergeMapIsInertial2)
    {
        KeyFrame *p_keyFrame    = p_mergeMatchedKF;
        int       insertedCount = 0;

        /* ------------------------------------------------------------------ *
         * Walk backwards
         * ------------------------------------------------------------------ */

        while (p_keyFrame && insertedCount < (COUNT_TEMPORAL_KEY_FRAMES / 2))
        {
            mergeConnectedKeyFrames.insert(p_keyFrame);

            p_keyFrame = p_keyFrame->p_prevKF;

            insertedCount++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards
         * ------------------------------------------------------------------ */

        p_keyFrame = p_mergeMatchedKF->p_nextKF;

        while (p_keyFrame && insertedCount < COUNT_TEMPORAL_KEY_FRAMES)
        {
            mergeConnectedKeyFrames.insert(p_keyFrame);

            p_keyFrame = p_keyFrame->p_nextKF;

            insertedCount++;
        }
    }
    else
    {
        mergeConnectedKeyFrames.insert(p_mergeMatchedKF);
    }

    /* ---------------------------------------------------------------------- *
     * Expand the local window using covisibility.
     * ---------------------------------------------------------------------- */

    /* Create list of strongest covisibility connectsion to current keyframe */
    std::vector<KeyFrame *> mergeMatchedKFBestCovisibilityKeyFrames{};
    if (p_mergeMatchedKF->getBestCovisibilityKeyFrames(
            COUNT_TEMPORAL_KEY_FRAMES,
            mergeMatchedKFBestCovisibilityKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBestCovisibilityKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    covisibleKeyFrames = mergeMatchedKFBestCovisibilityKeyFrames;

    /* Insert keyframes with best connections into local window */
    mergeConnectedKeyFrames.insert(covisibleKeyFrames.begin(),
                                   covisibleKeyFrames.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    mergeConnectedKeyFrames.insert(p_mergeMatchedKF);

    /* Reset counter */
    expansionCount = 0;

    /*!
     * If there is not enough keyframes in the merge connected window, then we
     * look at the keyframes currently in spMergeConnectedKFs and find there
     * best covisibility keyframes. We do this to gaurantee there is enough
     * keyframes in the `spMergeConnectedKFs`. We attmemp to reach that amount
     * `kMaxExpansionIterations`. In other words, we are willing to expand the
     * window with `kMaxExpansionIterations` iterations to reach
     * `kNumTemporalKFs` in `spMergeConnectedKFs`, if the current keyframe
     * doesn't have enough covisible keyframes attached to it.
     */
    while (mergeConnectedKeyFrames.size() < COUNT_TEMPORAL_KEY_FRAMES &&
           expansionCount < MAXIMUM_EXPANSION_ITERATIONS)
    {
        std::vector<KeyFrame *> newCovisibles;

        for (KeyFrame *p_keyFrame : mergeConnectedKeyFrames)
        {
            std::vector<KeyFrame *> covisibles{};
            if (p_keyFrame->getBestCovisibilityKeyFrames(
                    COUNT_TEMPORAL_KEY_FRAMES / 2,
                    covisibles) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getBestCovisibilityKeyFrames returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }

            for (KeyFrame *p_covisibleKeyFrame : covisibles)
            {
                if (!p_covisibleKeyFrame)
                    continue;

                bool covisibleKeyFrameIsBad2{};
                if (p_covisibleKeyFrame->isBad(covisibleKeyFrameIsBad2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (covisibleKeyFrameIsBad2)
                    continue;

                if (mergeConnectedKeyFrames.count(p_covisibleKeyFrame))
                    continue;

                newCovisibles.push_back(p_covisibleKeyFrame);
            }
        }

        mergeConnectedKeyFrames.insert(newCovisibles.begin(),
                                       newCovisibles.end());

        ++expansionCount;
    }

    /* ---------------------------------------------------------------------- *
     * Collect all landmarks observed by the imported welding window.
     * ---------------------------------------------------------------------- */

    for (KeyFrame *p_keyFrame : mergeConnectedKeyFrames)
    {
        /* Skip invalid keyframes. (Shouldn't need this but good for safety) */
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
        {
            continue;
        }

        /* Extract the map points from the keyframe */
        std::set<MapPoint *> mapPoints{};
        if (p_keyFrame->getMapPoints(mapPoints) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPoints returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Insert all the map points into the map point local window */
        mapPointMerges.insert(mapPoints.begin(), mapPoints.end());
    }

    /*!
     * Search and fuse only accepts std::vector<MapPoint *>, not
     * std::set<MapPoint *>. Hence, need to move the `spLocalWindowMPs` to a
     * new object to enable search and fuse. With `spLocalWindowMPs` being a
     * set this does mean that `vpCheckFuseMapPoint` will have no duplicates.
     */

    /* Init list of fused map points */
    std::vector<MapPoint *> checkFuseMapPoints;

    /* Reserve the memory for the fused map points */
    checkFuseMapPoints.reserve(localWindowMapPoints.size());

    /* Copy the map points from the local window */
    checkFuseMapPoints.assign(localWindowMapPoints.begin(),
                              localWindowMapPoints.end());

    /* ---------------------------------------------------------------------- *
     * SECTION 7 - COMPUTE THE MAP-TO-MAP SIMILARITY TRANSFORM
     *
     *
     * Purpose
     * ----------------------------------------------------------------------
     *
     * Compute the similarity transform required to express all geometry from
     * the merge-map world frame inside the surviving current-map world frame.
     *
     *
     * Coordinate Frames
     * ----------------------------------------------------------------------
     *
     *      Merge World -----> Current Camera -----> Current World
     *          |                 mg2oMergeScw            Twc
     *
     *
     * Result
     * ----------------------------------------------------------------------
     *
     * g2oSwCurrentWMerge :
     *      Maps merge-world coordinates into current-world coordinates.
     *
     * g2oSwMergeWCurrent :
     *      Inverse transform used when correcting imported keyframe poses.
     *
     *
     * ---------------------------------------------------------------------- */

    /* The map-to-map transforms were computed before the floor preflight so a
     * rejected merge could leave poses and ownership untouched. */

    /* ---------------------------------------------------------------------- *
     * SECTION 8 - CORRECT IMPORTED KEYFRAME POSES
     *
     * Every keyframe belonging to the merge-map welding window is transformed
     * into the current-map reference frame.
     *
     * During this stage no ownership changes occur. The corrected poses are
     * simply cached for later insertion into the surviving map. These
     * transforms are stored in: `vNonCorrectedSim3` and `vCorrectedSim3`.
     * ---------------------------------------------------------------------- */

    /* Stores merge side keyframe's original pose before applying correction */
    KeyFrameAndPose vNonCorrectedSim3;

    /* Stores the corrected merge keyframe pose */
    KeyFrameAndPose vCorrectedSim3;

    /* Iterate through every merge keyframe in the merge connected KF list */
    for (KeyFrame *p_keyFrame : mergeConnectedKeyFrames)
    {
        /* Skip invalid keyframes */
        bool keyFrameIsBad3{};
        if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad3) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Map *p_keyFrameMap = nullptr;
        if (!(!p_keyFrame || keyFrameIsBad3) &&
            p_keyFrame->getMap(p_keyFrameMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad3 || p_keyFrameMap != p_mergeMap)
        {
            continue;
        }

        /* Extract the current pose of the merge keyframe iteration */
        Sophus::SE3f keyFramePose{};
        if (p_keyFrame->getPose(keyFramePose) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const Sophus::SE3d TiwMerge = keyFramePose.cast<double>();

        /* Convert to a g2o::Sim3 object type */
        const g2o::Sim3 g2oSiwMerge(TiwMerge.unit_quaternion(),
                                    TiwMerge.translation(),
                                    1.0);

        /* Find the transform from current world to keyframe iteration (i) */
        const g2o::Sim3 g2oSiwCurrent = g2oSiwMerge * g2oSwMergeWCurrent;

        /* Store transforms */
        vNonCorrectedSim3[p_keyFrame] = g2oSiwMerge;
        vCorrectedSim3[p_keyFrame]    = g2oSiwCurrent;

        /* Find the scale of transform */
        const double s = g2oSiwCurrent.scale();

        /* Find the transform from merge to current map */
        p_keyFrame->correctedScale = s;
        p_keyFrame->tcwMerge       = Sophus::SE3d(g2oSiwCurrent.rotation(),
                                            g2oSiwCurrent.translation() / s)
                                   .cast<float>();

        /* If there is IMU, extract velocity */
        bool currentMapIsImuInitialized{};
        if (p_currentMap->isImuInitialized(currentMapIsImuInitialized) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isImuInitialized returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (currentMapIsImuInitialized)
        {
            const Eigen::Quaternionf Rcor =
                (g2oSiwCurrent.rotation().inverse() * g2oSiwMerge.rotation())
                    .cast<float>();
            Eigen::Vector3f keyFrameVelocity{};
            if (p_keyFrame->getVelocity(keyFrameVelocity) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            p_keyFrame->vwbMerge = Rcor * keyFrameVelocity;
        }
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 9 - CORRECT IMPORTED MAP POINTS
     *
     * Transform every imported landmark into the current-map coordinate frame.
     *
     * Position:
     *      Full Sim3 transformation.
     *
     * Normal:
     *      Rotation only.
     *
     * Invalid landmarks are removed from the temporary welding set.
     * ---------------------------------------------------------------------- */

    /* Iterate through all the mapped points in the merge map */
    for (std::set<MapPoint *>::const_iterator itMapPoint =
             mapPointMerges.begin();
         itMapPoint != mapPointMerges.end();)
    {
        /* Copy the map points */
        MapPoint *p_currentMapPoint = *itMapPoint;

        /* If the mapped points are invalud, erase and skip */
        bool currentMapPointIsBad{};
        if (!(!p_currentMapPoint) &&
            p_currentMapPoint->isBad(currentMapPointIsBad) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Map *p_currentMapPointMap = nullptr;
        if (!(!p_currentMapPoint || currentMapPointIsBad) &&
            p_currentMapPoint->getMap(p_currentMapPointMap) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_currentMapPoint || currentMapPointIsBad ||
            p_currentMapPointMap != p_mergeMap)
        {
            itMapPoint = mapPointMerges.erase(itMapPoint);
            continue;
        }

        /* Extract position of point */
        Eigen::Vector3f currentMapPointWorldPos{};
        if (p_currentMapPoint->getWorldPos(currentMapPointWorldPos) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d P3DwMerge =
            currentMapPointWorldPos.cast<double>();

        /* Transform the point into the current map world frame */
        p_currentMapPoint->posMerge =
            g2oSwCurrentWMerge.map(P3DwMerge).cast<float>();

        /* Transform the points surface normal into current map world frame */
        Eigen::Vector3f currentMapPointNormal{};
        if (p_currentMapPoint->getNormal(currentMapPointNormal) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getNormal returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_currentMapPoint->normalVectorMerge =
            g2oSwCurrentWMerge.rotation().cast<float>() * currentMapPointNormal;

        /* Step to next mapped point */
        itMapPoint++;
    }

    /* Existing current-map landmarks are used as fusion candidates. */
    checkFuseMapPoints.assign(localWindowMapPoints.begin(),
                              localWindowMapPoints.end());

    /* ---------------------------------------------------------------------- *
     * SECTION 10 - TRANSFER THE WELDING WINDOW
     *
     * Both maps are locked simultaneously using std::scoped_lock to guarantee a
     * deadlock-free ownership transfer.
     *
     * The current map remains the authoritative map throughout this section.
     * ---------------------------------------------------------------------- */
    {
        /*!
         * Lock both maps with deadlock-safe acquisition while ownership moves
         */
        std::scoped_lock mapLocks(p_currentMap->mapUpdateMutex,
                                  p_mergeMap->mapUpdateMutex);

        /* ------------------------------------------------------------------ *
         * SECTION 11 - TRANSFER CORRECTED KEYFRAMES
         *
         * Every corrected merge-map keyframe is:
         *
         *      1. Pose corrected.
         *      2. Assigned to the current map.
         *      3. Added to the current-map container.
         *      4. Removed from the merge-map container.
         * ------------------------------------------------------------------ */

        /* For every keyframe in merge map, iterate through and transfer */
        for (KeyFrame *p_keyFrame : mergeConnectedKeyFrames)
        {
            /* Skip invalud keyframes */
            bool keyFrameIsBad4{};
            if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad4) !=
                                      KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_keyFrameMap2 = nullptr;
            if (!(!p_keyFrame || keyFrameIsBad4) &&
                p_keyFrame->getMap(p_keyFrameMap2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!p_keyFrame || keyFrameIsBad4 || p_keyFrameMap2 != p_mergeMap)
            {
                continue;
            }

            /* Store the old pose of the keyframe */
            Sophus::SE3f keyFramePose2{};
            if (p_keyFrame->getPose(keyFramePose2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            p_keyFrame->tcwBefMerge = keyFramePose2;
            Sophus::SE3f keyFramePoseInverse{};
            if (p_keyFrame->getPoseInverse(keyFramePoseInverse) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            p_keyFrame->twcBefMerge = keyFramePoseInverse;

            /* Apply corrected world-to-camera pose in the current-map frame */
            if (p_keyFrame->setPose(p_keyFrame->tcwMerge) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            /* Change keyframe's internal owning-map pointer to current map */
            if (p_keyFrame->updateMap(p_currentMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updateMap returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            /* Record which current keyframe triggered this merge correction */
            p_keyFrame->mergeCorrectedKeyFrameId = p_currentKF->id;

            /* Insert the same keyframe pointer into surviving map container */
            if (p_currentMap->addKeyFrame(p_keyFrame) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Remove the keyframe pointer from the old merge-map container */
            if (p_mergeMap->eraseKeyFrame(p_keyFrame) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* If there is IMU, add velocity */
            bool currentMapIsImuInitialized2{};
            if (p_currentMap->isImuInitialized(currentMapIsImuInitialized2) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isImuInitialized returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (currentMapIsImuInitialized2)
            {
                if (p_keyFrame->setVelocity(p_keyFrame->vwbMerge) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setVelocity returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
        }

        /* ------------------------------------------------------------------ *
         * SECTION 12 - TRANSFER CORRECTED MAP POINTS
         *
         * The imported landmarks have already been transformed into the
         * current-map frame and therefore only require ownership transfer.
         * ------------------------------------------------------------------ */

        /* Iterate over every merge-map point selected for transfer */
        for (MapPoint *p_currentMapPoint : mapPointMerges)
        {
            /* Skip null, invalid, or no-longer merge-owned map points */
            bool currentMapPointIsBad2{};
            if (!(!p_currentMapPoint) &&
                p_currentMapPoint->isBad(currentMapPointIsBad2) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_currentMapPointMap2 = nullptr;
            if (!(!p_currentMapPoint || currentMapPointIsBad2) &&
                p_currentMapPoint->getMap(p_currentMapPointMap2) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!p_currentMapPoint || currentMapPointIsBad2 ||
                p_currentMapPointMap2 != p_mergeMap)
            {
                continue;
            }

            /* Apply position expressed in the surviving current-map frame */
            if (p_currentMapPoint->setWorldPos(p_currentMapPoint->posMerge) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Apply the normal rotated into the surviving current-map frame */
            if (p_currentMapPoint->setNormalVector(
                    p_currentMapPoint->normalVectorMerge) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNormalVector returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Change the map point's internal owner to the current map */
            if (p_currentMapPoint->updateMap(p_currentMap) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updateMap returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            /* Register the same map-point pointer in the surviving map */
            if (p_currentMap->addMapPoint(p_currentMapPoint) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Remove the map-point pointer from the obsolete merge map */
            if (p_mergeMap->eraseMapPoint(p_currentMapPoint) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        /* Set the map to be the current map */
        if (p_atlas->changeMap(p_currentMap) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: changeMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Incrase index tracking the amount of times the maps been changed */
        if (p_currentMap->increaseChangeIndex() !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: increaseChangeIndex returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 13 - REBUILD THE IMPORTED SPANNING TREE
     *
     * The imported spanning tree is attached beneath the surviving current
     * keyframe before the previous parent chain is reversed.
     *
     * This preserves graph connectivity while preventing cyclic parent
     * relationships.
     *
     * The links are updated so the imported merge-map keyframes belong to one
     * connected keyframe graph rooted in the surviving current map. You first
     * transfer and reconnect the merge-map keyframes into the current map’s
     * keyframe graph, then (IN SECTION 14) recompute the covisibility graph
     * from their shared map-point observations
     * ---------------------------------------------------------------------- */

    /* If the oriign keyframe of the merp map is valid */
    KeyFrame *p_mergeMapOriginKeyFrame = nullptr;
    if (p_mergeMap->getOriginKeyFrame(p_mergeMapOriginKeyFrame) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getOriginKeyFrame returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_mergeMapOriginKeyFrame)
    {
        /* Allow the former merge-map root to become a normal tree child */
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

    /* Init variables to track the new child and parent keyframes */
    KeyFrame *p_newChild  = nullptr;
    KeyFrame *p_newParent = nullptr;

    /* Start with the original parent of the matched merge keyframe */
    KeyFrame *p_mergeMatchedKFParent = nullptr;
    if (p_mergeMatchedKF->getParent(p_mergeMatchedKFParent) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParent returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_newChild = p_mergeMatchedKFParent;

    /* The matched merge keyframe becomes the first reversed parent */
    p_newParent = p_mergeMatchedKF;

    /* Attach the matched merge keyframe beneath the current keyframe */
    if (p_mergeMatchedKF->changeParent(p_currentKF) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: changeParent returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Reverse each edge along the original merge-map parent chain */
    while (p_newChild)
    {
        /* Remove the old child edge before reversing its direction */
        if (p_newChild->eraseChild(p_newParent) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseChild returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Save the next original parent before changing this relation */
        KeyFrame *p_oldParent = nullptr;
        if (p_newChild->getParent(p_oldParent) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParent returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Make the former parent a child of the previous keyframe */
        if (p_newChild->changeParent(p_newParent) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: changeParent returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Advance the new-parent pointer one level up the old chain */
        p_newParent = p_newChild;

        /* Continue with the next parent from the original tree chain */
        p_newChild = p_oldParent;
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 14 - REFRESH COVISIBILITY GRAPH
     *
     * Refresh the imported covisibility graph before searching for duplicate
     * landmarks between the two welding windows.
     * ---------------------------------------------------------------------- */

    /* Refresh links for the matched merge-side keyframe */
    if (p_mergeMatchedKF->updateConnections() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Init list of connected keyframes in merge map */
    std::vector<KeyFrame *> vpMergeConnectedKFs;

    /* Retrieve keyframes covisible with the matched merge keyframe */
    std::vector<KeyFrame *> mergeMatchedKFVectorCovisibleKeyFrames{};
    if (p_mergeMatchedKF->getVectorCovisibleKeyFrames(
            mergeMatchedKFVectorCovisibleKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getVectorCovisibleKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    vpMergeConnectedKFs = mergeMatchedKFVectorCovisibleKeyFrames;

    /* Include the matched merge keyframe in the fusion set */
    vpMergeConnectedKFs.push_back(p_mergeMatchedKF);

    /* Fuse duplicate current-map points into corrected merge keyframes */
    if (searchAndFuse(vCorrectedSim3, checkFuseMapPoints) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: searchAndFuse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Refresh covisibility links for current-map local keyframes */
    for (KeyFrame *p_keyFrame : localWindowKeyFrames)
    {
        /* Skip null keyframes and keyframes marked as invalid */
        bool keyFrameIsBad5{};
        if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad5) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad5)
        {
            continue;
        }

        /* Recompute graph connections from shared map-point observations */
        if (p_keyFrame->updateConnections() !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateConnections returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Refresh covisibility links for imported merge-side keyframes */
    for (KeyFrame *p_keyFrame : mergeConnectedKeyFrames)
    {
        /* Skip null keyframes and keyframes marked as invalid */
        bool keyFrameIsBad6{};
        if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad6) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad6)
        {
            continue;
        }

        /* Recompute graph connections from shared map-point observations */
        if (p_keyFrame->updateConnections() !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateConnections returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 15 - LOCAL MERGE OPTIMISATION
     *
     * Perform a local optimisation immediately after the welding window has
     * been merged.
     *
     * Depending on the sensor configuration either:
     *
     *      • MergeInertialBA()
     *      • LoopClosureLocalBundleAdjustment()
     *
     * is executed.
     * ---------------------------------------------------------------------- */

    /* Shared stop flag passed to the selected optimisation routine */
    bool shouldStop = false;

    /* Init list of local keyframes in current window */
    std::vector<KeyFrame *> localCurrentWindowKeyFrames;

    /* Remove keyframes stored by any previous merge operation */
    localCurrentWindowKeyFrames.clear();

    /* Remove merge-connected keyframes stored by earlier processing */
    vpMergeConnectedKFs.clear();

    /* Copy current-side local keyframes into the optimiser vector */
    std::copy(localWindowKeyFrames.begin(),
              localWindowKeyFrames.end(),
              std::back_inserter(localCurrentWindowKeyFrames));

    /* Copy merge-side connected keyframes into the optimiser vector */
    std::copy(mergeConnectedKeyFrames.begin(),
              mergeConnectedKeyFrames.end(),
              std::back_inserter(vpMergeConnectedKFs));

    /* Check whether the active sensor configuration includes an IMU */
    if (p_tracker->sensor == System::IMU_MONOCULAR ||
        p_tracker->sensor == System::IMU_STEREO ||
        p_tracker->sensor == System::IMU_RGBD)
    {
        /* Refine the merged region using visual and inertial constraints */
        if (Optimizer::mergeInertialBA(p_currentKF,
                                       p_mergeMatchedKF,
                                       &shouldStop,
                                       p_currentMap,
                                       vCorrectedSim3) !=
            OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: mergeInertialBA returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    else
    {
        /* Refine the merged region using visual observations only */
        if (Optimizer::loopClosureLocalBundleAdjustment(
                p_mergeMatchedKF,
                vpMergeConnectedKFs,
                localCurrentWindowKeyFrames,
                &shouldStop) != OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: loopClosureLocalBundleAdjustment returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }
    }

    /* Resume local mapping after merge optimisation is complete */
    if (p_localMapper->release() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: release returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 16 - RETRIEVE THE REMAINING MERGE MAP
     *
     * The local welding window has already been transferred.
     *
     * This section retrieves every remaining object that still belongs to the
     * obsolete merge map.
     * ---------------------------------------------------------------------- */

    /* Copy all planes currently owned by the merge map */
    std::vector<geometric::Plane *> currentMapPlanes{};
    if (p_mergeMap->getAllPlanes(currentMapPlanes) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all keyframes currently owned by the merge map */
    std::vector<KeyFrame *> currentMapKeyFrames{};
    if (p_mergeMap->getAllKeyFrames(currentMapKeyFrames) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    const bool hasValidRemainingMergeKeyFrame = std::any_of(
        currentMapKeyFrames.begin(),
        currentMapKeyFrames.end(),
        [p_mergeMap](KeyFrame *p_keyFrame_in)
        {
            bool keyFrameIsBad{};
            if ((p_keyFrame_in != nullptr) &&
                p_keyFrame_in->isBad(keyFrameIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_keyFrameMap = nullptr;
            if ((p_keyFrame_in != nullptr && !keyFrameIsBad) &&
                p_keyFrame_in->getMap(p_keyFrameMap) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            return p_keyFrame_in != nullptr && !keyFrameIsBad &&
                   p_keyFrameMap == p_mergeMap;
        });

    /* Copy all map points currently owned by the merge map */
    std::vector<MapPoint *> currentMapMapPoints{};
    if (p_mergeMap->getAllMapPoints(currentMapMapPoints) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all markers currently owned by the merge map */
    std::vector<semantic::Marker *> currentMapMarkers{};
    if (p_mergeMap->getAllMarkers(currentMapMarkers) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMarkers returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all passages currently owned by the merge map */
    std::vector<vs_graphs::core::semantic::Passage *> currentMapPassages{};
    if (p_mergeMap->getAllPassages(currentMapPassages) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all detected rooms currently owned by the merge map */
    std::vector<semantic::Room *> currentDetectedMapRooms{};
    if (p_mergeMap->getAllDetectedMapRooms(currentDetectedMapRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllDetectedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all marker-based rooms currently owned by the merge map */
    std::vector<semantic::Room *> currentMarkerBasedMapRooms{};
    if (p_mergeMap->getAllMarkerBasedMapRooms(currentMarkerBasedMapRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMarkerBasedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Copy all floors currently owned by the merge map */
    std::vector<semantic::Floor *> currentMapFloors{};
    if (p_mergeMap->getAllFloors(currentMapFloors) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Stop local mapping before any remaining ownership is transferred. */
    if (p_localMapper->requestStop() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestStop returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    for (;;)
    {
        bool localMapperIsStopped2{};
        if (p_localMapper->isStopped(localMapperIsStopped2) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isStopped returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (localMapperIsStopped2)
        {
            break;
        }
        usleep(1000);
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 17 - CORRECT REMAINING MERGE-MAP GEOMETRY
     *
     * Every remaining keyframe and landmark that was not part of the welding
     * window is transformed into the surviving current-map frame prior to graph
     * optimisation.
     * ---------------------------------------------------------------------- */

    /* Process remaining keyframes only when the merge map is not empty */
    if (hasValidRemainingMergeKeyFrame)
    {
        /* Apply monocular scale correction to the remaining merge map */
        if (p_tracker->sensor == System::MONOCULAR)
        {
            /* Lock the merge map while updating its poses and landmarks */
            std::unique_lock<std::mutex> mergeLock(p_mergeMap->mapUpdateMutex);

            /* Correct each remaining merge keyframe into current world */
            for (KeyFrame *p_keyFrame : currentMapKeyFrames)
            {
                /* Skip invalid keyframes or keyframes no longer in this map */
                bool keyFrameIsBad7{};
                if (!(!p_keyFrame) &&
                    p_keyFrame->isBad(keyFrameIsBad7) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                Map *p_keyFrameMap3 = nullptr;
                if (!(!p_keyFrame || keyFrameIsBad7) &&
                    p_keyFrame->getMap(p_keyFrameMap3) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (!p_keyFrame || keyFrameIsBad7 ||
                    p_keyFrameMap3 != p_mergeMap)
                {
                    continue;
                }

                /* Read the keyframe pose in the merge map world frame */
                Sophus::SE3f keyFramePose3{};
                if (p_keyFrame->getPose(keyFramePose3) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                const Sophus::SE3d TiwMerge = keyFramePose3.cast<double>();

                /* Convert the rigid keyframe pose into a unit scale Sim3 */
                const g2o::Sim3 g2oSiwMerge(TiwMerge.unit_quaternion(),
                                            TiwMerge.translation(),
                                            1.0);

                /* Express the keyframe pose in the current world frame */
                const g2o::Sim3 g2oSiwCurrent =
                    g2oSiwMerge * g2oSwMergeWCurrent;

                /* Store the original keyframe pose before correction */
                vNonCorrectedSim3[p_keyFrame] = g2oSiwMerge;

                /* Store the corrected keyframe pose for later processing */
                vCorrectedSim3[p_keyFrame] = g2oSiwCurrent;

                /* Extract the scale introduced by the map correction */
                const double s = g2oSiwCurrent.scale();

                /* Store the applied scale in the keyframe */
                p_keyFrame->correctedScale = s;

                /* Preserve the original world to camera pose */
                Sophus::SE3f keyFramePose4{};
                if (p_keyFrame->getPose(keyFramePose4) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                p_keyFrame->tcwBefMerge = keyFramePose4;

                /* Preserve the original camera to world pose */
                Sophus::SE3f keyFramePoseInverse2{};
                if (p_keyFrame->getPoseInverse(keyFramePoseInverse2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPoseInverse returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                p_keyFrame->twcBefMerge = keyFramePoseInverse2;

                /* Apply the corrected rigid pose in the current world frame */
                if (p_keyFrame->setPose(
                        Sophus::SE3d(g2oSiwCurrent.rotation(),
                                     g2oSiwCurrent.translation() / s)
                            .cast<float>()) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                /* Rotate velocity when the surviving map uses inertial data */
                bool currentMapIsImuInitialized3{};
                if (p_currentMap->isImuInitialized(
                        currentMapIsImuInitialized3) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isImuInitialized returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (currentMapIsImuInitialized3)
                {
                    /* Compute the rotation from old to corrected world frame */
                    const Eigen::Quaternionf Rcor =
                        (g2oSiwCurrent.rotation().inverse() *
                         g2oSiwMerge.rotation())
                            .cast<float>();

                    /* Express the keyframe velocity in the corrected frame */
                    Eigen::Vector3f keyFrameVelocity2{};
                    if (p_keyFrame->getVelocity(keyFrameVelocity2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getVelocity returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_keyFrame->setVelocity(Rcor * keyFrameVelocity2) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setVelocity returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }

            /* Correct each remaining merge landmark into current world */
            for (MapPoint *p_currentMapPoint : currentMapMapPoints)
            {
                /* Skip invalid points or points no longer in this map */
                bool currentMapPointIsBad3{};
                if (!(!p_currentMapPoint) &&
                    p_currentMapPoint->isBad(currentMapPointIsBad3) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                Map *p_currentMapPointMap3 = nullptr;
                if (!(!p_currentMapPoint || currentMapPointIsBad3) &&
                    p_currentMapPoint->getMap(p_currentMapPointMap3) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (!p_currentMapPoint || currentMapPointIsBad3 ||
                    p_currentMapPointMap3 != p_mergeMap)
                {
                    continue;
                }

                /* Read the landmark position in the merge world frame */
                Eigen::Vector3f currentMapPointWorldPos2{};
                if (p_currentMapPoint->getWorldPos(currentMapPointWorldPos2) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWorldPos returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                const Eigen::Vector3d P3DwMerge =
                    currentMapPointWorldPos2.cast<double>();

                Eigen::Vector3f normal_mergeWorld{};
                if (p_currentMapPoint->getNormal(normal_mergeWorld) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getNormal returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                /* Transform the landmark into the current world frame */
                if (p_currentMapPoint->setWorldPos(
                        g2oSwCurrentWMerge.map(P3DwMerge).cast<float>()) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setWorldPos returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                if (p_currentMapPoint->setNormalVector(
                        g2oSwCurrentWMerge.rotation().cast<float>() *
                        normal_mergeWorld) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setNormalVector returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                /* Refresh the point normal and valid viewing depth range */
                if (p_currentMapPoint->updateNormalAndDepth() !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: updateNormalAndDepth returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }

        /* ------------------------------------------------------------------ *
         * SECTION 18 - OPTIMISE THE COMPLETE MERGED GRAPH
         *
         * Once every remaining object has been corrected into the current-map
         * frame, optimise the complete merged graph before changing ownership.
         * ------------------------------------------------------------------ */

        if (p_tracker->sensor != System::MONOCULAR)
        {
            if (Optimizer::optimizeEssentialGraph(p_mergeMatchedKF,
                                                  p_mergeMap,
                                                  localCurrentWindowKeyFrames,
                                                  vpMergeConnectedKFs,
                                                  currentMapKeyFrames,
                                                  currentMapMapPoints,
                                                  g2oSwCurrentWMerge) !=
                OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: optimizeEssentialGraph returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            /*!
             * Skeleton topology is derived from the live Voxblox volume. The
             * map-revision notification invalidates that volume after this
             * correction, so no stale topology snapshot is retained here.
             */
        }
    }

    const bool semanticGeometryWasOptimized =
        hasValidRemainingMergeKeyFrame &&
        p_tracker->sensor != System::MONOCULAR;

    bool semanticGeometryWasPropagated = semanticGeometryWasOptimized;

    /*!
     * Small source maps and monocular merges do not run the merge essential
     * graph. Propagate the final welding-BA pose deltas directly so semantic
     * geometry follows the corrected imported keyframes rather than receiving
     * only the coarse map-level Sim3.
     */
    if (!semanticGeometryWasOptimized)
    {
        KeyFrameAndPose finalKeyFramePoses_worldToCamera;

        for (const auto &[p_keyFrame, poseBefore_worldToCamera] :
             vNonCorrectedSim3)
        {
            (void)poseBefore_worldToCamera;

            bool keyFrameIsBad8{};
            if (!(p_keyFrame == nullptr) &&
                p_keyFrame->isBad(keyFrameIsBad8) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame == nullptr || keyFrameIsBad8)
            {
                continue;
            }

            Sophus::SE3f keyFramePose5{};
            if (p_keyFrame->getPose(keyFramePose5) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            const Sophus::SE3d poseAfter_worldToCamera =
                keyFramePose5.cast<double>();

            /*
             * A monocular map merge can change scale. ORB-SLAM stores the
             * corrected keyframe as SE3 by dividing the Sim3 translation by
             * its scale, so reconstruct the corresponding Sim3 before
             * deriving the semantic world-frame correction. Using a unit
             * scale here would leave planes, rooms and passages at their old
             * size while MapPoints receive the full map Sim3.
             */
            double poseAfterScale = 1.0;

            const KeyFrameAndPose::iterator correctedPoseIterator =
                vCorrectedSim3.find(p_keyFrame);

            if (correctedPoseIterator != vCorrectedSim3.end() &&
                std::isfinite(correctedPoseIterator->second.scale()) &&
                std::abs(correctedPoseIterator->second.scale()) > 1e-12)
            {
                poseAfterScale = correctedPoseIterator->second.scale();
            }

            finalKeyFramePoses_worldToCamera.insert_or_assign(
                p_keyFrame,
                g2o::Sim3(poseAfter_worldToCamera.unit_quaternion(),
                          poseAfterScale *
                              poseAfter_worldToCamera.translation(),
                          poseAfterScale));
        }

        if (utils::utils::Utils::propagateSemanticPoseCorrections(
                p_mergeMap,
                vNonCorrectedSim3,
                finalKeyFramePoses_worldToCamera,
                g2oSwCurrentWMerge) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: propagateSemanticPoseCorrections returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }

        semanticGeometryWasPropagated = true;
    }

    /*
     * A map can retain orphan landmarks after every keyframe in its local
     * window has already moved. Those points still require the map-level Sim3
     * even though there is no essential graph left to optimize.
     */
    if (!hasValidRemainingMergeKeyFrame)
    {
        for (MapPoint *p_mapPoint : currentMapMapPoints)
        {
            bool mapPointIsBad{};
            if (!(p_mapPoint == nullptr) &&
                p_mapPoint->isBad(mapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_mapPointMap = nullptr;
            if (!(p_mapPoint == nullptr || mapPointIsBad) &&
                p_mapPoint->getMap(p_mapPointMap) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapPoint == nullptr || mapPointIsBad ||
                p_mapPointMap != p_mergeMap)
            {
                continue;
            }

            Eigen::Vector3f position_mergeWorld_m{};
            if (p_mapPoint->getWorldPos(position_mergeWorld_m) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            Eigen::Vector3f normal_mergeWorld{};
            if (p_mapPoint->getNormal(normal_mergeWorld) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getNormal returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->setWorldPos(
                    g2oSwCurrentWMerge.map(position_mergeWorld_m.cast<double>())
                        .cast<float>()) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->setNormalVector(
                    g2oSwCurrentWMerge.rotation().cast<float>() *
                    normal_mergeWorld) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNormalVector returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->updateNormalAndDepth() !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateNormalAndDepth returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
    }

    /* ------------------------------------------------------------------ *
     * SECTION 19 - TRANSFER THE REMAINING MAP CONTENTS
     *
     * Every remaining object stored inside the obsolete merge map is
     * transferred into the surviving current map.
     *
     * Object Types
     * ------------------------------------------------------------------
     *  - KeyFrames
     *  - MapPoints
     *  - Planes
     *  - Passages
     *  - Detected Rooms
     *  - Marker Rooms
     *  - Markers
     * ------------------------------------------------------------------ */
    {
        const bool primarySemanticGeometryWasCorrected =
            semanticGeometryWasOptimized || semanticGeometryWasPropagated;

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

        // Get Merge Map Mutex
        std::scoped_lock mapLocks(p_currentMap->mapUpdateMutex,
                                  p_mergeMap->mapUpdateMutex);

        // Loop over the KeyFrames of the current map and move them to the
        // new map
        for (KeyFrame *p_keyFrame : currentMapKeyFrames)
        {
            bool keyFrameIsBad9{};
            if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad9) !=
                                      KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_keyFrameMap4 = nullptr;
            if (!(!p_keyFrame || keyFrameIsBad9) &&
                p_keyFrame->getMap(p_keyFrameMap4) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!p_keyFrame || keyFrameIsBad9 || p_keyFrameMap4 != p_mergeMap)
                continue;

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

        // Loop over the MapPoints of the current map and move them to the
        // new map
        for (MapPoint *p_currentMapPoint : currentMapMapPoints)
        {
            bool currentMapPointIsBad4{};
            if (!(!p_currentMapPoint) &&
                p_currentMapPoint->isBad(currentMapPointIsBad4) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_currentMapPointMap4 = nullptr;
            if (!(!p_currentMapPoint || currentMapPointIsBad4) &&
                p_currentMapPoint->getMap(p_currentMapPointMap4) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!p_currentMapPoint || currentMapPointIsBad4 ||
                p_currentMapPointMap4 != p_mergeMap)
                continue;

            if (p_currentMapPoint->updateMap(p_currentMap) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updateMap returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap->addMapPoint(p_currentMapPoint) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mergeMap->eraseMapPoint(p_currentMapPoint) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        /* -------------------------------------------------------------- *
         * SECTION 20 - TRANSFER SEMANTIC OBJECTS
         *
         * Semantic objects are transformed into the current-map reference
         * frame before ownership is transferred.
         *
         * Geometry is preserved while map ownership is updated.
         * -------------------------------------------------------------- */
        for (geometric::Plane *p_plane : currentMapPlanes)
        {
            /* Skip invalid planes */
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

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                if (p_plane->applyTransform(g2oSwCurrentWMerge) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: applyTransform returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }

            /* Update the map the plane belongs to */
            if (p_plane->setMap(p_currentMap) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }

            /*!
             * Take index size of planes in new map to find an id to add to
             * the map which hasn't been taken.
             */
            if (p_plane->setId(nextPlaneId++) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }

            /* Add the plane to the map new merged plane to the new map */
            if (p_currentMap->addMapPlane(p_plane) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Remove the current plane from the old map */
            if (p_mergeMap->eraseMapPlane(p_plane) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        // Loop over the Markers of the current map and move them to the new
        // map
        for (semantic::Marker *p_marker : currentMapMarkers)
        {
            if (!p_marker)
                continue;

            if (!primarySemanticGeometryWasCorrected)
            {
                if (p_marker->applyTransform(g2oSwCurrentWMerge) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: applyTransform returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
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

        /*!
         * Loop over the passages of the primary map and move them to the
         * secondary map.
         */
        for (vs_graphs::core::semantic::Passage *p_passage : currentMapPassages)
        {
            /* Skip invalid rooms */
            if (p_passage == nullptr)
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                if (p_passage->applyTransform(g2oSwCurrentWMerge) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: applyTransform returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
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
                for (semantic::Room *p_room : currentDetectedMapRooms)
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
                for (semantic::Room *p_room : currentMarkerBasedMapRooms)
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

        /*!
         * Loop over the rooms of the primary map and move them to the
         * secondary map.
         */
        for (vs_graphs::core::semantic::Room *p_currentDetectedRoom :
             currentDetectedMapRooms)
        {
            /* Skip invalid rooms */
            bool currentDetectedRoomIsBad{};
            if (!(p_currentDetectedRoom == nullptr) &&
                p_currentDetectedRoom->isBad(currentDetectedRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentDetectedRoom == nullptr || currentDetectedRoomIsBad)
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                if (p_currentDetectedRoom->applyTransform(g2oSwCurrentWMerge) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: applyTransform returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }

            /* Set the map of the room in the current map */
            if (p_currentDetectedRoom->setMap(p_currentMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }

            /* Add the room to the current map */
            if (p_currentMap->addDetectedMapRoom(p_currentDetectedRoom) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addDetectedMapRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Remove the room from the merged map */
            if (p_mergeMap->eraseDetectedMapRoom(p_currentDetectedRoom) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: eraseDetectedMapRoom returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        // Loop over the Marker-based Rooms of the current map and move them
        // to the new map
        for (vs_graphs::core::semantic::Room *pRoom :
             currentMarkerBasedMapRooms)
        {
            if (!pRoom)
                continue;

            if (!primarySemanticGeometryWasCorrected)
            {
                if (pRoom->applyTransform(g2oSwCurrentWMerge) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: applyTransform returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }

            if (pRoom->setMap(p_currentMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap->addCandidateMapRoom(pRoom) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: addCandidateMapRoom returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_mergeMap->eraseMarkerBasedMapRoom(pRoom) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: eraseMarkerBasedMapRoom returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        for (semantic::Floor *p_floor : currentMapFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }

            if (!semanticGeometryWasPropagated)
            {
                if (p_floor->applyTransform(g2oSwCurrentWMerge) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: applyTransform returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
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

        /*
         * Voxblox topology is derived from a TSDF/ESDF volume and is not an
         * independently mergeable landmark set. Appending snapshots from two
         * map frames creates disconnected duplicate edges and false wall
         * crossings. The external Voxblox node receives the map-revision event,
         * clears its volume, and supplies a fresh snapshot after reintegration.
         */
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

        /* Rebuild imported room-wall index entries before fusion. */
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

        /* Fuse only after every semantic relationship is visible. */
        if (p_sysParams->semSeg.reassociate.enabled)
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

        std::vector<semantic::Room *> importedRooms = currentDetectedMapRooms;
        importedRooms.insert(importedRooms.end(),
                             currentMarkerBasedMapRooms.begin(),
                             currentMarkerBasedMapRooms.end());

        /* Stable semantic identity reconciliation is a merge invariant, not
         * an optional geometry-reassociation feature. */
        if (utils::utils::Utils::fuseDuplicateRoomsAfterMerge(p_currentMap,
                                                              importedRooms) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: fuseDuplicateRoomsAfterMerge returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        if (p_sysParams->semSeg.reassociate.enabled)
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
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: reAssociatePassages returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 22 - FINALISE THE MERGE
     *
     * Merge Complete
     * ----------------------------------------------------------------------
     *      - Current map survives.
     *      - Merge map contains no remaining objects.
     *      - Atlas ownership updated.
     *      - Merge edge inserted.
     *      - Obsolete map removed from the atlas.
     *
     * After this point every surviving SLAM object belongs exclusively to
     * pCurrentMap.
     * ---------------------------------------------------------------------- */

    if (p_mergeMatchedKF->addMergeEdge(p_currentKF) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addMergeEdge returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_currentKF->addMergeEdge(p_mergeMatchedKF) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addMergeEdge returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (p_currentMap->increaseChangeIndex() != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: increaseChangeIndex returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /*!
     * A map merge changes the world-frame poses of previously integrated
     * observations. Notify derived mapping consumers only after ownership,
     * semantic reconciliation, and graph connectivity are fully committed.
     * Voxblox uses this revision to discard TSDF/ESDF state expressed in the
     * pre-merge coordinate frame.
     */
    if (p_currentMap->informNewBigChange() != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: informNewBigChange returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* All surviving objects now belong to pCurrentMap. */
    if (p_atlas->changeMap(p_currentMap) != AtlasStatus::ATLAS_STATUS_SUCCESS)
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
                     "%s: removeBadMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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

    semanticUpdateLock.unlock();
    if (p_localMapper->release() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: release returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    bool currentMapIsImuInitialized4{};
    if ((shouldRelaunchBa) &&
        p_currentMap->isImuInitialized(currentMapIsImuInitialized4) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    unsigned long currentMapKeyFrameCount{};
    if ((shouldRelaunchBa) && !(!currentMapIsImuInitialized4) &&
        p_currentMap->getKeyFrameCount(currentMapKeyFrameCount) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKeyFrameCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    int atlasMaps{};
    if ((shouldRelaunchBa) && !(!currentMapIsImuInitialized4) &&
        (currentMapKeyFrameCount < 200) &&
        p_atlas->countMaps(atlasMaps) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: countMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (shouldRelaunchBa && (!currentMapIsImuInitialized4 ||
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

    local_out = semantic::SemanticMergeDecision::ACCEPT;
    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
