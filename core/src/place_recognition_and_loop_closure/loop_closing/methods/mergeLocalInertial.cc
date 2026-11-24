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

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/map/map.h"
#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingClass.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticMergeDecisionEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "../private_functions/private_functions.h"

#include <mutex>
#include <rclcpp/logging.hpp>
#include <set>
#include <vector>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{
namespace loop_closing
{

LoopClosingStatus LoopClosing::mergeLocalInertial(
    semantic::SemanticMergeDecision &localInertial_out)
{
    PROFILE_FUNCTION();

    /* Reject stale place-recognition candidates before stopping workers */
    map::Map *p_currentMap = nullptr;
    map::Map *p_mergeMap   = nullptr;
    if (!selectMergeMaps(p_currentKF,
                         p_mergeMatchedKF,
                         p_atlas,
                         p_currentMap,
                         p_mergeMap))
    {
        localInertial_out = semantic::SemanticMergeDecision::REJECT;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;

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
    const bool shouldRelaunchBa = wasRunning;

    /* Wait until Local Mapping has effectively stopped */
    stopLocalMapping(p_localMapper);

    /* Keep the inertial semantic transfer atomic for the complete merge. */
    std::unique_lock<std::mutex> semanticUpdateLock{};
    if (!lockAndRevalidateMergeCandidate(p_atlas,
                                         p_localMapper,
                                         p_currentKF,
                                         p_mergeMatchedKF,
                                         p_currentMap,
                                         p_mergeMap,
                                         semanticUpdateLock))
    {
        localInertial_out = semantic::SemanticMergeDecision::REJECT;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    SemanticsManager *p_semanticsManager = nullptr;
    if (p_atlas->getSemanticsManager(p_semanticsManager) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSemanticsManager returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    semantic::SemanticMergeGateResult semanticMergeGate{};
    {
        /* The gate only reads the two graphs. */
        GraphLockScope gateGraphScope{};
        lockMergeGraphs(p_atlas,
                        p_currentMap,
                        p_mergeMap,
                        GraphLockMode::GRAPH_LOCK_MODE_SHARED,
                        gateGraphScope);
        evaluateSemanticMergeGate(p_atlas,
                                  p_currentMap,
                                  p_mergeMap,
                                  oldCorrectedPose.inverse(),
                                  semanticMergeGate);
    }
    if (semanticMergeGate.decision != semantic::SemanticMergeDecision::ACCEPT)
    {
        resumeMergeWorkers(semanticUpdateLock, p_localMapper);
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

    applyInertialMapAlignment(p_localMapper,
                              p_tracker,
                              p_currentKF,
                              p_currentMap,
                              p_semanticsManager,
                              oldCorrectedPose);

    unsigned long keyFrameNewCountValue{};
    if (p_currentMap->getKeyFrameCount(keyFrameNewCountValue) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKeyFrameCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const int keyFrameNewCount = static_cast<int>(keyFrameNewCountValue);

    initializeInertialMap(p_tracker, p_currentMap);

    /* Load KFs and MPs from merge map */
    MergeMapContents           mergeMapContents;
    AbsorbedSceneGraphContents absorbedContents{};
    /* Both graphs are edited in place until the merge is committed: exclusive
     * locks, taken inside the block below after the map update mutexes. */
    GraphLockScope             mergeGraphScope{};
    {
        /*!
         * Acquire both map-update mutexes without imposing an unsafe order.
         *
         * @note            Get Merge map::Map Mutex and stop tracking.
         */
        std::scoped_lock mapUpdateLocks(p_currentMap->mapUpdateMutex,
                                        p_mergeMap->mapUpdateMutex);
        lockMergeGraphs(p_atlas,
                        p_currentMap,
                        p_mergeMap,
                        GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
                        mergeGraphScope);

        collectMergeMapContents(p_mergeMap, mergeMapContents);
        transferKeyFramesAndMapPoints(p_currentMap,
                                      p_mergeMap,
                                      mergeMapContents.keyFrames,
                                      mergeMapContents.mapPoints);
        transferPlanes(p_currentMap, p_mergeMap, mergeMapContents.planes);
        if (p_semanticsManager->transferSceneGraphContents(p_currentMap,
                                                           p_mergeMap,
                                                           true,
                                                           absorbedContents) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: transferSceneGraphContents found no scene graph "
                         "for a merged map; continuing as before.",
                         __func__);
        }

        /* Rebuild derived free-space topology in the corrected map frame. */
        clearDerivedSkeleton(p_currentMap);

        /* Save non corrected poses (already merged maps) */
        collectKeyFramePoses(p_currentMap, NonCorrectedSim3);
    }

    reverseMergeSpanningTree(p_currentKF, p_mergeMatchedKF, p_mergeMap);

    /*!
     * map_point::MapPoint vector from current map to allow to fuse duplicated
     * points with the old map (merge)
     */
    std::vector<map_point::MapPoint *> checkFuseMapPoints;
    std::vector<keyframe::KeyFrame *>  currentConnectedKeyFrames;
    gatherInertialFusionCandidates(p_mergeMatchedKF,
                                   p_currentKF,
                                   mergeConnectedKFs,
                                   currentConnectedKeyFrames,
                                   checkFuseMapPoints);
    if (searchAndFuse(currentConnectedKeyFrames, checkFuseMapPoints) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: searchAndFuse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    updateKeyFrameConnections(currentConnectedKeyFrames);
    updateKeyFrameConnections(mergeConnectedKFs);

    /* A sufficiently established current map can support inertial welding BA.
     */
    if (runInertialMergeBundleAdjustment(p_tracker,
                                         p_mergeMatchedKF,
                                         p_currentMap,
                                         CorrectedSim3,
                                         keyFrameNewCount))
    {
        propagateInertialPoseCorrections(p_currentMap,
                                         p_semanticsManager,
                                         NonCorrectedSim3,
                                         CorrectedSim3);
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
    if (p_semanticsManager->reassociateMergedSceneGraph(
            p_atlas,
            p_currentMap,
            p_mergeMap,
            p_params,
            absorbedContents,
            semanticMergeGate.p_survivingAnchorRoom,
            semanticMergeGate.p_absorbedAnchorRoom) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: reassociateMergedSceneGraph found no scene graph "
                     "for a merged map; continuing as before.",
                     __func__);
    }

    /*!
     * Inertial welding changes the same derived-map coordinate contract as a
     * visual merge. The commit increments the externally observed revision
     * after all corrected poses and semantic entities have become
     * authoritative.
     */
    commitMapMerge(p_atlas,
                   p_currentKF,
                   p_mergeMatchedKF,
                   p_currentMap,
                   p_mergeMap);
    recordAcceptedMerge(p_currentMap,
                        p_mergeMap,
                        semanticMergeGate.floorDecision,
                        oldCorrectedPose.inverse(),
                        p_currentKF->getCaptureTime());

    /* The semantic graph and atlas::Atlas ownership are now stable for other
     * workers.
     */
    releaseGraphLockEntries(mergeGraphScope.entries);
    resumeMergeWorkers(semanticUpdateLock, p_localMapper);

    if (shouldRelaunchGlobalBundleAdjustment(shouldRelaunchBa,
                                             p_currentMap,
                                             p_atlas))
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

} /* namespace loop_closing */
} /* namespace core */
} /* namespace vs_graphs */
