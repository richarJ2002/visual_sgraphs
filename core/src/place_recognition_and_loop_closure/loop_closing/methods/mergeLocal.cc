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

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "mapping/local_mapping/objects/LocalMappingStatusEnum.h"
#include "optimization/optimizer/objects/OptimizerStatusEnum.h"
#include "optimization/optimizer/optimizer.h"
#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingClass.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticMergeDecisionEnum.h"

#include "../private_functions/private_functions.h"
#include "estimation/tracking/tracking.h"
#include "execution/system/system.h"
#include "mapping/local_mapping/local_mapping.h"

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

LoopClosingStatus
    LoopClosing::mergeLocal(semantic::SemanticMergeDecision &local_out)
{
    PROFILE_FUNCTION();
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
    map::Map *p_currentMap = nullptr;
    map::Map *p_mergeMap   = nullptr;
    if (!selectMergeMaps(p_currentKF,
                         p_mergeMatchedKF,
                         p_atlas,
                         p_currentMap,
                         p_mergeMap))
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

    /* Stop local mapping */
    stopLocalMapping(p_localMapper);

    /*
     * Prevent semantic worker threads from modifying either graph while map
     * frames, ownership, and cross-entity references are being changed.
     */
    std::unique_lock<std::mutex> semanticUpdateLock{};
    if (!lockAndRevalidateMergeCandidate(p_atlas,
                                         p_localMapper,
                                         p_currentKF,
                                         p_mergeMatchedKF,
                                         p_currentMap,
                                         p_mergeMap,
                                         semanticUpdateLock))
    {
        local_out = semantic::SemanticMergeDecision::REJECT;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    /* Maps merge-world coordinates into current-world coordinates and back. */
    const g2o::Sim3 g2oSwCurrentWMerge =
        calculateMergeMapTransform(p_currentKF, mg2oMergeScw);
    const g2o::Sim3 g2oSwMergeWCurrent = g2oSwCurrentWMerge.inverse();

    semantic::SemanticMergeGateResult semanticMergeGate{};
    evaluateSemanticMergeGate(p_currentMap,
                              p_mergeMap,
                              g2oSwCurrentWMerge,
                              semanticMergeGate);
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
        local_out = semanticMergeGate.decision;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    /* Discard queued keyframes only after every merge rejection gate passed. */
    if (p_localMapper->emptyQueue() !=
        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: emptyQueue returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Update the connections of the current keyframe */
    if (p_currentKF->updateConnections() !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Welding windows: the fixed current side and the absorbed merge side. */
    std::set<keyframe::KeyFrame *>  localWindowKeyFrames;
    std::set<map_point::MapPoint *> localWindowMapPoints;
    buildCurrentLocalWindow(p_currentKF,
                            p_currentMap,
                            p_mergeMap,
                            localWindowKeyFrames,
                            localWindowMapPoints);

    std::set<keyframe::KeyFrame *>  mergeConnectedKeyFrames;
    std::set<map_point::MapPoint *> mapPointMerges;
    buildMergeLocalWindow(p_mergeMatchedKF,
                          p_currentMap,
                          p_mergeMap,
                          mergeConnectedKeyFrames,
                          mapPointMerges);

    /*
     * The imported keyframes keep their original poses until the transfer;
     * only the corrected values are cached for it.
     */
    KeyFrameAndPose vNonCorrectedSim3;
    KeyFrameAndPose vCorrectedSim3;
    correctMergeKeyFramePoses(mergeConnectedKeyFrames,
                              p_currentMap,
                              p_mergeMap,
                              g2oSwMergeWCurrent,
                              vNonCorrectedSim3,
                              vCorrectedSim3);
    correctMergeMapPoints(mapPointMerges, p_mergeMap, g2oSwCurrentWMerge);

    /* Existing current-map landmarks are used as fusion candidates. */
    std::vector<map_point::MapPoint *> checkFuseMapPoints(
        localWindowMapPoints.begin(),
        localWindowMapPoints.end());

    transferMergeWindow(p_atlas,
                        p_currentKF,
                        p_currentMap,
                        p_mergeMap,
                        mergeConnectedKeyFrames,
                        mapPointMerges);
    reverseMergeSpanningTree(p_currentKF, p_mergeMatchedKF, p_mergeMap);

    /* Refresh the matched keyframe links before searching for duplicates. */
    if (p_mergeMatchedKF->updateConnections() !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Fuse duplicate current-map points into corrected merge keyframes */
    if (searchAndFuse(vCorrectedSim3, checkFuseMapPoints) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: searchAndFuse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Refresh covisibility links of both welding windows. */
    std::vector<keyframe::KeyFrame *> localCurrentWindowKeyFrames(
        localWindowKeyFrames.begin(),
        localWindowKeyFrames.end());
    std::vector<keyframe::KeyFrame *> vpMergeConnectedKFs(
        mergeConnectedKeyFrames.begin(),
        mergeConnectedKeyFrames.end());
    updateKeyFrameConnections(localCurrentWindowKeyFrames);
    updateKeyFrameConnections(vpMergeConnectedKFs);

    /* Local merge optimisation of the welded region. */
    optimizeMergeWindow(p_tracker,
                        p_currentKF,
                        p_mergeMatchedKF,
                        p_currentMap,
                        vCorrectedSim3,
                        vpMergeConnectedKFs,
                        localCurrentWindowKeyFrames);

    /* Resume local mapping after merge optimisation is complete */
    if (p_localMapper->release() !=
        local_mapping::LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: release returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* Everything that still belongs to the obsolete merge map. */
    MergeMapContents mergeMapContents;
    collectMergeMapContents(p_mergeMap, mergeMapContents);
    const bool hasValidRemainingMergeKeyFrame =
        hasMergeMapKeyFrame(mergeMapContents.keyFrames, p_mergeMap);

    /* Stop local mapping before any remaining ownership is transferred. */
    stopLocalMapping(p_localMapper);

    /* Process remaining keyframes only when the merge map is not empty */
    if (hasValidRemainingMergeKeyFrame)
    {
        /* Apply monocular scale correction to the remaining merge map */
        if (p_tracker->sensor == system::System::MONOCULAR)
        {
            correctMergeMapMonocular(p_currentMap,
                                     p_mergeMap,
                                     mergeMapContents,
                                     g2oSwCurrentWMerge,
                                     g2oSwMergeWCurrent,
                                     vNonCorrectedSim3,
                                     vCorrectedSim3);
        }
    }

    const bool semanticGeometryWasOptimized =
        hasValidRemainingMergeKeyFrame &&
        p_tracker->sensor != system::System::MONOCULAR;

    if (semanticGeometryWasOptimized)
    {
        /*
         * Skeleton topology is derived from the live Voxblox volume. The
         * map-revision notification invalidates that volume after this
         * correction, so no stale topology snapshot is retained here.
         */
        if (optimizer::Optimizer::optimizeEssentialGraph(
                p_mergeMatchedKF,
                p_mergeMap,
                localCurrentWindowKeyFrames,
                vpMergeConnectedKFs,
                mergeMapContents.keyFrames,
                mergeMapContents.mapPoints,
                g2oSwCurrentWMerge) !=
            optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: optimizeEssentialGraph returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    else
    {
        /*
         * Small source maps and monocular merges do not run the merge
         * essential graph. Propagate the final welding-BA pose deltas directly
         * so semantic geometry follows the corrected imported keyframes rather
         * than receiving only the coarse map-level Sim3.
         */
        propagateMergedPoseCorrections(p_mergeMap,
                                       vNonCorrectedSim3,
                                       vCorrectedSim3,
                                       g2oSwCurrentWMerge);
    }

    /*
     * A map can retain orphan landmarks after every keyframe in its local
     * window has already moved. Those points still require the map-level Sim3
     * even though there is no essential graph left to optimize.
     */
    if (!hasValidRemainingMergeKeyFrame)
    {
        correctMapPointsToCurrentWorld(mergeMapContents.mapPoints,
                                       p_mergeMap,
                                       g2oSwCurrentWMerge);
    }

    /* Transfer everything left in the merge map into the surviving map. */
    {
        std::scoped_lock mapLocks(p_currentMap->mapUpdateMutex,
                                  p_mergeMap->mapUpdateMutex);

        transferKeyFramesAndMapPoints(p_currentMap,
                                      p_mergeMap,
                                      mergeMapContents.keyFrames,
                                      mergeMapContents.mapPoints);
        transferSemanticEntities(p_currentMap,
                                 p_mergeMap,
                                 mergeMapContents,
                                 false);
        rebuildSemanticIndexes(p_currentMap);

        /* Fuse only after every semantic relationship is visible. */
        reassociateMergedSemantics(p_atlas,
                                   p_currentMap,
                                   p_mergeMap,
                                   p_sysParams,
                                   mergeMapContents,
                                   semanticMergeGate.p_survivingAnchorRoom,
                                   semanticMergeGate.p_absorbedAnchorRoom);
    }

    /*
     * Merge complete: the current map survives, the merge map contains no
     * remaining objects and is removed from the atlas.
     */
    commitMapMerge(p_atlas,
                   p_currentKF,
                   p_mergeMatchedKF,
                   p_currentMap,
                   p_mergeMap);
    recordAcceptedMerge(p_currentMap,
                        p_mergeMap,
                        semanticMergeGate.floorDecision,
                        g2oSwCurrentWMerge,
                        p_currentKF->getCaptureTime());

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

    local_out = semantic::SemanticMergeDecision::ACCEPT;
    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} /* namespace loop_closing */
} /* namespace core */
} /* namespace vs_graphs */
