/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            run.cc
 *
 * @brief           Implements SemanticsManager::run(), declared in
 *                  SemanticsManager.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/SemanticAxiomEvaluatorStatusEnum.h"
#include "scene_understanding/semantic/semantic_candidates/objects/SemanticCandidatesStatusEnum.h"
#include "scene_understanding/semantic/semantic_graph_snapshot/objects/SemanticGraphSnapshotStatusEnum.h"
#include "scene_understanding/semantic/semantic_graph_snapshot/objects/UnavailableReasonEnum.h"
#include "scene_understanding/semantic/semantic_report_cache/objects/SemanticReportCacheStatusEnum.h"
#include "scene_understanding/semantic/semantic_report_cache/semantic_report_cache.h"
#include "scene_understanding/semantic/sha256_digest/objects/Sha256DigestStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantic/semantic_axiom_evaluator/semantic_axiom_evaluator.h"
#include "scene_understanding/semantic/semantic_candidates/semantic_candidates.h"
#include "scene_understanding/semantic/semantic_canonical_serilization/semantic_canonical_serilization.h"
#include "scene_understanding/semantic/semantic_graph_snapshot/semantic_graph_snapshot.h"
#include "scene_understanding/semantic/sha256_digest/sha256_digest.h"

#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <chrono>
#include <map>
#include <rclcpp/logging.hpp>
#include <string>
#include <thread>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::run(void)
{
    /* Init counter to track summary cycle */
    std::size_t summaryCycle = 0U;

    while (true)
    {

        /* ------------------------------------------------------------------ *
         * DETERMINE IF SYSTEM IS STILL RUNNING
         * ------------------------------------------------------------------ */

        /* Declare variable which will track if a shutdown should happen */
        bool shouldFinish{};

        /* Extract status to see if system should finish */
        if (checkFinish(shouldFinish) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkFinish returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* If system should finish, break while loop */
        if (shouldFinish)
        {
            break;
        }

        /* ------------------------------------------------------------------ *
         * SGRAPH LOCKING INFORMATION
         * ------------------------------------------------------------------ */

        /* Find the current time of the loop */
        const std::chrono::steady_clock::time_point start =
            std::chrono::steady_clock::now();

        /* Declare variable which tracks the semantic lock on SGraph */
        std::unique_lock<std::mutex> semanticUpdateLock{};

        /*!
         * Treat one hierarchy update as an atomic semantic transaction. Map
         * merging takes the same atlas-owned lock before changing frames or
         * ownership, so this cycle can never traverse a half-merged graph.
         */
        if (p_atlas->acquireSemanticUpdateLock(semanticUpdateLock) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: acquireSemanticUpdateLock returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /*!
         * The cycle edits rooms, passages and storeys in place, so it holds
         * the exclusive lock of every map graph until the sweep has freed the
         * flagged nodes (lock order: semantic update lock, then graph locks).
         * Readers on other threads wait for it or, on a tracking thread, skip
         * their read.
         */
        GraphLockScope graphLockScope{};
        if (lockAllGraphs(GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
                          graphLockScope) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: lockAllGraphs returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* ------------------------------------------------------------------ *
         * CYCLE
         * ------------------------------------------------------------------ */

        /* Incriment summary cycle and store in pipelineSemanticCycle */
        pipelineSemanticCycle = ++summaryCycle;

        /* Stages 1-7 and room tracking, in pipeline order */
        if (runCycleStages() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: runCycleStages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Merge candidates: generated and verified, never merged here */
        if (verifyMergeCandidates() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: verifyMergeCandidates returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* SG_PIPELINE heartbeat of this completed transaction */
        if (logPipelineHeartbeat() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: logPipelineHeartbeat returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* ------------------------------------------------------------------ *
         * SEMANTIC MONITOR BOUNDARY: capture a complete, pointer-free
         * snapshot plus manager-private evidence while the semantic-update
         * lock is still held, unlock, then evaluate/digest/cache/log
         * outside the lock. Read-only with respect to inference,
         * ownership, passage, room, and completeness decisions -- this
         * never mutates Atlas/Map/Room/Wall/Passage state.
         * ------------------------------------------------------------------ */
        const std::uint64_t semanticCycle = pipelineSemanticCycle;

        semantic::SemanticGraphSnapshot snapshot{};
        std::optional<int>              currentMapRevision{};
        if (captureMonitorSnapshot(snapshot, currentMapRevision) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: captureMonitorSnapshot returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Deletion ends the cycle: flagged rooms, passages and storeys leave
         * the scene graphs and are freed here, under the same lock that every
         * writer holds, so no cycle step can still use them. */
        if (sweepBadNodes() !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: sweepBadNodes returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Tracking never waits for the graph lock, so when it loses the map
         * while a cycle runs it exports the room context cached here, as plain
         * data, at the end of the last cycle. */
        if (p_atlas->refreshRoomContextCache() !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: refreshRoomContextCache returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }

        /* Unlock before any evaluation/serialization/caching/logging work --
         * the monitor must never hold the semantic-update lock while doing
         * read-only diagnostic work. */
        releaseGraphLockEntries(graphLockScope.entries);
        semanticUpdateLock.unlock();

        if (reportMonitorEvaluation(snapshot,
                                    semanticCycle,
                                    currentMapRevision) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: reportMonitorEvaluation returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Find the time after it took to run the loop */
        const std::chrono::steady_clock::time_point end =
            std::chrono::steady_clock::now();

        /* Calculate the elapsed time */
        const std::chrono::duration<double> elapsed = end - start;

        /* Find how much longer in the loop is left */
        const double remainingSeconds =
            p_sysParams->semanticsManager.runInterval_s - elapsed.count();

        /* If there is remaining time, sleep until next loop cycle */
        if (remainingSeconds > 0.0)
        {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(remainingSeconds));
        }
        else
        {
            /* Let a waiting merge or segmentation transaction acquire next. */
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    /* Signal shutdown completion to ~System. */
    if (setFinish() != SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setFinish returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
