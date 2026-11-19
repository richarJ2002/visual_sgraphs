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
 * @file            runCycleStages.cc
 *
 * @brief           Implements SemanticsManager::runCycleStages(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
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

SemanticsManagerStatus SemanticsManager::runCycleStages(void)
{
    /* Declare pointer which will hold addres of current map */
    map::Map *p_atlasCurrentMap = nullptr;

    /* Extract the current map */
    if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Reset the temporal state of the map */
    if (resetTemporalStateForMap(p_atlasCurrentMap) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: resetTemporalStateForMap returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Stage 1, plane intake: camera centre and this cycle's planes */
    std::vector<geometric::Plane *> intakePlanes{};
    if (runPlaneIntake(intakePlanes) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: runPlaneIntake returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Declare variable which tracks the bootstrap result of the map */
    SemanticsManager::ActiveMapBootstrapResult bootstrapResult{};

    /* Ensure active map bootstraps the SGraph hierarchy */
    if (ensureActiveMapBootstrapHierarchy(bootstrapResult) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: ensureActiveMapBootstrapHierarchy returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Stage 2, plane checks: the plane rule table on the intake planes */
    if (runPlaneChecks(intakePlanes) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: runPlaneChecks returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Stage 3, plane fusion: duplicate planes fused before room edges */
    if (runPlaneFusion() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: runPlaneFusion returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Stage 4, passage detection: passage geometry before room edges */
    if (runPassageDetection() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: runPassageDetection returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Room tracking input: passage crossings by the camera path, from
     * the rooms and passages as the previous cycle left them; may seed
     * or reset the current room before the association offers it walls.
     */
    if (p_sysParams->semSeg.enablePassageDetection &&
        updateTraversalEvidence(p_atlas) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateTraversalEvidence returned a failure "
                     "status although it cannot fail; continuing as "
                     "before.",
                     __func__);
    }

    /* Stage 5, room association: walls to rooms, once per cycle */
    if (runRoomAssociation() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: runRoomAssociation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Link each wall's opposite-facing twin (plane fusion stage), now
     * that ownership has settled for this cycle. Must run before
     * validateRoomBoundaries() so boundary/corner logic can rely on
     * current twin identity. */
    if (reconcileWallFacePairs() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: reconcileWallFacePairs returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Stage 7, room validation: boundaries, centroids, floors */
    if (runRoomValidation() !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: runRoomValidation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Room candidate generation is pre-verification only. The legacy
     * tag-and-wall-transfer entry point remains disabled. */

    /* Advance the room-state machine. It consumes the
     * traversal crossings recorded above and reports accepted/rejected
     * transitions. It is read-only with respect to the room id members. */
    const std::chrono::duration<double> roomTrackerElapsed =
        std::chrono::steady_clock::now().time_since_epoch();
    if (updateRoomTrackerState(roomTrackerElapsed.count()) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateRoomTrackerState returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* The current room is final for this cycle: the key frames taken since
     * the last cycle are linked to it. */
    if (linkNewKeyFramesToCurrentRoom(p_atlasCurrentMap) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: linkNewKeyFramesToCurrentRoom returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
