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
 * @file            verifyMergeCandidates.cc
 *
 * @brief           Implements SemanticsManager::verifyMergeCandidates(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
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

SemanticsManagerStatus SemanticsManager::verifyMergeCandidates(void)
{
    /* Verification-gated merge is not enabled here. In particular, a tag
     * match must never activate a merge. */
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>>
        copiedContext{};
    if (p_atlas->copyRoomContextHistory(copiedContext) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: copyRoomContextHistory returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    map::Map *p_candidateMap = nullptr;
    if (p_atlas->getCurrentMap(p_candidateMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_candidateMap != nullptr)
    {
        std::vector<semantic::RoomContextSnapshot> currentSnapshots{};
        if (p_atlas->copyRoomContextForMap(p_candidateMap, currentSnapshots) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: copyRoomContextForMap returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        unsigned long candidateMapId{};
        if (p_candidateMap->getId(candidateMapId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        copiedContext[candidateMapId] = currentSnapshots;
    }
    semantic::SemanticCandidateConfig candidateConfiguration;
    candidateConfiguration.topK = p_sysParams->candidateGen.topK;
    candidateConfiguration.candidatePairCap =
        p_sysParams->candidateGen.candidatePairCap;
    candidateConfiguration.topologyNodesCap =
        p_sysParams->candidateGen.topologyNodesCap;
    candidateConfiguration.globalFallbackCap =
        p_sysParams->candidateGen.globalFallbackCap;
    candidateConfiguration.weightAngle = p_sysParams->candidateGen.weightAngle;
    candidateConfiguration.weightExtent =
        p_sysParams->candidateGen.weightExtent;
    candidateConfiguration.weightAperture =
        p_sysParams->candidateGen.weightAperture;
    candidateConfiguration.weightTopology =
        p_sysParams->candidateGen.weightTopology;
    candidateConfiguration.angleMissingPenalty =
        p_sysParams->candidateGen.angleMissingPenalty;
    candidateConfiguration.extentMissingPenalty =
        p_sysParams->candidateGen.extentMissingPenalty;
    candidateConfiguration.apertureMissingPenalty =
        p_sysParams->candidateGen.apertureMissingPenalty;
    candidateConfiguration.ambiguityMargin =
        p_sysParams->candidateGen.ambiguityMargin;
    candidateConfiguration.angleTolerance_rad =
        p_sysParams->candidateGen.angleTolerance_rad;
    candidateConfiguration.runtimeBudget_ms =
        p_sysParams->candidateGen.runtimeBudget_ms;
    candidateConfiguration.descriptorElementsCap =
        p_sysParams->candidateGen.descriptorElementsCap;
    candidateConfiguration.topoRefinementIters =
        p_sysParams->candidateGen.topoRefinementIters;
    /* The "last-confirmed room" anchor for adjacency-
     * prioritised candidate search. -1 (unset) maps to no anchor. */
    int lastKnownRoomIdSnapshot{};
    if (getLastKnownRoomId(lastKnownRoomIdSnapshot) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLastKnownRoomId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const std::optional<int> anchorRoomId =
        lastKnownRoomIdSnapshot >= 0
            ? std::optional<int>(lastKnownRoomIdSnapshot)
            : std::nullopt;
    std::vector<semantic::SemanticCandidate> candidates{};
    if (semantic::SemanticCandidates::generate(copiedContext,
                                               candidates,
                                               candidateConfiguration,
                                               anchorRoomId) !=
        semantic::SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: generate returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "[SemMgr] semantic_candidates count=" << candidates.size()
              << std::endl;

    /* Run the geometric verifier on the single best candidate and feed
     * the resulting VerificationVerdict to roomTracker via
     * submitVerificationVerdict(). This still only makes the *verdict*
     * real -- it must not trigger a merge or otherwise mutate
     * the Atlas; that trigger is a separate, deliberately gated step. */
    if (evaluateTopCandidateVerification(candidates) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateTopCandidateVerification returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
