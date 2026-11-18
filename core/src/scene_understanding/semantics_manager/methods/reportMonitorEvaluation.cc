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
 * @file            reportMonitorEvaluation.cc
 *
 * @brief           Implements SemanticsManager::reportMonitorEvaluation(),
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

SemanticsManagerStatus SemanticsManager::reportMonitorEvaluation(
    const semantic::SemanticGraphSnapshot &snapshot_in,
    std::uint64_t                          semanticCycle_in,
    const std::optional<int>              &currentMapRevision_in)
{
    const std::chrono::steady_clock::time_point evaluationStart =
        std::chrono::steady_clock::now();
    semantic::AxiomEvaluationReport evaluationReport{};
    if (semantic::evaluateState(snapshot_in, evaluationReport) !=
        semantic::SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateState returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::MapCompletenessResult> completenessResults{};
    if (semantic::evaluateMapCompleteness(snapshot_in, completenessResults) !=
        semantic::SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateMapCompleteness returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::string topologyDigest{};
    if (semantic::sha256HexDigest(
            semantic::serializeSnapshotTopologyOnly(snapshot_in).dump(),
            topologyDigest) !=
        semantic::Sha256DigestStatus::SHA256_DIGEST_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: sha256HexDigest returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::string fullGeometryDigest{};
    if (semantic::sha256HexDigest(
            semantic::serializeSnapshotFullGeometry(snapshot_in).dump(),
            fullGeometryDigest) !=
        semantic::Sha256DigestStatus::SHA256_DIGEST_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: sha256HexDigest returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const std::chrono::milliseconds evaluationDuration =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - evaluationStart);

    if (semanticReportCache.update(snapshot_in,
                                   evaluationReport,
                                   completenessResults,
                                   semanticCycle_in,
                                   snapshot_in.currentMapId,
                                   currentMapRevision_in,
                                   topologyDigest,
                                   fullGeometryDigest,
                                   evaluationDuration) !=
        semantic::SemanticReportCacheStatus::
            SEMANTIC_REPORT_CACHE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: update returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    semantic::SemanticReportCacheEntry semanticReportCacheGetLatest{};
    if (semanticReportCache.getLatest(semanticReportCacheGetLatest) !=
        semantic::SemanticReportCacheStatus::
            SEMANTIC_REPORT_CACHE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLatest returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (logSemanticDiagnostics(semanticReportCacheGetLatest) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: logSemanticDiagnostics returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
