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
 * @file            captureMonitorSnapshot.cc
 *
 * @brief           Implements SemanticsManager::captureMonitorSnapshot(),
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

SemanticsManagerStatus SemanticsManager::captureMonitorSnapshot(
    semantic::SemanticGraphSnapshot &snapshot_out,
    std::optional<int>              &currentMapRevision_out)
{
    currentMapRevision_out.reset();

    if (semantic::captureSemanticGraphSnapshot(p_atlas, snapshot_out) !=
        semantic::SemanticGraphSnapshotStatus::
            SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: captureSemanticGraphSnapshot returned a failure status "
            "although it cannot fail; continuing as before.",
            __func__);
    }
    std::vector<semantic::UnresolvedWallHypothesisRecord>
        captureUnresolvedWallHypotheses2{};
    if (captureUnresolvedWallHypotheses(captureUnresolvedWallHypotheses2) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: captureUnresolvedWallHypotheses returned a failure status "
            "although it cannot fail; continuing as before.",
            __func__);
    }
    snapshot_out.managerPrivateUnresolvedWallHypotheses =
        captureUnresolvedWallHypotheses2;
    snapshot_out.managerPrivateUnresolvedWallHypothesesReason =
        semantic::UnavailableReason::NONE;

    if (snapshot_out.currentMapId.has_value())
    {
        map::Map *p_currentMap = nullptr;
        if (p_atlas->getCurrentMap(p_currentMap) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long currentMapId2{};
        if ((p_currentMap != nullptr) && p_currentMap->getId(currentMapId2) !=
                                             map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap != nullptr &&
            currentMapId2 == *snapshot_out.currentMapId)
        {
            int currentMapMapChangeIndex{};
            if (p_currentMap->getMapChangeIndex(currentMapMapChangeIndex) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapChangeIndex returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            currentMapRevision_out = currentMapMapChangeIndex;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
