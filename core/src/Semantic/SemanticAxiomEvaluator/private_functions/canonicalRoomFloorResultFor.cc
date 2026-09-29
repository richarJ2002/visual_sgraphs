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
 * @file            canonicalRoomFloorResultFor.cc
 *
 * @brief           Implements canonicalRoomFloorResultFor(), declared in
 *                  private_functions.h.
 *
 *                  Returns the full aggregate AxiomResult
 *                  (FAIL/UNKNOWN/PASS) of the endpoint room's own canonical
 *                  evaluateOneRoomFloorReciprocity() findings (rather than
 *                  a lossy FAIL-or-not bool), so
 *                  evaluatePassageFloorAgreement() can propagate a canonical
 *                  UNKNOWN (not only FAIL) before ever comparing floor keys.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    canonicalRoomFloorResultFor(const ResolvedRoomEndpoint  &endpoint_in,
                                const SemanticGraphSnapshot &snapshot_in,
                                AxiomResult &roomFloorResult_out)
{
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId != endpoint_in.key->mapId)
        {
            continue;
        }
        const RoomRecord *p_room = nullptr;
        if (findRecordByKey(mapSnapshot.rooms, *endpoint_in.key, p_room) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findRecordByKey returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr)
        {
            roomFloorResult_out = AxiomResult::PASS;
            return SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
        }
        std::vector<Finding> scratch;
        if (evaluateOneRoomFloorReciprocity(*p_room,
                                            snapshot_in,
                                            mapSnapshot,
                                            scratch) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateOneRoomFloorReciprocity returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        bool hasFinding{};
        if (anyFindingIs(scratch, AxiomResult::FAIL, hasFinding) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: anyFindingIs returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (hasFinding)
        {
            roomFloorResult_out = AxiomResult::FAIL;
            return SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
        }
        bool hasFinding2{};
        if (anyFindingIs(scratch, AxiomResult::UNKNOWN, hasFinding2) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: anyFindingIs returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (hasFinding2)
        {
            roomFloorResult_out = AxiomResult::UNKNOWN;
            return SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
        }
        roomFloorResult_out = AxiomResult::PASS;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    roomFloorResult_out = AxiomResult::PASS;
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
