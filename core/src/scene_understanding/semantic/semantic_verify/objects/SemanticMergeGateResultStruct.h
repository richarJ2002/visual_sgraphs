/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            SemanticMergeGateResultStruct.h
 *
 * @brief           Declares SemanticMergeGateResult, the complete gate result.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_GATE_RESULT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_GATE_RESULT_STRUCT_H

#include "scene_understanding/semantic/semantic_verify/objects/SemanticMergeDecisionEnum.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticMergeReasonEnum.h"

#include <cstddef>
#include <string>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Complete, reason-coded result of the physical map-merge
 *                  semantic gate.
 */
struct SemanticMergeGateResult
{
    /*!
     * @brief           Outcome of the gate: accept, defer or reject the merge.
     */
    SemanticMergeDecision decision{SemanticMergeDecision::DEFER};
    /*!
     * @brief           Why the gate chose its decision.
     */
    SemanticMergeReason   reason{
        SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING};
    /*!
     * @brief           Floor check outcome text: "ACCEPTED", "REJECTED",
     *                  "DEFERRED" or empty when the floor check did not run.
     */
    std::string floorDecision;
    /*!
     * @brief           Number of anchor room pairs: rooms on both sides of
     *                  the merge that carry the same room tag.
     */
    std::size_t sharedRoomCount{0U};
    /*!
     * @brief           Number of anchor room pairs whose floor and walls
     *                  agree under the proposed transform.
     */
    std::size_t alignedRoomCount{0U};
    /*!
     * @brief           Total number of wall pairs found between paired
     *                  rooms, anchor rooms included.
     */
    std::size_t matchedWallCount{0U};
    /*!
     * @brief           Total number of passage pairs found across the
     *                  overlap of the two maps.
     */
    std::size_t matchedPassageCount{0U};
    /*!
     * @brief           Element pairs and problems behind the decision, as
     *                  space separated key=value fields for the pairs log
     *                  line.
     */
    std::string pairDetail;
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_GATE_RESULT_STRUCT_H \
        */
