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
     * @brief           Number of rooms found on both sides of the merge.
     */
    std::size_t sharedRoomCount{0U};
    /*!
     * @brief           Number of shared rooms whose walls and passages agree
     *                  under the proposed transform.
     */
    std::size_t alignedRoomCount{0U};
    /*!
     * @brief           Total number of wall pairs matched across the shared
     *                  rooms.
     */
    std::size_t matchedWallCount{0U};
    /*!
     * @brief           Total number of passage pairs matched across the shared
     *                  rooms.
     */
    std::size_t matchedPassageCount{0U};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_GATE_RESULT_STRUCT_H \
        */
