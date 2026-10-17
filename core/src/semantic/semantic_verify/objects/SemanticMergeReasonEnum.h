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
 * @file            SemanticMergeReasonEnum.h
 *
 * @brief           Declares SemanticMergeReason, the typed reason for a
 * decision.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_REASON_ENUM_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_REASON_ENUM_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Typed reason for a semantic merge-gate decision.
 */
enum class SemanticMergeReason
{
    ALIGNED,
    INVALID_INPUT,
    FLOOR_EVIDENCE_MISSING,
    FLOOR_CONTRADICTION,
    SHARED_ROOM_IDENTITY_MISSING,
    WALL_EVIDENCE_MISSING,
    WALL_ALIGNMENT_CONTRADICTION,
    PASSAGE_EVIDENCE_MISSING,
    PASSAGE_IDENTITY_CONTRADICTION,
    PASSAGE_ENDPOINT_CONTRADICTION,
    PASSAGE_DIRECTION_CONTRADICTION
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_REASON_ENUM_H \
        */
