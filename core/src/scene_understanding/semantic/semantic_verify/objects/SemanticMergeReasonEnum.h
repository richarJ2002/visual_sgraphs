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
    /*! The anchor rooms agree and nothing contradicts the pairing. */
    ALIGNED,
    /*! A map is missing or the transform is not a finite similarity. */
    INVALID_INPUT,
    /*! An anchor room has no floor plane yet: no answer. */
    FLOOR_EVIDENCE_MISSING,
    /*! The anchor rooms' floors are not coplanar under the transform. */
    FLOOR_CONTRADICTION,
    /*! The maps are not linked in the map chain: no anchor pair. */
    SHARED_ROOM_IDENTITY_MISSING,
    /*! Anchor walls overlap at a different offset and no wall agrees. */
    WALL_ALIGNMENT_CONTRADICTION,
    /*! One wall, room or passage pairs with two counterparts. */
    PAIRING_CONTRADICTION
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_SEMANTIC_MERGE_REASON_ENUM_H \
        */
