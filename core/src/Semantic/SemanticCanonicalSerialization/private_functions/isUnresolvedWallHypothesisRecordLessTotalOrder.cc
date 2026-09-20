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
 * @file            isUnresolvedWallHypothesisRecordLessTotalOrder.cc
 *
 * @brief           Implements isUnresolvedWallHypothesisRecordLessTotalOrder(),
 *                  declared in private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isUnresolvedWallHypothesisRecordLessTotalOrder(
    const UnresolvedWallHypothesisRecord &lhs_in,
    const UnresolvedWallHypothesisRecord &rhs_in)
{
    /* UnresolvedWallHypothesisRecord (unresolvedCycles, cloudPointCount,
     * observationCount) carries no geometric field, so one total order
     * already serves both the topology-only and full-geometry projections
     * identically -- unlike OpenPassageHypothesisRecord, no distinct
     * *TopologyOnly()/*FullGeometry() split is needed here. */
    if (isRawPlaneRefLess(lhs_in.wallRef, rhs_in.wallRef))
    {
        return true;
    }
    if (isRawPlaneRefLess(rhs_in.wallRef, lhs_in.wallRef))
    {
        return false;
    }

    if (lhs_in.unresolvedCycles != rhs_in.unresolvedCycles)
    {
        return lhs_in.unresolvedCycles < rhs_in.unresolvedCycles;
    }
    if (lhs_in.cloudPointCount != rhs_in.cloudPointCount)
    {
        return lhs_in.cloudPointCount < rhs_in.cloudPointCount;
    }
    if (lhs_in.observationCount != rhs_in.observationCount)
    {
        return lhs_in.observationCount < rhs_in.observationCount;
    }

    return false;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
