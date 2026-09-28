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

#include "SemanticsManager.h"

#include "Semantic/SemanticGraphSnapshot.h"
#include "Semantic/ValueOrder.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{

std::vector<semantic::UnresolvedWallHypothesisRecord>
    SemanticsManager::captureUnresolvedWallHypotheses(void) const
{
    std::vector<semantic::UnresolvedWallHypothesisRecord> records;
    records.reserve(undefendedWalls.size());
    for (const auto &[wallId, state] : undefendedWalls)
    {
        semantic::UnresolvedWallHypothesisRecord record;
        record.wallRef          = semantic::rawPlaneRef(state.p_wall);
        record.unresolvedCycles = state.unresolvedCycles;
        record.cloudPointCount  = state.cloudPointCount;
        record.observationCount = state.observationCount;
        records.push_back(record);
    }
    std::sort(
        records.begin(),
        records.end(),
        [](const semantic::UnresolvedWallHypothesisRecord &lhs_in,
           const semantic::UnresolvedWallHypothesisRecord &rhs_in)
        {
            if (semantic::isRawPlaneRefLess(lhs_in.wallRef, rhs_in.wallRef))
                return true;
            if (semantic::isRawPlaneRefLess(rhs_in.wallRef, lhs_in.wallRef))
                return false;
            return lhs_in.unresolvedCycles < rhs_in.unresolvedCycles;
        });
    return records;
}

} // namespace core
} // namespace vs_graphs
