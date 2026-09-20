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
 * @file            isPassageRecordLessTopologyOnly.cc
 *
 * @brief           Implements isPassageRecordLessTopologyOnly(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <algorithm>
#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isPassageRecordLessTopologyOnly(const PassageRecord &lhs_in,
                                     const PassageRecord &rhs_in)
{
    if (lhs_in.key != rhs_in.key)
    {
        return lhs_in.key < rhs_in.key;
    }
    if (lhs_in.isLive != rhs_in.isLive)
    {
        return static_cast<int>(lhs_in.isLive) <
               static_cast<int>(rhs_in.isLive);
    }
    if (lhs_in.declaredMapId.has_value() != rhs_in.declaredMapId.has_value())
    {
        return lhs_in.declaredMapId.has_value();
    }
    if (lhs_in.declaredMapId.has_value() &&
        *lhs_in.declaredMapId != *rhs_in.declaredMapId)
    {
        return *lhs_in.declaredMapId < *rhs_in.declaredMapId;
    }
    if (lhs_in.passageType != rhs_in.passageType)
    {
        return static_cast<int>(lhs_in.passageType) <
               static_cast<int>(rhs_in.passageType);
    }
    if (lhs_in.passable != rhs_in.passable)
    {
        return static_cast<int>(lhs_in.passable) <
               static_cast<int>(rhs_in.passable);
    }

    std::vector<RawPlaneRef> lhsWallRefs = lhs_in.associateWallRefs;
    std::vector<RawPlaneRef> rhsWallRefs = rhs_in.associateWallRefs;
    std::sort(lhsWallRefs.begin(), lhsWallRefs.end(), &isRawPlaneRefLess);
    std::sort(rhsWallRefs.begin(), rhsWallRefs.end(), &isRawPlaneRefLess);
    if (lhsWallRefs.size() != rhsWallRefs.size())
    {
        return lhsWallRefs.size() < rhsWallRefs.size();
    }
    for (std::size_t i = 0U; i < lhsWallRefs.size(); ++i)
    {
        if (isRawPlaneRefLess(lhsWallRefs[i], rhsWallRefs[i]))
        {
            return true;
        }
        if (isRawPlaneRefLess(rhsWallRefs[i], lhsWallRefs[i]))
        {
            return false;
        }
    }

    if (isRawPlaneRefLess(lhs_in.associateDoorRef, rhs_in.associateDoorRef))
    {
        return true;
    }
    if (isRawPlaneRefLess(rhs_in.associateDoorRef, lhs_in.associateDoorRef))
    {
        return false;
    }
    if (isEntityRefLess(lhs_in.knownSideRoomRef, rhs_in.knownSideRoomRef))
    {
        return true;
    }
    if (isEntityRefLess(rhs_in.knownSideRoomRef, lhs_in.knownSideRoomRef))
    {
        return false;
    }
    if (isEntityRefLess(lhs_in.prospectiveRoomRef, rhs_in.prospectiveRoomRef))
    {
        return true;
    }
    if (isEntityRefLess(rhs_in.prospectiveRoomRef, lhs_in.prospectiveRoomRef))
    {
        return false;
    }
    if (lhs_in.traversalKnownToFarCount != rhs_in.traversalKnownToFarCount)
    {
        return lhs_in.traversalKnownToFarCount <
               rhs_in.traversalKnownToFarCount;
    }
    if (lhs_in.traversalFarToKnownCount != rhs_in.traversalFarToKnownCount)
    {
        return lhs_in.traversalFarToKnownCount <
               rhs_in.traversalFarToKnownCount;
    }
    if (lhs_in.traversalUnknownCount != rhs_in.traversalUnknownCount)
    {
        return lhs_in.traversalUnknownCount < rhs_in.traversalUnknownCount;
    }
    return lhs_in.endpointSlotReason < rhs_in.endpointSlotReason;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
