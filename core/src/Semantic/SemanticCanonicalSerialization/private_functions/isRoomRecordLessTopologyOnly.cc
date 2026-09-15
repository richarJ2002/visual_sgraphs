/**
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
 * @file            isRoomRecordLessTopologyOnly.cc
 *
 * @brief           Implements isRoomRecordLessTopologyOnly(), declared in
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

bool isRoomRecordLessTopologyOnly(const RoomRecord &lhs_in,
                                  const RoomRecord &rhs_in)
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
    if (lhs_in.isDetectedMember != rhs_in.isDetectedMember)
    {
        return static_cast<int>(lhs_in.isDetectedMember) <
               static_cast<int>(rhs_in.isDetectedMember);
    }
    if (lhs_in.isMarkerBasedMember != rhs_in.isMarkerBasedMember)
    {
        return static_cast<int>(lhs_in.isMarkerBasedMember) <
               static_cast<int>(rhs_in.isMarkerBasedMember);
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
    if (lhs_in.variant != rhs_in.variant)
    {
        return static_cast<int>(lhs_in.variant) <
               static_cast<int>(rhs_in.variant);
    }
    if (lhs_in.boundaryStatus != rhs_in.boundaryStatus)
    {
        return static_cast<unsigned int>(lhs_in.boundaryStatus) <
               static_cast<unsigned int>(rhs_in.boundaryStatus);
    }

    std::vector<RawPlaneRef> lhsWallRefs = lhs_in.wallRefs;
    std::vector<RawPlaneRef> rhsWallRefs = rhs_in.wallRefs;
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

    std::vector<EntityRef> lhsPassageRefs = lhs_in.passageRefs;
    std::vector<EntityRef> rhsPassageRefs = rhs_in.passageRefs;
    std::sort(lhsPassageRefs.begin(), lhsPassageRefs.end(), &isEntityRefLess);
    std::sort(rhsPassageRefs.begin(), rhsPassageRefs.end(), &isEntityRefLess);
    if (lhsPassageRefs.size() != rhsPassageRefs.size())
    {
        return lhsPassageRefs.size() < rhsPassageRefs.size();
    }
    for (std::size_t i = 0U; i < lhsPassageRefs.size(); ++i)
    {
        if (isEntityRefLess(lhsPassageRefs[i], rhsPassageRefs[i]))
        {
            return true;
        }
        if (isEntityRefLess(rhsPassageRefs[i], lhsPassageRefs[i]))
        {
            return false;
        }
    }

    if (isEntityRefLess(lhs_in.floorRef, rhs_in.floorRef))
    {
        return true;
    }
    if (isEntityRefLess(rhs_in.floorRef, lhs_in.floorRef))
    {
        return false;
    }

    if (isRawPlaneRefLess(lhs_in.groundPlaneRef, rhs_in.groundPlaneRef))
    {
        return true;
    }
    if (isRawPlaneRefLess(rhs_in.groundPlaneRef, lhs_in.groundPlaneRef))
    {
        return false;
    }

    return lhs_in.creationProvenanceReason < rhs_in.creationProvenanceReason;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
