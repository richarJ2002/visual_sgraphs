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
 * @file            isWallRecordLessTopologyOnly.cc
 *
 * @brief           Implements isWallRecordLessTopologyOnly(), declared in
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

bool isWallRecordLessTopologyOnly(const WallRecord &lhs_in,
                                  const WallRecord &rhs_in)
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
    if (lhs_in.planeType != rhs_in.planeType)
    {
        return static_cast<int>(lhs_in.planeType) <
               static_cast<int>(rhs_in.planeType);
    }
    if (lhs_in.observationSideConsensusReason !=
        rhs_in.observationSideConsensusReason)
    {
        return lhs_in.observationSideConsensusReason <
               rhs_in.observationSideConsensusReason;
    }
    if (isRawPlaneRefLess(lhs_in.twinRef, rhs_in.twinRef))
    {
        return true;
    }
    if (isRawPlaneRefLess(rhs_in.twinRef, lhs_in.twinRef))
    {
        return false;
    }

    std::vector<EntityRef> lhsOwnerRoomRefs = lhs_in.ownerRoomRefs;
    std::vector<EntityRef> rhsOwnerRoomRefs = rhs_in.ownerRoomRefs;
    std::sort(lhsOwnerRoomRefs.begin(),
              lhsOwnerRoomRefs.end(),
              &isEntityRefLess);
    std::sort(rhsOwnerRoomRefs.begin(),
              rhsOwnerRoomRefs.end(),
              &isEntityRefLess);
    if (lhsOwnerRoomRefs.size() != rhsOwnerRoomRefs.size())
    {
        return lhsOwnerRoomRefs.size() < rhsOwnerRoomRefs.size();
    }
    for (std::size_t lhsOwnerRoomRefIndex = 0U;
         lhsOwnerRoomRefIndex < lhsOwnerRoomRefs.size();
         ++lhsOwnerRoomRefIndex)
    {
        if (isEntityRefLess(lhsOwnerRoomRefs[lhsOwnerRoomRefIndex],
                            rhsOwnerRoomRefs[lhsOwnerRoomRefIndex]))
        {
            return true;
        }
        if (isEntityRefLess(rhsOwnerRoomRefs[lhsOwnerRoomRefIndex],
                            lhsOwnerRoomRefs[lhsOwnerRoomRefIndex]))
        {
            return false;
        }
    }

    if (lhs_in.quarantineReason != rhs_in.quarantineReason)
    {
        return lhs_in.quarantineReason < rhs_in.quarantineReason;
    }
    return lhs_in.observationRayEvidenceReason <
           rhs_in.observationRayEvidenceReason;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
