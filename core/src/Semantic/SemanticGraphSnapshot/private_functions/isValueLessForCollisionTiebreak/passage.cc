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
 * @file            passage.cc
 *
 * @brief           Implements the PassageRecord overload of
 *                  isValueLessForCollisionTiebreak(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isValueLessForCollisionTiebreak(const PassageRecord &lhs_in,
                                     const PassageRecord &rhs_in)
{
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
        return lhs_in.passageType < rhs_in.passageType;
    }
    if (isVector4dLess(lhs_in.equation_World, rhs_in.equation_World) ||
        isVector4dLess(rhs_in.equation_World, lhs_in.equation_World))
    {
        return isVector4dLess(lhs_in.equation_World, rhs_in.equation_World);
    }
    if (isVector3dLess(lhs_in.centroid_World_m, rhs_in.centroid_World_m) ||
        isVector3dLess(rhs_in.centroid_World_m, lhs_in.centroid_World_m))
    {
        return isVector3dLess(lhs_in.centroid_World_m, rhs_in.centroid_World_m);
    }
    if (isDoubleLess(lhs_in.width_m, rhs_in.width_m) ||
        isDoubleLess(rhs_in.width_m, lhs_in.width_m))
    {
        return isDoubleLess(lhs_in.width_m, rhs_in.width_m);
    }
    if (isDoubleLess(lhs_in.height_m, rhs_in.height_m) ||
        isDoubleLess(rhs_in.height_m, lhs_in.height_m))
    {
        return isDoubleLess(lhs_in.height_m, rhs_in.height_m);
    }
    if (lhs_in.passable != rhs_in.passable)
    {
        return static_cast<int>(lhs_in.passable) <
               static_cast<int>(rhs_in.passable);
    }
    if (std::lexicographical_compare(lhs_in.associateWallRefs.begin(),
                                     lhs_in.associateWallRefs.end(),
                                     rhs_in.associateWallRefs.begin(),
                                     rhs_in.associateWallRefs.end(),
                                     &isRawPlaneRefLess) ||
        std::lexicographical_compare(rhs_in.associateWallRefs.begin(),
                                     rhs_in.associateWallRefs.end(),
                                     lhs_in.associateWallRefs.begin(),
                                     lhs_in.associateWallRefs.end(),
                                     &isRawPlaneRefLess))
    {
        return std::lexicographical_compare(lhs_in.associateWallRefs.begin(),
                                            lhs_in.associateWallRefs.end(),
                                            rhs_in.associateWallRefs.begin(),
                                            rhs_in.associateWallRefs.end(),
                                            &isRawPlaneRefLess);
    }
    if (isRawPlaneRefLess(lhs_in.associateDoorRef, rhs_in.associateDoorRef) ||
        isRawPlaneRefLess(rhs_in.associateDoorRef, lhs_in.associateDoorRef))
    {
        return isRawPlaneRefLess(lhs_in.associateDoorRef,
                                 rhs_in.associateDoorRef);
    }
    if (isEntityRefLess(lhs_in.knownSideRoomRef, rhs_in.knownSideRoomRef) ||
        isEntityRefLess(rhs_in.knownSideRoomRef, lhs_in.knownSideRoomRef))
    {
        return isEntityRefLess(lhs_in.knownSideRoomRef,
                               rhs_in.knownSideRoomRef);
    }
    if (lhs_in.knownSideDirection_World.has_value() !=
        rhs_in.knownSideDirection_World.has_value())
    {
        return lhs_in.knownSideDirection_World.has_value();
    }
    if (lhs_in.knownSideDirection_World.has_value() &&
        (isVector3dLess(*lhs_in.knownSideDirection_World,
                        *rhs_in.knownSideDirection_World) ||
         isVector3dLess(*rhs_in.knownSideDirection_World,
                        *lhs_in.knownSideDirection_World)))
    {
        return isVector3dLess(*lhs_in.knownSideDirection_World,
                              *rhs_in.knownSideDirection_World);
    }
    if (isEntityRefLess(lhs_in.prospectiveRoomRef, rhs_in.prospectiveRoomRef) ||
        isEntityRefLess(rhs_in.prospectiveRoomRef, lhs_in.prospectiveRoomRef))
    {
        return isEntityRefLess(lhs_in.prospectiveRoomRef,
                               rhs_in.prospectiveRoomRef);
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
