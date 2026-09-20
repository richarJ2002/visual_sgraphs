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
 * @file            wall.cc
 *
 * @brief           Implements the WallRecord overload of
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

bool isValueLessForCollisionTiebreak(const WallRecord &lhs_in,
                                     const WallRecord &rhs_in)
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
    if (lhs_in.planeType != rhs_in.planeType)
    {
        return lhs_in.planeType < rhs_in.planeType;
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
    if (isDoubleLess(lhs_in.minPlaneU_m, rhs_in.minPlaneU_m) ||
        isDoubleLess(rhs_in.minPlaneU_m, lhs_in.minPlaneU_m))
    {
        return isDoubleLess(lhs_in.minPlaneU_m, rhs_in.minPlaneU_m);
    }
    if (isDoubleLess(lhs_in.maxPlaneU_m, rhs_in.maxPlaneU_m) ||
        isDoubleLess(rhs_in.maxPlaneU_m, lhs_in.maxPlaneU_m))
    {
        return isDoubleLess(lhs_in.maxPlaneU_m, rhs_in.maxPlaneU_m);
    }
    if (isDoubleLess(lhs_in.minPlaneV_m, rhs_in.minPlaneV_m) ||
        isDoubleLess(rhs_in.minPlaneV_m, lhs_in.minPlaneV_m))
    {
        return isDoubleLess(lhs_in.minPlaneV_m, rhs_in.minPlaneV_m);
    }
    if (isDoubleLess(lhs_in.maxPlaneV_m, rhs_in.maxPlaneV_m) ||
        isDoubleLess(rhs_in.maxPlaneV_m, lhs_in.maxPlaneV_m))
    {
        return isDoubleLess(lhs_in.maxPlaneV_m, rhs_in.maxPlaneV_m);
    }
    if (lhs_in.finiteSupportCount != rhs_in.finiteSupportCount)
    {
        return lhs_in.finiteSupportCount < rhs_in.finiteSupportCount;
    }
    if (lhs_in.observationCount != rhs_in.observationCount)
    {
        return lhs_in.observationCount < rhs_in.observationCount;
    }
    if (lhs_in.cloudGeneration != rhs_in.cloudGeneration)
    {
        return lhs_in.cloudGeneration < rhs_in.cloudGeneration;
    }
    if (lhs_in.successfulRefitGeneration != rhs_in.successfulRefitGeneration)
    {
        return lhs_in.successfulRefitGeneration <
               rhs_in.successfulRefitGeneration;
    }
    if (lhs_in.observationOrigin_World_m.has_value() !=
        rhs_in.observationOrigin_World_m.has_value())
    {
        return lhs_in.observationOrigin_World_m.has_value();
    }
    if (lhs_in.observationOrigin_World_m.has_value() &&
        (isVector3dLess(*lhs_in.observationOrigin_World_m,
                        *rhs_in.observationOrigin_World_m) ||
         isVector3dLess(*rhs_in.observationOrigin_World_m,
                        *lhs_in.observationOrigin_World_m)))
    {
        return isVector3dLess(*lhs_in.observationOrigin_World_m,
                              *rhs_in.observationOrigin_World_m);
    }
    if (lhs_in.observationSideConsensusReason !=
        rhs_in.observationSideConsensusReason)
    {
        return lhs_in.observationSideConsensusReason <
               rhs_in.observationSideConsensusReason;
    }
    if (isRawPlaneRefLess(lhs_in.twinRef, rhs_in.twinRef) ||
        isRawPlaneRefLess(rhs_in.twinRef, lhs_in.twinRef))
    {
        return isRawPlaneRefLess(lhs_in.twinRef, rhs_in.twinRef);
    }
    if (std::lexicographical_compare(lhs_in.ownerRoomRefs.begin(),
                                     lhs_in.ownerRoomRefs.end(),
                                     rhs_in.ownerRoomRefs.begin(),
                                     rhs_in.ownerRoomRefs.end(),
                                     &isEntityRefLess) ||
        std::lexicographical_compare(rhs_in.ownerRoomRefs.begin(),
                                     rhs_in.ownerRoomRefs.end(),
                                     lhs_in.ownerRoomRefs.begin(),
                                     lhs_in.ownerRoomRefs.end(),
                                     &isEntityRefLess))
    {
        return std::lexicographical_compare(lhs_in.ownerRoomRefs.begin(),
                                            lhs_in.ownerRoomRefs.end(),
                                            rhs_in.ownerRoomRefs.begin(),
                                            rhs_in.ownerRoomRefs.end(),
                                            &isEntityRefLess);
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
