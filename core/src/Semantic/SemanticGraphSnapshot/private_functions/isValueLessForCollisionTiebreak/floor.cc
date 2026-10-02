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
 * @file            floor.cc
 *
 * @brief           Implements the FloorRecord overload of
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

bool isValueLessForCollisionTiebreak(const FloorRecord &lhs_in,
                                     const FloorRecord &rhs_in)
{
    if (lhs_in.declaredMapId.has_value() != rhs_in.declaredMapId.has_value())
    {
        return lhs_in.declaredMapId.has_value();
    }
    if (lhs_in.declaredMapId.has_value() &&
        *lhs_in.declaredMapId != *rhs_in.declaredMapId)
    {
        return *lhs_in.declaredMapId < *rhs_in.declaredMapId;
    }
    if (isVector3dLess(lhs_in.floorCentroid_world_m,
                       rhs_in.floorCentroid_world_m) ||
        isVector3dLess(rhs_in.floorCentroid_world_m,
                       lhs_in.floorCentroid_world_m))
    {
        return isVector3dLess(lhs_in.floorCentroid_world_m,
                              rhs_in.floorCentroid_world_m);
    }
    if (lhs_in.planeIdentity.has_value() != rhs_in.planeIdentity.has_value())
    {
        return lhs_in.planeIdentity.has_value();
    }
    if (lhs_in.planeIdentity.has_value())
    {
        const Floor::PlaneIdentity &lhsIdentity = *lhs_in.planeIdentity;
        const Floor::PlaneIdentity &rhsIdentity = *rhs_in.planeIdentity;
        if (isVector4dLess(lhsIdentity.planeEquation_world,
                           rhsIdentity.planeEquation_world) ||
            isVector4dLess(rhsIdentity.planeEquation_world,
                           lhsIdentity.planeEquation_world))
        {
            return isVector4dLess(lhsIdentity.planeEquation_world,
                                  rhsIdentity.planeEquation_world);
        }
        if (lhsIdentity.finiteSupportCount != rhsIdentity.finiteSupportCount)
        {
            return lhsIdentity.finiteSupportCount <
                   rhsIdentity.finiteSupportCount;
        }
        if (lhsIdentity.observationCount != rhsIdentity.observationCount)
        {
            return lhsIdentity.observationCount < rhsIdentity.observationCount;
        }
    }
    return std::lexicographical_compare(lhs_in.roomRefs.begin(),
                                        lhs_in.roomRefs.end(),
                                        rhs_in.roomRefs.begin(),
                                        rhs_in.roomRefs.end(),
                                        &isEntityRefLess);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
