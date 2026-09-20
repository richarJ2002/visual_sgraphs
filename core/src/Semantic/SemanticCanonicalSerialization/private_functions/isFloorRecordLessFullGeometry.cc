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
 * @file            isFloorRecordLessFullGeometry.cc
 *
 * @brief           Implements isFloorRecordLessFullGeometry(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isFloorRecordLessFullGeometry(const FloorRecord &lhs_in,
                                   const FloorRecord &rhs_in)
{
    if (isFloorRecordLessTopologyOnly(lhs_in, rhs_in))
    {
        return true;
    }
    if (isFloorRecordLessTopologyOnly(rhs_in, lhs_in))
    {
        return false;
    }

    if (isVector3dLess(lhs_in.centroid_World_m, rhs_in.centroid_World_m))
    {
        return true;
    }
    if (isVector3dLess(rhs_in.centroid_World_m, lhs_in.centroid_World_m))
    {
        return false;
    }

    if (lhs_in.planeIdentity.has_value() != rhs_in.planeIdentity.has_value())
    {
        return lhs_in.planeIdentity.has_value();
    }
    if (lhs_in.planeIdentity.has_value())
    {
        const Floor::PlaneIdentity &lhsPlane = *lhs_in.planeIdentity;
        const Floor::PlaneIdentity &rhsPlane = *rhs_in.planeIdentity;
        if (isVector4dLess(lhsPlane.equation_World, rhsPlane.equation_World))
        {
            return true;
        }
        if (isVector4dLess(rhsPlane.equation_World, lhsPlane.equation_World))
        {
            return false;
        }
        if (lhsPlane.finiteSupportCount != rhsPlane.finiteSupportCount)
        {
            return lhsPlane.finiteSupportCount < rhsPlane.finiteSupportCount;
        }
        if (lhsPlane.observationCount != rhsPlane.observationCount)
        {
            return lhsPlane.observationCount < rhsPlane.observationCount;
        }
    }

    return false;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
