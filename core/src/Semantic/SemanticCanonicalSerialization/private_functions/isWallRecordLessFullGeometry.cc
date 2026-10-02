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
 * @file            isWallRecordLessFullGeometry.cc
 *
 * @brief           Implements isWallRecordLessFullGeometry(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isWallRecordLessFullGeometry(const WallRecord &lhs_in,
                                  const WallRecord &rhs_in)
{
    if (isWallRecordLessTopologyOnly(lhs_in, rhs_in))
    {
        return true;
    }
    if (isWallRecordLessTopologyOnly(rhs_in, lhs_in))
    {
        return false;
    }

    if (isVector4dLess(lhs_in.planeEquation_world, rhs_in.planeEquation_world))
    {
        return true;
    }
    if (isVector4dLess(rhs_in.planeEquation_world, lhs_in.planeEquation_world))
    {
        return false;
    }
    if (isVector3dLess(lhs_in.planeCentroid_world_m,
                       rhs_in.planeCentroid_world_m))
    {
        return true;
    }
    if (isVector3dLess(rhs_in.planeCentroid_world_m,
                       lhs_in.planeCentroid_world_m))
    {
        return false;
    }
    if (isDoubleLess(lhs_in.minPlaneU_m, rhs_in.minPlaneU_m))
    {
        return true;
    }
    if (isDoubleLess(rhs_in.minPlaneU_m, lhs_in.minPlaneU_m))
    {
        return false;
    }
    if (isDoubleLess(lhs_in.maxPlaneU_m, rhs_in.maxPlaneU_m))
    {
        return true;
    }
    if (isDoubleLess(rhs_in.maxPlaneU_m, lhs_in.maxPlaneU_m))
    {
        return false;
    }
    if (isDoubleLess(lhs_in.minPlaneV_m, rhs_in.minPlaneV_m))
    {
        return true;
    }
    if (isDoubleLess(rhs_in.minPlaneV_m, lhs_in.minPlaneV_m))
    {
        return false;
    }
    if (isDoubleLess(lhs_in.maxPlaneV_m, rhs_in.maxPlaneV_m))
    {
        return true;
    }
    if (isDoubleLess(rhs_in.maxPlaneV_m, lhs_in.maxPlaneV_m))
    {
        return false;
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
    if (lhs_in.observationOrigin_world_m.has_value() !=
        rhs_in.observationOrigin_world_m.has_value())
    {
        return lhs_in.observationOrigin_world_m.has_value();
    }
    if (lhs_in.observationOrigin_world_m.has_value())
    {
        if (isVector3dLess(*lhs_in.observationOrigin_world_m,
                           *rhs_in.observationOrigin_world_m))
        {
            return true;
        }
        if (isVector3dLess(*rhs_in.observationOrigin_world_m,
                           *lhs_in.observationOrigin_world_m))
        {
            return false;
        }
    }

    return false;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
