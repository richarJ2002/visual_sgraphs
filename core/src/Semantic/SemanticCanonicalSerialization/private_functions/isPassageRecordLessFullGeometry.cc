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
 * @file            isPassageRecordLessFullGeometry.cc
 *
 * @brief           Implements isPassageRecordLessFullGeometry(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

bool isPassageRecordLessFullGeometry(const PassageRecord &lhs_in,
                                     const PassageRecord &rhs_in)
{
    if (isPassageRecordLessTopologyOnly(lhs_in, rhs_in))
    {
        return true;
    }
    if (isPassageRecordLessTopologyOnly(rhs_in, lhs_in))
    {
        return false;
    }

    if (isVector4dLess(lhs_in.equation_World, rhs_in.equation_World))
    {
        return true;
    }
    if (isVector4dLess(rhs_in.equation_World, lhs_in.equation_World))
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
    if (isDoubleLess(lhs_in.width_m, rhs_in.width_m))
    {
        return true;
    }
    if (isDoubleLess(rhs_in.width_m, lhs_in.width_m))
    {
        return false;
    }
    if (isDoubleLess(lhs_in.height_m, rhs_in.height_m))
    {
        return true;
    }
    if (isDoubleLess(rhs_in.height_m, lhs_in.height_m))
    {
        return false;
    }
    if (lhs_in.knownSideDirection_World.has_value() !=
        rhs_in.knownSideDirection_World.has_value())
    {
        return lhs_in.knownSideDirection_World.has_value();
    }
    if (lhs_in.knownSideDirection_World.has_value())
    {
        if (isVector3dLess(*lhs_in.knownSideDirection_World,
                           *rhs_in.knownSideDirection_World))
        {
            return true;
        }
        if (isVector3dLess(*rhs_in.knownSideDirection_World,
                           *lhs_in.knownSideDirection_World))
        {
            return false;
        }
    }

    return false;
}

} // namespace semantic
} // namespace ORB_SLAM3
