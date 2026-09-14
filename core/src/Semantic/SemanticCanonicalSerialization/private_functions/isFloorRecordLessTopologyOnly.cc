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
 * @file            isFloorRecordLessTopologyOnly.cc
 *
 * @brief           Implements isFloorRecordLessTopologyOnly(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <algorithm>
#include <cstddef>

namespace ORB_SLAM3
{
namespace semantic
{

bool isFloorRecordLessTopologyOnly(const FloorRecord &lhs_in,
                                   const FloorRecord &rhs_in)
{
    if (lhs_in.key != rhs_in.key)
    {
        return lhs_in.key < rhs_in.key;
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

    std::vector<EntityRef> lhsRoomRefs = lhs_in.roomRefs;
    std::vector<EntityRef> rhsRoomRefs = rhs_in.roomRefs;
    std::sort(lhsRoomRefs.begin(), lhsRoomRefs.end(), &isEntityRefLess);
    std::sort(rhsRoomRefs.begin(), rhsRoomRefs.end(), &isEntityRefLess);
    if (lhsRoomRefs.size() != rhsRoomRefs.size())
    {
        return lhsRoomRefs.size() < rhsRoomRefs.size();
    }
    for (std::size_t i = 0U; i < lhsRoomRefs.size(); ++i)
    {
        if (isEntityRefLess(lhsRoomRefs[i], rhsRoomRefs[i]))
        {
            return true;
        }
        if (isEntityRefLess(rhsRoomRefs[i], lhsRoomRefs[i]))
        {
            return false;
        }
    }

    return false;
}

} // namespace semantic
} // namespace ORB_SLAM3
