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
 * @file            isRawPlaneRefLess.cc
 *
 * @brief           Implements isRawPlaneRefLess(), declared in
 *                  Semantic/ValueOrder.h.
 */

#include "Semantic/ValueOrder.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isRawPlaneRefLess(const RawPlaneRef &lhs_in, const RawPlaneRef &rhs_in)
{
    if (lhs_in.mapId.has_value() != rhs_in.mapId.has_value())
    {
        return lhs_in.mapId.has_value();
    }
    if (lhs_in.mapId.has_value() && *lhs_in.mapId != *rhs_in.mapId)
    {
        return *lhs_in.mapId < *rhs_in.mapId;
    }
    if (lhs_in.planeId != rhs_in.planeId)
    {
        return lhs_in.planeId < rhs_in.planeId;
    }
    if (lhs_in.isLive != rhs_in.isLive)
    {
        return static_cast<int>(lhs_in.isLive) <
               static_cast<int>(rhs_in.isLive);
    }
    if (lhs_in.planeType != rhs_in.planeType)
    {
        return lhs_in.planeType < rhs_in.planeType;
    }
    if (lhs_in.reason != rhs_in.reason)
    {
        return lhs_in.reason < rhs_in.reason;
    }
    if (lhs_in.wallKey.has_value() != rhs_in.wallKey.has_value())
    {
        return lhs_in.wallKey.has_value();
    }
    if (lhs_in.wallKey.has_value() && *lhs_in.wallKey != *rhs_in.wallKey)
    {
        return *lhs_in.wallKey < *rhs_in.wallKey;
    }
    return false;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
