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
 * @file            isEntityRefLess.cc
 *
 * @brief           Implements isEntityRefLess(), declared in
 *                  Semantic/ValueOrder.h.
 */

#include "Semantic/ValueOrder.h"

namespace ORB_SLAM3
{
namespace semantic
{

bool isEntityRefLess(const EntityRef &lhs_in, const EntityRef &rhs_in)
{
    if (lhs_in.key.has_value() != rhs_in.key.has_value())
    {
        return lhs_in.key.has_value();
    }
    if (lhs_in.key.has_value() && *lhs_in.key != *rhs_in.key)
    {
        return *lhs_in.key < *rhs_in.key;
    }
    if (lhs_in.reason != rhs_in.reason)
    {
        return lhs_in.reason < rhs_in.reason;
    }
    if (lhs_in.localId.has_value() != rhs_in.localId.has_value())
    {
        return lhs_in.localId.has_value();
    }
    if (lhs_in.localId.has_value() && *lhs_in.localId != *rhs_in.localId)
    {
        return *lhs_in.localId < *rhs_in.localId;
    }
    if (lhs_in.isLive.has_value() != rhs_in.isLive.has_value())
    {
        return lhs_in.isLive.has_value();
    }
    if (lhs_in.isLive.has_value() && *lhs_in.isLive != *rhs_in.isLive)
    {
        return static_cast<int>(*lhs_in.isLive) <
               static_cast<int>(*rhs_in.isLive);
    }
    return lhs_in.livenessUnavailableReason < rhs_in.livenessUnavailableReason;
}

} // namespace semantic
} // namespace ORB_SLAM3
