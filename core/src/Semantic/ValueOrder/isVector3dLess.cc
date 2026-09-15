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
 * @file            isVector3dLess.cc
 *
 * @brief           Implements isVector3dLess(), declared in
 *                  Semantic/ValueOrder.h.
 */

#include "Semantic/ValueOrder.h"

#include <cstdint>
#include <tuple>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isVector3dLess(const Eigen::Vector3d &lhs_in,
                    const Eigen::Vector3d &rhs_in)
{
    /* Compare pre-mapped total-order keys, most-significant component
     * first, via std::tie -- a plain componentwise `<` chain over the raw
     * doubles would not be a valid strict weak ordering whenever a
     * component is NaN. */
    const std::uint64_t lhsX = doubleTotalOrderKey(lhs_in.x());
    const std::uint64_t lhsY = doubleTotalOrderKey(lhs_in.y());
    const std::uint64_t lhsZ = doubleTotalOrderKey(lhs_in.z());
    const std::uint64_t rhsX = doubleTotalOrderKey(rhs_in.x());
    const std::uint64_t rhsY = doubleTotalOrderKey(rhs_in.y());
    const std::uint64_t rhsZ = doubleTotalOrderKey(rhs_in.z());
    return std::tie(lhsX, lhsY, lhsZ) < std::tie(rhsX, rhsY, rhsZ);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
