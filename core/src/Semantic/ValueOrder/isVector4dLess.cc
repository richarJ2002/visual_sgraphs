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
 * @file            isVector4dLess.cc
 *
 * @brief           Implements isVector4dLess(), declared in
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

bool isVector4dLess(const Eigen::Vector4d &lhs_in,
                    const Eigen::Vector4d &rhs_in)
{
    /* Compare pre-mapped total-order keys, most-significant component
     * first (Eigen's raw coefficient order (0,1,2,3)), via std::tie -- see
     * isVector3dLess() for why a plain componentwise `<` chain is unsafe. */
    const std::uint64_t lhs0 = doubleTotalOrderKey(lhs_in[0]);
    const std::uint64_t lhs1 = doubleTotalOrderKey(lhs_in[1]);
    const std::uint64_t lhs2 = doubleTotalOrderKey(lhs_in[2]);
    const std::uint64_t lhs3 = doubleTotalOrderKey(lhs_in[3]);
    const std::uint64_t rhs0 = doubleTotalOrderKey(rhs_in[0]);
    const std::uint64_t rhs1 = doubleTotalOrderKey(rhs_in[1]);
    const std::uint64_t rhs2 = doubleTotalOrderKey(rhs_in[2]);
    const std::uint64_t rhs3 = doubleTotalOrderKey(rhs_in[3]);
    return std::tie(lhs0, lhs1, lhs2, lhs3) < std::tie(rhs0, rhs1, rhs2, rhs3);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
