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
 * @file            isDoubleLess.cc
 *
 * @brief           Implements isDoubleLess(), declared in
 *                  Semantic/ValueOrder.h.
 */

#include "Semantic/ValueOrder.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isDoubleLess(double lhs_in, double rhs_in)
{
    /* Ordinary lhs_in < rhs_in is not a strict weak ordering when either
     * operand may be NaN (every comparison against NaN is false, so two
     * NaN values are simultaneously "not less" in both directions, and a
     * value compared against them is neither less nor greater -- violating
     * transitivity of equivalence). Comparing the total-order keys instead
     * keeps this a genuine total order for every double bit pattern. */
    return doubleTotalOrderKey(lhs_in) < doubleTotalOrderKey(rhs_in);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
