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

#include "SemanticsManager.h"

#include "private_functions.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief Computes the scalar two-dimensional cross product.
 *
 * @param[in] firstVector_in First vector.
 * @param[in] secondVector_in Second vector.
 * @return Signed scalar cross product.
 */
double crossProduct2d(const Eigen::Vector2d &firstVector_in,
                      const Eigen::Vector2d &secondVector_in)
{
    return firstVector_in.x() * secondVector_in.y() -
           firstVector_in.y() * secondVector_in.x();
}

} // namespace core
} // namespace vs_graphs
