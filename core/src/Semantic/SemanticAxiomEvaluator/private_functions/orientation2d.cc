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
 * @file            orientation2d.cc
 *
 * @brief           Implements orientation2d(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

double orientation2d(const Eigen::Vector2d &p_in,
                     const Eigen::Vector2d &q_in,
                     const Eigen::Vector2d &r_in)
{
    return (q_in.x() - p_in.x()) * (r_in.y() - p_in.y()) -
           (q_in.y() - p_in.y()) * (r_in.x() - p_in.x());
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
