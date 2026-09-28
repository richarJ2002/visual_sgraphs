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
 * @file            newellNormal.cc
 *
 * @brief           Implements newellNormal(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    newellNormal(const std::vector<Eigen::Vector3d> &corners_in,
                 Eigen::Vector3d                    &normal_out)
{
    Eigen::Vector3d   normal      = Eigen::Vector3d::Zero();
    const std::size_t cornerCount = corners_in.size();
    for (std::size_t cornerIndex = 0U; cornerIndex < cornerCount; ++cornerIndex)
    {
        const Eigen::Vector3d &current = corners_in[cornerIndex];
        const Eigen::Vector3d &next =
            corners_in[(cornerIndex + 1U) % cornerCount];
        normal.x() += (current.y() - next.y()) * (current.z() + next.z());
        normal.y() += (current.z() - next.z()) * (current.x() + next.x());
        normal.z() += (current.x() - next.x()) * (current.y() + next.y());
    }
    normal_out = normal;
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
