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
 * @file            computeLevelTilt.cc
 *
 * @brief           Implements the level tilt measure of the plane rules.
 */

#include "scene_understanding/semantic_constraints/plane_rules/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

double computeLevelTilt(const Eigen::Vector3d &planeNormal_common_in,
                        const Eigen::Vector3d &upDirection_common_in)
{
    const double upComponent = planeNormal_common_in.dot(upDirection_common_in);
    return (planeNormal_common_in - upComponent * upDirection_common_in).norm();
}

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */
