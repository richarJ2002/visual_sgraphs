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
 * @file            private_functions.h
 *
 * @brief           Declares the measures the plane rules share.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULES_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULES_PRIVATE_FUNCTIONS_H

#include <Eigen/Core>

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

/*!
 * @brief           Measures how far a plane is from level: the length of the
 *                  normal's part across up (the sine of the angle between the
 *                  normal and the up axis). The normal's sign is irrelevant.
 *
 * @param[in]       planeNormal_common_in
 *                  Unit plane normal, in the frame of upDirection_common_in.
 *
 * @param[in]       upDirection_common_in
 *                  Unit up direction.
 *
 * @return          Tilt in [0, 1]; 0 for a level plane.
 */
[[nodiscard]] double
    computeLevelTilt(const Eigen::Vector3d &planeNormal_common_in,
                     const Eigen::Vector3d &upDirection_common_in);

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULES_PRIVATE_FUNCTIONS_H \
        */
