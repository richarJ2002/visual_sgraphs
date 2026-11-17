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
 * @brief           Declares the helpers of the plane check stage.
 *                  Package-internal: not included by any public header.
 */

#ifndef VS_GRAPHS_CORE_SRC_PLANE_CHECKS_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_PLANE_CHECKS_PRIVATE_FUNCTIONS_H

#include <Eigen/Geometry>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Computes Eigen's rotation between two directions in its
 *                  own translation unit.
 *
 * @param[in]       sourceDirection_in
 *                  Borrowed source direction; not modified or retained.
 *
 * @param[in]       targetDirection_in
 *                  Borrowed target direction; not modified or retained.
 *
 * @return          Double-precision quaternion from setFromTwoVectors.
 *
 * @note            Delegates unchanged to Eigen, including its normalization,
 *                  opposite-direction branch and degenerate/nonfinite behavior.
 *                  No additional validation or shared mutable state is added.
 *
 * @frame           Active source-to-target rotation, applied on the left of
 *                  column vectors; the caller supplies the frame conventions.
 *
 * @units           Dimensionless directions and quaternion coefficients.
 */
Eigen::Quaterniond
    computeRotationBetweenDirections(const Eigen::Vector3d &sourceDirection_in,
                                     const Eigen::Vector3d &targetDirection_in);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_PLANE_CHECKS_PRIVATE_FUNCTIONS_H */
