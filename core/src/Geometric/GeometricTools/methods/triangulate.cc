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

#include "GeometricTools.h"

#include "KeyFrame.h"

namespace vs_graphs
{
namespace core
{

GeometricToolsStatus
    GeometricTools::triangulate(Eigen::Vector3f            &x_c1,
                                Eigen::Vector3f            &x_c2,
                                Eigen::Matrix<float, 3, 4> &Tc1w_in,
                                Eigen::Matrix<float, 3, 4> &Tc2w_in,
                                Eigen::Vector3f            &x3D_inout)
{
    Eigen::Matrix4f A;
    A.block<1, 4>(0, 0) =
        x_c1(0) * Tc1w_in.block<1, 4>(2, 0) - Tc1w_in.block<1, 4>(0, 0);
    A.block<1, 4>(1, 0) =
        x_c1(1) * Tc1w_in.block<1, 4>(2, 0) - Tc1w_in.block<1, 4>(1, 0);
    A.block<1, 4>(2, 0) =
        x_c2(0) * Tc2w_in.block<1, 4>(2, 0) - Tc2w_in.block<1, 4>(0, 0);
    A.block<1, 4>(3, 0) =
        x_c2(1) * Tc2w_in.block<1, 4>(2, 0) - Tc2w_in.block<1, 4>(1, 0);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

    Eigen::Vector4f x3Dh = svd.matrixV().col(3);

    if (x3Dh(3) == 0)
        return GeometricToolsStatus::GEOMETRIC_TOOLS_STATUS_NUMERICAL_FAILURE;

    // Euclidean coordinates
    x3D_inout = x3Dh.head(3) / x3Dh(3);

    return GeometricToolsStatus::GEOMETRIC_TOOLS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
