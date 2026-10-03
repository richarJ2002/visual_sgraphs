/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
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
 * @file            test_GeometricToolsStatus.cpp
 *
 * @brief           Statuses of GeometricTools::triangulate: a point at a
 *                  finite depth succeeds; a solution at infinity (zero
 *                  homogeneous scale) is a numerical failure that leaves the
 *                  output unchanged.
 */

#include "GeometricTools.h"

#include <gtest/gtest.h>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Checks that GeometricTools::triangulate recovers the 3D
 *                  point seen by two cameras and returns success.
 */
TEST(GeometricToolsStatus, TriangulatesAPointSeenByTwoCameras)
{
    /* Camera 1 at the world origin, camera 2 one metre along +x; both look
     * along +z. Normalised image coordinates of the point (0.5, 0.2, 4) m. */
    Eigen::Matrix<float, 3, 4> Tc1w = Eigen::Matrix<float, 3, 4>::Zero();
    Tc1w.block<3, 3>(0, 0)          = Eigen::Matrix3f::Identity();
    Eigen::Matrix<float, 3, 4> Tc2w = Tc1w;
    Tc2w(0, 3)                      = -1.0F;
    Eigen::Vector3f x_c1(0.125F, 0.05F, 1.0F);
    Eigen::Vector3f x_c2(-0.125F, 0.05F, 1.0F);
    Eigen::Vector3f x3D(0.0F, 0.0F, 0.0F);

    ASSERT_EQ(GeometricTools::triangulate(x_c1, x_c2, Tc1w, Tc2w, x3D),
              GeometricToolsStatus::GEOMETRIC_TOOLS_STATUS_SUCCESS);
    EXPECT_NEAR(x3D(0), 0.5F, 1e-4F);
    EXPECT_NEAR(x3D(1), 0.2F, 1e-4F);
    EXPECT_NEAR(x3D(2), 4.0F, 1e-4F);
}

/*!
 * @brief           Checks that triangulate returns a numerical-failure status,
 *                  and leaves the output point untouched, when the solution
 *                  lies at infinity.
 */
TEST(GeometricToolsStatus, ReportsASolutionAtInfinityAsNumericalFailure)
{
    /* The projections make the linear system diag(1, 1, 0, 1): its null
     * vector is exactly (0, 0, 1, 0), whose homogeneous scale is zero. */
    Eigen::Matrix<float, 3, 4> Tc1w = Eigen::Matrix<float, 3, 4>::Zero();
    Tc1w(0, 0)                      = -1.0F;
    Tc1w(1, 1)                      = -1.0F;
    Eigen::Matrix<float, 3, 4> Tc2w = Eigen::Matrix<float, 3, 4>::Zero();
    Tc2w(1, 3)                      = -1.0F;
    Eigen::Vector3f x_c1(0.3F, -0.2F, 1.0F);
    Eigen::Vector3f x_c2(0.1F, 0.4F, 1.0F);
    Eigen::Vector3f x3D(7.0F, 8.0F, 9.0F);

    EXPECT_EQ(GeometricTools::triangulate(x_c1, x_c2, Tc1w, Tc2w, x3D),
              GeometricToolsStatus::GEOMETRIC_TOOLS_STATUS_NUMERICAL_FAILURE);
    EXPECT_EQ(x3D, Eigen::Vector3f(7.0F, 8.0F, 9.0F));
}

} // namespace core
} // namespace vs_graphs
