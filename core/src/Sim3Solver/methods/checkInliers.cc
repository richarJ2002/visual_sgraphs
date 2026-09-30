/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

/*!
 * @file            checkInliers.cc
 *
 * @brief           Implements Sim3Solver::checkInliers(), declared in
 *                  Sim3Solver.h.
 */

#include "Sim3Solver.h"

#include <cmath>
#include <opencv2/core/core.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace vs_graphs
{
namespace core
{

Sim3SolverStatus Sim3Solver::checkInliers()
{
    std::vector<Eigen::Vector2f> vP1im2, vP2im1;
    if (project(points3Dc2, vP2im1, mT12i, p_firstCamera) !=
        Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: project returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (project(points3Dc1, vP1im2, mT21i, p_secondCamera) !=
        Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: project returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    inlierCount = 0;

    for (size_t points1im1Index = 0; points1im1Index < points1im1.size();
         points1im1Index++)
    {
        Eigen::Vector2f distance1 =
            points1im1[points1im1Index] - vP2im1[points1im1Index];
        Eigen::Vector2f distance2 =
            vP1im2[points1im1Index] - points2im2[points1im1Index];

        const float error1 = distance1.dot(distance1);
        const float error2 = distance2.dot(distance2);

        if (error1 < maxError1[points1im1Index] &&
            error2 < maxError2[points1im1Index])
        {
            inlierFlags[points1im1Index] = true;
            inlierCount++;
        }
        else
            inlierFlags[points1im1Index] = false;
    }

    return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
