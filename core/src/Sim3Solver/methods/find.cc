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
 * @file            find.cc
 *
 * @brief           Implements Sim3Solver::find(), declared in Sim3Solver.h.
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

Sim3SolverStatus Sim3Solver::find(std::vector<bool> &inliers12Flags_inout,
                                  int               &inlierCount_inout,
                                  Eigen::Matrix4f   &transform_out)
{
    bool            areIterationsExhausted;
    Eigen::Matrix4f transform{};
    if (iterate(ransacMaxIterations,
                areIterationsExhausted,
                inliers12Flags_inout,
                inlierCount_inout,
                transform) != Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: iterate returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    transform_out = transform;
    return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
