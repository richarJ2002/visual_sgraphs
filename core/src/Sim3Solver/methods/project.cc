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
 * @file            project.cc
 *
 * @brief           Implements Sim3Solver::project(), declared in Sim3Solver.h.
 */

#include "Sim3Solver.h"

#include <cmath>
#include <opencv2/core/core.hpp>
#include <vector>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace vs_graphs
{
namespace core
{

Sim3SolverStatus Sim3Solver::project(
    const std::vector<Eigen::Vector3f>              &vP3Dw_in,
    std::vector<Eigen::Vector2f>                    &points2D_out,
    Eigen::Matrix4f                                  poseWorldToCamera_in,
    camera_models::geometriccamera::GeometricCamera *p_camera_inout)
{
    Eigen::Matrix3f rotationWorldToCamera =
        poseWorldToCamera_in.block<3, 3>(0, 0);
    Eigen::Vector3f translationWorldToCamera =
        poseWorldToCamera_in.block<3, 1>(0, 3);

    points2D_out.clear();
    points2D_out.reserve(vP3Dw_in.size());

    for (size_t pointIndex = 0, iend = vP3Dw_in.size(); pointIndex < iend;
         pointIndex++)
    {
        Eigen::Vector3f P3Dc = rotationWorldToCamera * vP3Dw_in[pointIndex] +
                               translationWorldToCamera;
        Eigen::Vector2f point2d = p_camera_inout->project(P3Dc);
        points2D_out.push_back(point2d);
    }

    return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
