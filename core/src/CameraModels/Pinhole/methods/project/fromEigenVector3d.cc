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
 * @file            fromEigenVector3d.cc
 *
 * @brief           Implements the Pinhole::project() overload taking an
 *                  Eigen::Vector3d, declared in
 *                  CameraModels/Pinhole/objects/Pinhole.h.
 */

#include "CameraModels/Pinhole/objects/Pinhole.h"

#include <Eigen/Geometry>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace pinhole
{
Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &point3d_in)
{
    Eigen::Vector2d projectedPoint;
    projectedPoint[0] =
        parameters[0] * point3d_in[0] / point3d_in[2] + parameters[2];
    projectedPoint[1] =
        parameters[1] * point3d_in[1] / point3d_in[2] + parameters[3];

    return projectedPoint;
}
} // namespace pinhole
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
