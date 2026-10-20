/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            project.cc
 *
 * @brief           Implements all three Pinhole::project() overloads.
 */

#include "sensor_models/camera_models/pinhole/objects/PinholeClass.h"

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace pinhole
{

cv::Point2f Pinhole::project(const cv::Point3f &point3d_in)
{
    return cv::Point2f(
        parameters[0] * point3d_in.x / point3d_in.z + parameters[2],
        parameters[1] * point3d_in.y / point3d_in.z + parameters[3]);
}

Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &point3d_in)
{
    Eigen::Vector2d projectedPoint;
    projectedPoint[0] =
        parameters[0] * point3d_in[0] / point3d_in[2] + parameters[2];
    projectedPoint[1] =
        parameters[1] * point3d_in[1] / point3d_in[2] + parameters[3];

    return projectedPoint;
}

Eigen::Vector2f Pinhole::project(const Eigen::Vector3f &point3d_in)
{
    Eigen::Vector2f projectedPoint;
    projectedPoint[0] =
        parameters[0] * point3d_in[0] / point3d_in[2] + parameters[2];
    projectedPoint[1] =
        parameters[1] * point3d_in[1] / point3d_in[2] + parameters[3];

    return projectedPoint;
}

} /* namespace pinhole */
} /* namespace camera_models */
} /* namespace core */
} /* namespace vs_graphs */
