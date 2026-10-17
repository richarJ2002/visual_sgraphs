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
 * @brief           Implements all three KannalaBrandt8::project() overloads.
 */

#include "camera_models/kannala_brandt8/objects/KannalaBrandt8Class.h"

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace kannala_brandt8
{

cv::Point2f KannalaBrandt8::project(const cv::Point3f &point3d_in)
{
    const float planarRadiusSquared =
        point3d_in.x * point3d_in.x + point3d_in.y * point3d_in.y;
    const float incidenceAngle =
        atan2f(sqrtf(planarRadiusSquared), point3d_in.z);
    const float azimuthAngle = atan2f(point3d_in.y, point3d_in.x);

    const float incidenceAngleSquared = incidenceAngle * incidenceAngle;
    const float incidenceAngleCubed   = incidenceAngle * incidenceAngleSquared;
    const float incidenceAnglePow5 =
        incidenceAngleCubed * incidenceAngleSquared;
    const float incidenceAnglePow7 = incidenceAnglePow5 * incidenceAngleSquared;
    const float incidenceAnglePow9 = incidenceAnglePow7 * incidenceAngleSquared;
    const float distortedIncidenceAngle =
        incidenceAngle + parameters[4] * incidenceAngleCubed +
        parameters[5] * incidenceAnglePow5 +
        parameters[6] * incidenceAnglePow7 + parameters[7] * incidenceAnglePow9;

    return cv::Point2f(
        parameters[0] * distortedIncidenceAngle * cos(azimuthAngle) +
            parameters[2],
        parameters[1] * distortedIncidenceAngle * sin(azimuthAngle) +
            parameters[3]);
}

Eigen::Vector2d KannalaBrandt8::project(const Eigen::Vector3d &point3d_in)
{
    const double planarRadiusSquared =
        point3d_in[0] * point3d_in[0] + point3d_in[1] * point3d_in[1];
    const double incidenceAngle =
        atan2f(sqrtf(planarRadiusSquared), point3d_in[2]);
    const double azimuthAngle = atan2f(point3d_in[1], point3d_in[0]);

    const double incidenceAngleSquared = incidenceAngle * incidenceAngle;
    const double incidenceAngleCubed   = incidenceAngle * incidenceAngleSquared;
    const double incidenceAnglePow5 =
        incidenceAngleCubed * incidenceAngleSquared;
    const double incidenceAnglePow7 =
        incidenceAnglePow5 * incidenceAngleSquared;
    const double incidenceAnglePow9 =
        incidenceAnglePow7 * incidenceAngleSquared;
    const double distortedIncidenceAngle =
        incidenceAngle + parameters[4] * incidenceAngleCubed +
        parameters[5] * incidenceAnglePow5 +
        parameters[6] * incidenceAnglePow7 + parameters[7] * incidenceAnglePow9;

    Eigen::Vector2d projectedPoint;
    projectedPoint[0] =
        parameters[0] * distortedIncidenceAngle * cos(azimuthAngle) +
        parameters[2];
    projectedPoint[1] =
        parameters[1] * distortedIncidenceAngle * sin(azimuthAngle) +
        parameters[3];

    return projectedPoint;
}

Eigen::Vector2f KannalaBrandt8::project(const Eigen::Vector3f &point3d_in)
{
    const float planarRadiusSquared =
        point3d_in[0] * point3d_in[0] + point3d_in[1] * point3d_in[1];
    const float incidenceAngle =
        atan2f(sqrtf(planarRadiusSquared), point3d_in[2]);
    const float azimuthAngle = atan2f(point3d_in[1], point3d_in[0]);

    const float incidenceAngleSquared = incidenceAngle * incidenceAngle;
    const float incidenceAngleCubed   = incidenceAngle * incidenceAngleSquared;
    const float incidenceAnglePow5 =
        incidenceAngleCubed * incidenceAngleSquared;
    const float incidenceAnglePow7 = incidenceAnglePow5 * incidenceAngleSquared;
    const float incidenceAnglePow9 = incidenceAnglePow7 * incidenceAngleSquared;
    const float distortedIncidenceAngle =
        incidenceAngle + parameters[4] * incidenceAngleCubed +
        parameters[5] * incidenceAnglePow5 +
        parameters[6] * incidenceAnglePow7 + parameters[7] * incidenceAnglePow9;

    Eigen::Vector2f projectedPoint;
    projectedPoint[0] =
        parameters[0] * distortedIncidenceAngle * cos(azimuthAngle) +
        parameters[2];
    projectedPoint[1] =
        parameters[1] * distortedIncidenceAngle * sin(azimuthAngle) +
        parameters[3];

    return projectedPoint;
}

} /* namespace kannala_brandt8 */
} /* namespace camera_models */
} /* namespace core */
} /* namespace vs_graphs */
