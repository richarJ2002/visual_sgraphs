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
 * @brief           Implements the KannalaBrandt8::project() overload
 *                  taking an Eigen::Vector3d, declared in
 *                  CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
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
} // namespace vs_graphs::core::camera_models::kannalabrandt8
