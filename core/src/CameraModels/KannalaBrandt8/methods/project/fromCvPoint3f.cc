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
 * @file            fromCvPoint3f.cc
 *
 * @brief           Implements the KannalaBrandt8::project() overload
 *                  taking a cv::Point3f, declared in
 *                  CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <opencv2/core/core.hpp>

namespace vs_graphs::core::camera_models::kannalabrandt8
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
} // namespace vs_graphs::core::camera_models::kannalabrandt8
