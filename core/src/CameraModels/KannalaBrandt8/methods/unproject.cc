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
 * @file            unproject.cc
 *
 * @brief           Implements KannalaBrandt8::unproject(), declared in
 *                  CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <cmath>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
cv::Point3f KannalaBrandt8::unproject(const cv::Point2f &point2d_in)
{
    // Use Newthon method to solve for theta with good precision (err ~ e-6)
    cv::Point2f normalizedImagePoint(
        (point2d_in.x - parameters[2]) / parameters[0],
        (point2d_in.y - parameters[3]) / parameters[1]);
    float rayScale = 1.f;
    float distortedIncidenceAngle =
        sqrtf(normalizedImagePoint.x * normalizedImagePoint.x +
              normalizedImagePoint.y * normalizedImagePoint.y);
    distortedIncidenceAngle =
        fminf(fmaxf(-CV_PI / 2.f, distortedIncidenceAngle), CV_PI / 2.f);

    if (distortedIncidenceAngle > 1e-8)
    {
        // Compensate distortion iteratively
        float incidenceAngle = distortedIncidenceAngle;

        for (int iterationIndex = 0; iterationIndex < 10; iterationIndex++)
        {
            float incidenceAngleSquared = incidenceAngle * incidenceAngle,
                  incidenceAnglePow4 =
                      incidenceAngleSquared * incidenceAngleSquared,
                  incidenceAnglePow6 =
                      incidenceAnglePow4 * incidenceAngleSquared,
                  incidenceAnglePow8 = incidenceAnglePow4 * incidenceAnglePow4;
            float distortionTermPow2 = parameters[4] * incidenceAngleSquared,
                  distortionTermPow4 = parameters[5] * incidenceAnglePow4;
            float distortionTermPow6 = parameters[6] * incidenceAnglePow6,
                  distortionTermPow8 = parameters[7] * incidenceAnglePow8;
            float thetaCorrection =
                (incidenceAngle * (1 + distortionTermPow2 + distortionTermPow4 +
                                   distortionTermPow6 + distortionTermPow8) -
                 distortedIncidenceAngle) /
                (1 + 3 * distortionTermPow2 + 5 * distortionTermPow4 +
                 7 * distortionTermPow6 + 9 * distortionTermPow8);
            incidenceAngle = incidenceAngle - thetaCorrection;
            if (fabsf(thetaCorrection) < precision)
                break;
        }
        rayScale = std::tan(incidenceAngle) / distortedIncidenceAngle;
    }

    return cv::Point3f(normalizedImagePoint.x * rayScale,
                       normalizedImagePoint.y * rayScale,
                       1.f);
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
