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
cv::Point3f KannalaBrandt8::unproject(const cv::Point2f &point2D_in)
{
    // Use Newthon method to solve for theta with good precision (err ~ e-6)
    cv::Point2f pw((point2D_in.x - parameters[2]) / parameters[0],
                   (point2D_in.y - parameters[3]) / parameters[1]);
    float       scale   = 1.f;
    float       theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);
    theta_d             = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);

    if (theta_d > 1e-8)
    {
        // Compensate distortion iteratively
        float theta = theta_d;

        for (int j = 0; j < 10; j++)
        {
            float theta2 = theta * theta, theta4 = theta2 * theta2,
                  theta6 = theta4 * theta2, theta8 = theta4 * theta4;
            float k0_theta2 = parameters[4] * theta2,
                  k1_theta4 = parameters[5] * theta4;
            float k2_theta6 = parameters[6] * theta6,
                  k3_theta8 = parameters[7] * theta8;
            float theta_fix =
                (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) -
                 theta_d) /
                (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 +
                 9 * k3_theta8);
            theta = theta - theta_fix;
            if (fabsf(theta_fix) < precision)
                break;
        }
        scale = std::tan(theta) / theta_d;
    }

    return cv::Point3f(pw.x * scale, pw.y * scale, 1.f);
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
