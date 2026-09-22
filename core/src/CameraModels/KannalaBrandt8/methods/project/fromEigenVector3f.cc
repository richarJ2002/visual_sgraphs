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
 * @file            fromEigenVector3f.cc
 *
 * @brief           Implements the KannalaBrandt8::project() overload
 *                  taking an Eigen::Vector3f, declared in
 *                  CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
Eigen::Vector2f KannalaBrandt8::project(const Eigen::Vector3f &point3D_in)
{
    const float x2_plus_y2 =
        point3D_in[0] * point3D_in[0] + point3D_in[1] * point3D_in[1];
    const float theta = atan2f(sqrtf(x2_plus_y2), point3D_in[2]);
    const float psi   = atan2f(point3D_in[1], point3D_in[0]);

    const float theta2 = theta * theta;
    const float theta3 = theta * theta2;
    const float theta5 = theta3 * theta2;
    const float theta7 = theta5 * theta2;
    const float theta9 = theta7 * theta2;
    const float r = theta + parameters[4] * theta3 + parameters[5] * theta5 +
                    parameters[6] * theta7 + parameters[7] * theta9;

    Eigen::Vector2f res;
    res[0] = parameters[0] * r * cos(psi) + parameters[2];
    res[1] = parameters[1] * r * sin(psi) + parameters[3];

    return res;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
