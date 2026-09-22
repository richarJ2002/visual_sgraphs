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
 * @file            computeProjectionJacobian.cc
 *
 * @brief           Implements
 *                  KannalaBrandt8::computeProjectionJacobian(), declared
 *                  in CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
Eigen::Matrix<double, 2, 3>
    KannalaBrandt8::computeProjectionJacobian(const Eigen::Vector3d &point3D_in)
{
    /* Declare loval variables */
    double x2    = point3D_in[0] * point3D_in[0];
    double y2    = point3D_in[1] * point3D_in[1];
    double z2    = point3D_in[2] * point3D_in[2];
    double r2    = x2 + y2;
    double r     = sqrt(r2);
    double r3    = r2 * r;
    double theta = atan2(r, point3D_in[2]);

    double theta2 = theta * theta;
    double theta3 = theta2 * theta;
    double theta4 = theta2 * theta2;
    double theta5 = theta4 * theta;
    double theta6 = theta2 * theta4;
    double theta7 = theta6 * theta;
    double theta8 = theta4 * theta4;
    double theta9 = theta8 * theta;

    double f = theta + theta3 * parameters[4] + theta5 * parameters[5] +
               theta7 * parameters[6] + theta9 * parameters[7];
    double fd = 1 + 3 * parameters[4] * theta2 + 5 * parameters[5] * theta4 +
                7 * parameters[6] * theta6 + 9 * parameters[7] * theta8;

    Eigen::Matrix<double, 2, 3> JacGood;
    JacGood(0, 0) = parameters[0] *
                    (fd * point3D_in[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
    JacGood(1, 0) = parameters[1] * (fd * point3D_in[2] * point3D_in[1] *
                                         point3D_in[0] / (r2 * (r2 + z2)) -
                                     f * point3D_in[1] * point3D_in[0] / r3);

    JacGood(0, 1) = parameters[0] * (fd * point3D_in[2] * point3D_in[1] *
                                         point3D_in[0] / (r2 * (r2 + z2)) -
                                     f * point3D_in[1] * point3D_in[0] / r3);
    JacGood(1, 1) = parameters[1] *
                    (fd * point3D_in[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

    JacGood(0, 2) = -parameters[0] * fd * point3D_in[0] / (r2 + z2);
    JacGood(1, 2) = -parameters[1] * fd * point3D_in[1] / (r2 + z2);

    return JacGood;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
