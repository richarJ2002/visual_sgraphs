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
 * @file            uncertainty2.cc
 *
 * @brief           Implements KannalaBrandt8::uncertainty2(), declared
 *                  in CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
float KannalaBrandt8::uncertainty2(
    [[maybe_unused]] const Eigen::Matrix<double, 2, 1> &point2d_in)
{
    /*Eigen::Matrix<double,2,1> c;
    c << parameters[2], parameters[3];
    if ((point2D_in-c).squaredNorm()>57600) // 240*240 (256)
        return 100.f;
    else
        return 1.0f;*/
    return 1.f;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
