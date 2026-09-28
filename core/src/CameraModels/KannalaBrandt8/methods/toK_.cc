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
 * @file            toK_.cc
 *
 * @brief           Implements KannalaBrandt8::toK_(), declared in
 *                  CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
Eigen::Matrix3f KannalaBrandt8::toK_()
{
    Eigen::Matrix3f calibrationMatrix;
    calibrationMatrix << parameters[0], 0.f, parameters[2], 0.f, parameters[1],
        parameters[3], 0.f, 0.f, 1.f;
    return calibrationMatrix;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
