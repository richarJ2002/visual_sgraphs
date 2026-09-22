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
 * @file            static_members.cc
 *
 * @brief           Defines the GeometricCamera static data members,
 *                  declared in
 * CameraModels/GeometricCamera/objects/GeometricCamera.h.
 */

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"

namespace vs_graphs::core::camera_models::geometriccamera
{
long unsigned int GeometricCamera::nextId = 0;
} // namespace vs_graphs::core::camera_models::geometriccamera
