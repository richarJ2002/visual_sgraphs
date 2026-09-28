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
 * @file            isEqual.cc
 *
 * @brief           Implements Pinhole::isEqual(), declared in
 *                  CameraModels/Pinhole/objects/Pinhole.h.
 */

#include "CameraModels/Pinhole/objects/Pinhole.h"

#include <cstddef>
#include <cstdlib>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace pinhole
{
bool Pinhole::isEqual(geometriccamera::GeometricCamera *p_camera_in)
{
    if (p_camera_in->getType() != geometriccamera::GeometricCamera::CAM_PINHOLE)
        return false;

    Pinhole *p_otherPinhole = (Pinhole *)p_camera_in;

    if (size() != p_otherPinhole->size())
        return false;

    bool isSameCamera = true;
    for (size_t parameterIndex = 0; parameterIndex < size(); ++parameterIndex)
    {
        if (abs(parameters[parameterIndex] -
                p_otherPinhole->getParameter(parameterIndex)) > 1e-6)
        {
            isSameCamera = false;
            break;
        }
    }
    return isSameCamera;
}
} // namespace pinhole
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
