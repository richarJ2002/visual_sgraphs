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
PinholeStatus Pinhole::isEqual(geometriccamera::GeometricCamera *p_camera_in,
                               bool                             &isEqual_out)
{
    unsigned int cameraType{};
    if (p_camera_in->getType(cameraType) !=
        geometriccamera::GeometricCameraStatus::GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // getType cannot fail; continue as before.
    }
    if (cameraType != geometriccamera::GeometricCamera::CAM_PINHOLE)
    {
        isEqual_out = false;
        return PinholeStatus::PINHOLE_STATUS_SUCCESS;
    }

    Pinhole *p_otherPinhole = (Pinhole *)p_camera_in;

    size_t size2{};
    if (size(size2) !=
        geometriccamera::GeometricCameraStatus::GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // size cannot fail; continue as before.
    }
    size_t otherPinholeSize{};
    if (p_otherPinhole->size(otherPinholeSize) !=
        geometriccamera::GeometricCameraStatus::GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // size cannot fail; continue as before.
    }
    if (size2 != otherPinholeSize)
    {
        isEqual_out = false;
        return PinholeStatus::PINHOLE_STATUS_SUCCESS;
    }

    bool   isSameCamera = true;
    size_t size3{};
    if (size(size3) !=
        geometriccamera::GeometricCameraStatus::GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // size cannot fail; continue as before.
    }
    for (size_t parameterIndex = 0; parameterIndex < size3; ++parameterIndex)
    {
        float otherPinholeParameter{};
        if (p_otherPinhole->getParameter(parameterIndex,
                                         otherPinholeParameter) !=
            geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            // getParameter cannot fail; continue as before.
        }
        if (abs(parameters[parameterIndex] - otherPinholeParameter) > 1e-6)
        {
            isSameCamera = false;
            break;
        }
    }
    isEqual_out = isSameCamera;
    return PinholeStatus::PINHOLE_STATUS_SUCCESS;
}
} // namespace pinhole
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
