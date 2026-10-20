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
 *                  pinhole.h.
 */

#include "sensor_models/camera_models/pinhole/objects/PinholeClass.h"
#include "sensor_models/camera_models/geometric_camera/geometric_camera.h"
#include "sensor_models/camera_models/geometric_camera/objects/GeometricCameraStatusEnum.h"

#include <cstddef>
#include <cstdlib>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace pinhole
{
PinholeStatus Pinhole::isEqual(geometric_camera::GeometricCamera *p_camera_in,
                               bool                              &isEqual_out)
{
    unsigned int cameraType{};
    if (p_camera_in->getType(cameraType) !=
        geometric_camera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getType returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (cameraType != geometric_camera::GeometricCamera::CAM_PINHOLE)
    {
        isEqual_out = false;
        return pinhole::PinholeStatus::PINHOLE_STATUS_SUCCESS;
    }

    /* getType() above confirmed the camera is a Pinhole */
    Pinhole *p_otherPinhole = static_cast<Pinhole *>(p_camera_in);

    size_t size2{};
    if (size(size2) != geometric_camera::GeometricCameraStatus::
                           GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: size returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    size_t otherPinholeSize{};
    if (p_otherPinhole->size(otherPinholeSize) !=
        geometric_camera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: size returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (size2 != otherPinholeSize)
    {
        isEqual_out = false;
        return pinhole::PinholeStatus::PINHOLE_STATUS_SUCCESS;
    }

    bool   isSameCamera = true;
    size_t size3{};
    if (size(size3) != geometric_camera::GeometricCameraStatus::
                           GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: size returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    for (size_t parameterIndex = 0; parameterIndex < size3; ++parameterIndex)
    {
        float otherPinholeParameter{};
        if (p_otherPinhole->getParameter(parameterIndex,
                                         otherPinholeParameter) !=
            geometric_camera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (std::abs(parameters[parameterIndex] - otherPinholeParameter) > 1e-6)
        {
            isSameCamera = false;
            break;
        }
    }
    isEqual_out = isSameCamera;
    return pinhole::PinholeStatus::PINHOLE_STATUS_SUCCESS;
}
} /* namespace pinhole */
} /* namespace camera_models */
} /* namespace core */
} /* namespace vs_graphs */
