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
 * @brief           Implements KannalaBrandt8::isEqual(), declared in
 *                  CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <cstddef>
#include <cstdlib>
#include <rclcpp/logging.hpp>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
KannalaBrandt8Status
    KannalaBrandt8::isEqual(geometriccamera::GeometricCamera *p_camera_in,
                            bool                             &isEqual_out)
{
    unsigned int cameraType{};
    if (p_camera_in->getType(cameraType) !=
        geometriccamera::GeometricCameraStatus::GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getType returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (cameraType != geometriccamera::GeometricCamera::CAM_FISHEYE)
    {
        isEqual_out = false;
        return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
    }

    KannalaBrandt8 *p_kannalaCamera = (KannalaBrandt8 *)p_camera_in;

    float kannalaCameraPrecision{};
    if (p_kannalaCamera->getPrecision(kannalaCameraPrecision) !=
        KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPrecision returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (abs(precision - kannalaCameraPrecision) > 1e-6)
    {
        isEqual_out = false;
        return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
    }

    size_t size2{};
    if (size(size2) !=
        geometriccamera::GeometricCameraStatus::GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: size returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    size_t kannalaCameraSize{};
    if (p_kannalaCamera->size(kannalaCameraSize) !=
        geometriccamera::GeometricCameraStatus::GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: size returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (size2 != kannalaCameraSize)
    {
        isEqual_out = false;
        return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
    }

    bool   isSameCamera = true;
    size_t size3{};
    if (size(size3) !=
        geometriccamera::GeometricCameraStatus::GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: size returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    for (size_t parameterIndex = 0; parameterIndex < size3; ++parameterIndex)
    {
        float kannalaCameraParameter{};
        if (p_kannalaCamera->getParameter(parameterIndex,
                                          kannalaCameraParameter) !=
            geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (abs(parameters[parameterIndex] - kannalaCameraParameter) > 1e-6)
        {
            isSameCamera = false;
            break;
        }
    }
    isEqual_out = isSameCamera;
    return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
