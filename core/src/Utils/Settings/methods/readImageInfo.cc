/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            readImageInfo.cc
 *
 * @brief           Implements Settings::readImageInfo(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <opencv2/core/persistence.hpp>
#include <rclcpp/logging.hpp>

#include "System.h"

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

SettingsStatus Settings::readImageInfo(cv::FileStorage &storage_inout)
{
    bool found;
    // Read original and desired image dimensions
    int  originalRows{};
    if (readParameter<int>(storage_inout,
                           "Camera.height",
                           found,
                           originalRows) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    int originalCols{};
    if (readParameter<int>(storage_inout,
                           "Camera.width",
                           found,
                           originalCols) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    originalImageSize.width  = originalCols;
    originalImageSize.height = originalRows;

    newImageSize = originalImageSize;
    int newHeigh{};
    if (readParameter<int>(storage_inout,
                           "Camera.newHeight",
                           found,
                           newHeigh,
                           false) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (found)
    {
        isFirstResizeNeeded = true;
        newImageSize.height = newHeigh;

        if (!isRectificationNeeded)
        {
            // Update calibration
            float scaleRowFactor = static_cast<float>(newImageSize.height) /
                                   static_cast<float>(originalImageSize.height);
            float calibration1Parameter{};
            if (p_calibration1->getParameter(1, calibration1Parameter) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_calibration1->setParameter(calibration1Parameter *
                                                 scaleRowFactor,
                                             1) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            float calibration1Parameter2{};
            if (p_calibration1->getParameter(3, calibration1Parameter2) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_calibration1->setParameter(calibration1Parameter2 *
                                                 scaleRowFactor,
                                             3) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if ((sensor == System::STEREO || sensor == System::IMU_STEREO) &&
                cameraModel != CameraType::RECTIFIED)
            {
                float calibration2Parameter{};
                if (p_calibration2->getParameter(1, calibration2Parameter) !=
                    camera_models::geometriccamera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_calibration2->setParameter(calibration2Parameter *
                                                     scaleRowFactor,
                                                 1) !=
                    camera_models::geometriccamera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                float calibration2Parameter2{};
                if (p_calibration2->getParameter(3, calibration2Parameter2) !=
                    camera_models::geometriccamera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_calibration2->setParameter(calibration2Parameter2 *
                                                     scaleRowFactor,
                                                 3) !=
                    camera_models::geometriccamera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
    }

    int newWidth{};
    if (readParameter<int>(storage_inout,
                           "Camera.newWidth",
                           found,
                           newWidth,
                           false) != SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (found)
    {
        isFirstResizeNeeded = true;
        newImageSize.width  = newWidth;

        if (!isRectificationNeeded)
        {
            // Update calibration
            float scaleColFactor = static_cast<float>(newImageSize.width) /
                                   static_cast<float>(originalImageSize.width);
            float calibration1Parameter3{};
            if (p_calibration1->getParameter(0, calibration1Parameter3) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_calibration1->setParameter(calibration1Parameter3 *
                                                 scaleColFactor,
                                             0) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            float calibration1Parameter4{};
            if (p_calibration1->getParameter(2, calibration1Parameter4) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_calibration1->setParameter(calibration1Parameter4 *
                                                 scaleColFactor,
                                             2) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if ((sensor == System::STEREO || sensor == System::IMU_STEREO) &&
                cameraModel != CameraType::RECTIFIED)
            {
                float calibration2Parameter3{};
                if (p_calibration2->getParameter(0, calibration2Parameter3) !=
                    camera_models::geometriccamera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_calibration2->setParameter(calibration2Parameter3 *
                                                     scaleColFactor,
                                                 0) !=
                    camera_models::geometriccamera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                float calibration2Parameter4{};
                if (p_calibration2->getParameter(2, calibration2Parameter4) !=
                    camera_models::geometriccamera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_calibration2->setParameter(calibration2Parameter4 *
                                                     scaleColFactor,
                                                 2) !=
                    camera_models::geometriccamera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }

                if (cameraModel == CameraType::KANNALA_BRANDT)
                {
                    static_cast<
                        camera_models::kannalabrandt8::KannalaBrandt8 *>(
                        p_calibration1)
                        ->lappingArea[0] *= scaleColFactor;
                    static_cast<
                        camera_models::kannalabrandt8::KannalaBrandt8 *>(
                        p_calibration1)
                        ->lappingArea[1] *= scaleColFactor;

                    static_cast<
                        camera_models::kannalabrandt8::KannalaBrandt8 *>(
                        p_calibration2)
                        ->lappingArea[0] *= scaleColFactor;
                    static_cast<
                        camera_models::kannalabrandt8::KannalaBrandt8 *>(
                        p_calibration2)
                        ->lappingArea[1] *= scaleColFactor;
                }
            }
        }
    }

    int parameter{};
    if (readParameter<int>(storage_inout, "Camera.fps", found, parameter) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    framesPerSecond = parameter;
    int parameter2{};
    if (readParameter<int>(storage_inout, "Camera.RGB", found, parameter2) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    isRgbInputEnabled = static_cast<bool>(parameter2);

    return SettingsStatus::SETTINGS_STATUS_SUCCESS;
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
