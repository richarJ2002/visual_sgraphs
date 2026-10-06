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
 * @file            readCamera1.cc
 *
 * @brief           Implements Settings::readCamera1(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/persistence.hpp>
#include <rclcpp/logging.hpp>

#include "System.h"

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

SettingsStatus Settings::readCamera1(cv::FileStorage &storage_inout)
{
    // Variables
    bool               found;
    std::vector<float> calibrations;

    // Camera model
    std::string cameraModelName{};
    if (readParameter<std::string>(storage_inout,
                                   "Camera.type",
                                   found,
                                   cameraModelName) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (cameraModelName == "PinHole")
    {
        cameraModel = CameraType::PINHOLE;

        // Intrinsic parameters
        float fx{};
        if (readParameter<float>(storage_inout, "Camera1.fx", found, fx) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float fy{};
        if (readParameter<float>(storage_inout, "Camera1.fy", found, fy) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float cx{};
        if (readParameter<float>(storage_inout, "Camera1.cx", found, cx) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float cy{};
        if (readParameter<float>(storage_inout, "Camera1.cy", found, cy) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        calibrations = {fx, fy, cx, cy};

        p_calibration1 =
            std::make_unique<camera_models::pinhole::Pinhole>(calibrations);
        p_originalCalibration1 =
            std::make_unique<camera_models::pinhole::Pinhole>(calibrations);

        // Check if the Pinhole is distorted
        float parameter{};
        if (readParameter<float>(storage_inout,
                                 "Camera1.k1",
                                 found,
                                 parameter,
                                 false) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (found)
        {
            float parameter2{};
            if (readParameter<float>(storage_inout,
                                     "Camera1.k3",
                                     found,
                                     parameter2,
                                     false) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (found)
            {
                pinholeDistortion1.resize(5);
                float parameter3{};
                if (readParameter<float>(storage_inout,
                                         "Camera1.k3",
                                         found,
                                         parameter3) !=
                    SettingsStatus::SETTINGS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: readParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                pinholeDistortion1[4] = parameter3;
            }
            else
                pinholeDistortion1.resize(4);
            float parameter4{};
            if (readParameter<float>(storage_inout,
                                     "Camera1.k1",
                                     found,
                                     parameter4) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            pinholeDistortion1[0] = parameter4;
            float parameter5{};
            if (readParameter<float>(storage_inout,
                                     "Camera1.k2",
                                     found,
                                     parameter5) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            pinholeDistortion1[1] = parameter5;
            float parameter6{};
            if (readParameter<float>(storage_inout,
                                     "Camera1.p1",
                                     found,
                                     parameter6) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            pinholeDistortion1[2] = parameter6;
            float parameter7{};
            if (readParameter<float>(storage_inout,
                                     "Camera1.p2",
                                     found,
                                     parameter7) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            pinholeDistortion1[3] = parameter7;
        }

        // Check if we need to correct distortion from the images
        if ((sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR) &&
            pinholeDistortion1.size() != 0)
            isUndistortionNeeded = true;
    }
    else if (cameraModelName == "Rectified")
    {
        cameraModel = CameraType::RECTIFIED;

        // Intrinsic parameters
        float fx{};
        if (readParameter<float>(storage_inout, "Camera1.fx", found, fx) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float fy{};
        if (readParameter<float>(storage_inout, "Camera1.fy", found, fy) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float cx{};
        if (readParameter<float>(storage_inout, "Camera1.cx", found, cx) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float cy{};
        if (readParameter<float>(storage_inout, "Camera1.cy", found, cy) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        calibrations = {fx, fy, cx, cy};

        p_calibration1 =
            std::make_unique<camera_models::pinhole::Pinhole>(calibrations);
        p_originalCalibration1 =
            std::make_unique<camera_models::pinhole::Pinhole>(calibrations);
    }
    else if (cameraModelName == "camera_models::KannalaBrandt8")
    {
        cameraModel = CameraType::KANNALA_BRANDT;

        // Read intrinsic parameters
        float fx{};
        if (readParameter<float>(storage_inout, "Camera1.fx", found, fx) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float fy{};
        if (readParameter<float>(storage_inout, "Camera1.fy", found, fy) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float cx{};
        if (readParameter<float>(storage_inout, "Camera1.cx", found, cx) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float cy{};
        if (readParameter<float>(storage_inout, "Camera1.cy", found, cy) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        float k0{};
        if (readParameter<float>(storage_inout, "Camera1.k1", found, k0) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float k1{};
        if (readParameter<float>(storage_inout, "Camera1.k2", found, k1) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float k2{};
        if (readParameter<float>(storage_inout, "Camera1.k3", found, k2) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float k3{};
        if (readParameter<float>(storage_inout, "Camera1.k4", found, k3) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        calibrations = {fx, fy, cx, cy, k0, k1, k2, k3};
        p_calibration1 =
            std::make_unique<camera_models::kannalabrandt8::KannalaBrandt8>(
                calibrations);
        p_originalCalibration1 =
            std::make_unique<camera_models::kannalabrandt8::KannalaBrandt8>(
                calibrations);

        if (sensor == System::STEREO || sensor == System::IMU_STEREO)
        {
            int colBegin{};
            if (readParameter<int>(storage_inout,
                                   "Camera1.overlappingBegin",
                                   found,
                                   colBegin) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int colEnd{};
            if (readParameter<int>(storage_inout,
                                   "Camera1.overlappingEnd",
                                   found,
                                   colEnd) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            std::vector<int> overlappings = {colBegin, colEnd};
            static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                p_calibration1.get())
                ->lappingArea = overlappings;
        }
    }
    else
    {
        VSLAM_LOG_ERROR("[Settings] Could not find Camera#1 settings for '%s'! "
                        "Exiting ...\n",
                        cameraModelName.c_str());
        exit(-1);
    }

    return SettingsStatus::SETTINGS_STATUS_SUCCESS;
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
