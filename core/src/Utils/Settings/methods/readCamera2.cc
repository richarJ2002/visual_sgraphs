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
 * @file            readCamera2.cc
 *
 * @brief           Implements Settings::readCamera2(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <opencv2/core/persistence.hpp>
#include <rclcpp/logging.hpp>

#include "Utils/Converter/objects/Converter.h"

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

SettingsStatus Settings::readCamera2(cv::FileStorage &storage_inout)
{
    bool               found;
    std::vector<float> calibrations;
    if (cameraModel == CameraType::PINHOLE)
    {
        isRectificationNeeded = true;

        // Read intrinsic parameters
        float fx{};
        if (readParameter<float>(storage_inout, "Camera2.fx", found, fx) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float fy{};
        if (readParameter<float>(storage_inout, "Camera2.fy", found, fy) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float cx{};
        if (readParameter<float>(storage_inout, "Camera2.cx", found, cx) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float cy{};
        if (readParameter<float>(storage_inout, "Camera2.cy", found, cy) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        calibrations = {fx, fy, cx, cy};

        p_calibration2 = new camera_models::pinhole::Pinhole(calibrations);
        p_originalCalibration2 =
            new camera_models::pinhole::Pinhole(calibrations);

        // Check if it is a distorted Pinhole
        float parameter{};
        if (readParameter<float>(storage_inout,
                                 "Camera2.k1",
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
                                     "Camera2.k3",
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
                pinholeDistortion2.resize(5);
                float parameter3{};
                if (readParameter<float>(storage_inout,
                                         "Camera2.k3",
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
                pinholeDistortion2[4] = parameter3;
            }
            else
            {
                pinholeDistortion2.resize(4);
            }
            float parameter4{};
            if (readParameter<float>(storage_inout,
                                     "Camera2.k1",
                                     found,
                                     parameter4) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            pinholeDistortion2[0] = parameter4;
            float parameter5{};
            if (readParameter<float>(storage_inout,
                                     "Camera2.k2",
                                     found,
                                     parameter5) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            pinholeDistortion2[1] = parameter5;
            float parameter6{};
            if (readParameter<float>(storage_inout,
                                     "Camera2.p1",
                                     found,
                                     parameter6) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            pinholeDistortion2[2] = parameter6;
            float parameter7{};
            if (readParameter<float>(storage_inout,
                                     "Camera2.p2",
                                     found,
                                     parameter7) !=
                SettingsStatus::SETTINGS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: readParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            pinholeDistortion2[3] = parameter7;
        }
    }
    else if (cameraModel == CameraType::KANNALA_BRANDT)
    {
        // Read intrinsic parameters
        float fx{};
        if (readParameter<float>(storage_inout, "Camera2.fx", found, fx) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float fy{};
        if (readParameter<float>(storage_inout, "Camera2.fy", found, fy) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float cx{};
        if (readParameter<float>(storage_inout, "Camera2.cx", found, cx) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float cy{};
        if (readParameter<float>(storage_inout, "Camera2.cy", found, cy) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        float k0{};
        if (readParameter<float>(storage_inout, "Camera2.k1", found, k0) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float k1{};
        if (readParameter<float>(storage_inout, "Camera2.k2", found, k1) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float k2{};
        if (readParameter<float>(storage_inout, "Camera2.k3", found, k2) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        float k3{};
        if (readParameter<float>(storage_inout, "Camera2.k4", found, k3) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        calibrations = {fx, fy, cx, cy, k0, k1, k2, k3};

        p_calibration2 =
            new camera_models::kannalabrandt8::KannalaBrandt8(calibrations);
        p_originalCalibration2 =
            new camera_models::kannalabrandt8::KannalaBrandt8(calibrations);

        int colBegin{};
        if (readParameter<int>(storage_inout,
                               "Camera2.overlappingBegin",
                               found,
                               colBegin) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        int colEnd{};
        if (readParameter<int>(storage_inout,
                               "Camera2.overlappingEnd",
                               found,
                               colEnd) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<int> overlappings = {colBegin, colEnd};

        static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
            p_calibration2)
            ->lappingArea = overlappings;
    }

    // Load stereo extrinsic calibration
    if (cameraModel == CameraType::RECTIFIED)
    {
        float parameter8{};
        if (readParameter<float>(storage_inout,
                                 "Stereo.b",
                                 found,
                                 parameter8) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        stereoBaseline = parameter8;
        float calibration1Parameter{};
        if (p_calibration1->getParameter(0, calibration1Parameter) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        baselineFocal = stereoBaseline * calibration1Parameter;
    }
    else
    {
        cv::Mat cvTlr{};
        if (readParameter<cv::Mat>(storage_inout,
                                   "Stereo.T_c1_c2",
                                   found,
                                   cvTlr) !=
            SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3<float> sophus{};
        if (converter::Converter::toSophus(cvTlr, sophus) !=
            converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: toSophus returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        stereoTransform = sophus;

        // TODO: also search for Trl and invert if necessary

        stereoBaseline = stereoTransform.translation().norm();
        float calibration1Parameter2{};
        if (p_calibration1->getParameter(0, calibration1Parameter2) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        baselineFocal = stereoBaseline * calibration1Parameter2;
    }

    float parameter9{};
    if (readParameter<float>(storage_inout,
                             "Stereo.ThDepth",
                             found,
                             parameter9) !=
        SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: readParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    depthThreshold = parameter9;

    return SettingsStatus::SETTINGS_STATUS_SUCCESS;
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
