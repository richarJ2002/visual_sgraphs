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
 * @file            writeSettings.cc
 *
 * @brief           Implements the Settings stream-output operator,
 *                  declared in Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <iostream>
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

std::ostream &operator<<(std::ostream &output_inout, const Settings &s_in)
{
    // Camera#1
    output_inout << "\t- Camera#1 parameters (";
    if (s_in.cameraModel == Settings::CameraType::PINHOLE ||
        s_in.cameraModel == Settings::CameraType::RECTIFIED)
    {
        output_inout << "camera_models::Pinhole";
    }
    else
    {
        output_inout << "Kannala-Brandt";
    }
    output_inout << "): [";
    size_t size2{};
    if (s_in.p_originalCalibration1->size(size2) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: size returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    for (size_t originalCalibration1Index = 0;
         originalCalibration1Index < size2;
         originalCalibration1Index++)
    {
        float parameter{};
        if (s_in.p_originalCalibration1->getParameter(originalCalibration1Index,
                                                      parameter) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        output_inout << " " << parameter;
    }
    output_inout << " ]" << std::endl;

    if (!s_in.pinholeDistortion1.empty())
    {
        output_inout << "\t- Camera#1 distortion parameters: [ ";
        for (float d : s_in.pinholeDistortion1)
        {
            output_inout << " " << d;
        }
        output_inout << " ]" << std::endl;
    }

    if ((s_in.sensor == System::STEREO || s_in.sensor == System::IMU_STEREO) &&
        (s_in.cameraModel != Settings::CameraType::RECTIFIED))
    {
        output_inout << "\t- Camera#2 parameters (";
        if (s_in.cameraModel == Settings::CameraType::PINHOLE)
        {
            output_inout << "camera_models::Pinhole";
        }
        else
        {
            output_inout << "Kannala-Brandt";
        }
        output_inout << "): [";
        size_t size3{};
        if (s_in.p_originalCalibration2->size(size3) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: size returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (size_t originalCalibration1Index = 0;
             originalCalibration1Index < size3;
             originalCalibration1Index++)
        {
            float parameter2{};
            if (s_in.p_originalCalibration2->getParameter(
                    originalCalibration1Index,
                    parameter2) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            output_inout << " " << parameter2;
        }
        output_inout << " ]" << std::endl;

        if (!s_in.pinholeDistortion2.empty())
        {
            output_inout << "\t- Camera#2 distortion parameters: [ ";
            for (float d : s_in.pinholeDistortion2)
            {
                output_inout << " " << d;
            }
            output_inout << " ]" << std::endl;
        }
    }

    output_inout << "\t- Original frame size: [ "
                 << s_in.originalImageSize.width << ","
                 << s_in.originalImageSize.height << " ]" << std::endl;
    output_inout << "\t- Current frame size: [ " << s_in.newImageSize.width
                 << "," << s_in.newImageSize.height << " ]" << std::endl;

    if (s_in.isRectificationNeeded)
    {
        output_inout << "\t- Camera#1 parameters after rectification: [";
        size_t size4{};
        if (s_in.p_calibration1->size(size4) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: size returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (size_t originalCalibration1Index = 0;
             originalCalibration1Index < size4;
             originalCalibration1Index++)
        {
            float parameter3{};
            if (s_in.p_calibration1->getParameter(originalCalibration1Index,
                                                  parameter3) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            output_inout << " " << parameter3;
        }
        output_inout << " ]" << std::endl;

        if (s_in.sensor == System::STEREO || s_in.sensor == System::IMU_STEREO)
        {
            output_inout << "\t- Camera#2 parameters after rectification: [";
            size_t size5{};
            if (s_in.p_calibration2->size(size5) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: size returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            for (size_t originalCalibration1Index = 0;
                 originalCalibration1Index < size5;
                 originalCalibration1Index++)
            {
                float parameter4{};
                if (s_in.p_calibration2->getParameter(originalCalibration1Index,
                                                      parameter4) !=
                    camera_models::geometriccamera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                output_inout << " " << parameter4;
            }
            output_inout << " ]" << std::endl;
        }
    }
    else if (s_in.isFirstResizeNeeded)
    {
        output_inout << "\t- Camera#1 parameters after resize: [";
        size_t size6{};
        if (s_in.p_calibration1->size(size6) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: size returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (size_t originalCalibration1Index = 0;
             originalCalibration1Index < size6;
             originalCalibration1Index++)
        {
            float parameter5{};
            if (s_in.p_calibration1->getParameter(originalCalibration1Index,
                                                  parameter5) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            output_inout << " " << parameter5;
        }
        output_inout << " ]" << std::endl;

        if ((s_in.sensor == System::STEREO ||
             s_in.sensor == System::IMU_STEREO) &&
            s_in.cameraModel == Settings::CameraType::KANNALA_BRANDT)
        {
            output_inout << "\t- Camera#2 parameters after resize: [";
            size_t size7{};
            if (s_in.p_calibration2->size(size7) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: size returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            for (size_t originalCalibration1Index = 0;
                 originalCalibration1Index < size7;
                 originalCalibration1Index++)
            {
                float parameter6{};
                if (s_in.p_calibration2->getParameter(originalCalibration1Index,
                                                      parameter6) !=
                    camera_models::geometriccamera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                output_inout << " " << parameter6;
            }
            output_inout << " ]" << std::endl;
        }
    }

    // Frame rate
    output_inout << "\t- Sequence FPS: " << s_in.framesPerSecond << std::endl;

    // Stereo stuff
    if (s_in.sensor == System::STEREO || s_in.sensor == System::IMU_STEREO)
    {
        output_inout << "\t- Stereo baseline: " << s_in.stereoBaseline
                     << std::endl;
        output_inout << "\t- Stereo depth threshold : " << s_in.depthThreshold
                     << std::endl;

        if (s_in.cameraModel == Settings::CameraType::KANNALA_BRANDT)
        {
            std::vector<int> overlapping1 =
                static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    s_in.p_calibration1)
                    ->lappingArea;
            std::vector<int> overlapping2 =
                static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    s_in.p_calibration2)
                    ->lappingArea;
            output_inout << "\t- Camera 1 overlapping area: [ "
                         << overlapping1[0] << " , " << overlapping1[1] << " ]"
                         << std::endl;
            output_inout << "\t- Camera 2 overlapping area: [ "
                         << overlapping2[0] << " , " << overlapping2[1] << " ]"
                         << std::endl;
        }
    }

    // IMU parameters
    if (s_in.sensor == System::IMU_MONOCULAR ||
        s_in.sensor == System::IMU_STEREO || s_in.sensor == System::IMU_RGBD)
    {
        output_inout << "\t- Gyro noise: " << s_in.gyroNoise << std::endl;
        output_inout << "\t- Accelerometer noise: " << s_in.accelNoise
                     << std::endl;
        output_inout << "\t- Gyro walk: " << s_in.gyroWalkNoise << std::endl;
        output_inout << "\t- Accelerometer walk: " << s_in.accelWalkNoise
                     << std::endl;
        output_inout << "\t- IMU frequency: " << s_in.imuSampleRate
                     << std::endl;
        output_inout << "\t- IMU threshold: " << s_in.imuErrorThreshold
                     << std::endl;
    }

    // RGB-D parameters
    if (s_in.sensor == System::RGBD || s_in.sensor == System::IMU_RGBD)
    {
        output_inout << "\t- RGB-D depth map factor: " << s_in.depthMapScale
                     << std::endl;
        output_inout << "\t- Stereo depth threshold: " << s_in.depthThreshold
                     << std::endl;
        output_inout << "\t- Metric close depth: "
                     << s_in.stereoBaseline * s_in.depthThreshold << std::endl;
    }

    // ORB parameters
    output_inout << "\t- Features per image: " << s_in.featureCount
                 << std::endl;
    output_inout << "\t- ORB scale factor: " << s_in.orbScaleFactor
                 << std::endl;
    output_inout << "\t- ORB number of scales: " << s_in.pyramidLevels
                 << std::endl;
    output_inout << "\t- Initial FAST threshold: " << s_in.initialFastThreshold
                 << std::endl;
    output_inout << "\t- Min FAST threshold: " << s_in.minimumFastThreshold
                 << std::endl;

    return output_inout;
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
