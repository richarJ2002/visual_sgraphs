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
 * @brief           Implements the historical Settings stream-output contract.
 */

#include "configuration/settings/objects/SettingsClass.h"

#include <cstddef>
#include <ostream>
#include <vector>

#include <rclcpp/logging.hpp>

#include "sensor_models/camera_models/geometric_camera/geometric_camera.h"
#include "sensor_models/camera_models/kannala_brandt8/kannala_brandt8.h"
#include "execution/system/objects/SensorTypeEnum.h"

namespace vs_graphs::core::utils::settings
{

std::ostream &operator<<(std::ostream   &leftHandSideStream_inout,
                         const Settings &rightHandSideSettings_in)
{
    /* Camera#1 */
    leftHandSideStream_inout << "\t- Camera#1 parameters (";
    if (rightHandSideSettings_in.cameraModel == Settings::CameraType::PINHOLE ||
        rightHandSideSettings_in.cameraModel == Settings::CameraType::RECTIFIED)
    {
        leftHandSideStream_inout << "Pinhole";
    }
    else
    {
        leftHandSideStream_inout << "Kannala-Brandt";
    }
    leftHandSideStream_inout << "): [";
    std::size_t originalCalibration1Count{};
    if (rightHandSideSettings_in.p_originalCalibration1->size(
            originalCalibration1Count) !=
        camera_models::geometric_camera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: size returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    for (std::size_t calibrationIndex = 0;
         calibrationIndex < originalCalibration1Count;
         calibrationIndex++)
    {
        float originalCalibration1Parameter{};
        if (rightHandSideSettings_in.p_originalCalibration1->getParameter(
                calibrationIndex,
                originalCalibration1Parameter) !=
            camera_models::geometric_camera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        leftHandSideStream_inout << " " << originalCalibration1Parameter;
    }
    leftHandSideStream_inout << " ]" << std::endl;

    if (!rightHandSideSettings_in.pinholeDistortion1.empty())
    {
        leftHandSideStream_inout << "\t- Camera#1 distortion parameters: [ ";
        /* Historical output uses float for double-stored coefficients. */
        for (float distortionCoefficient :
             rightHandSideSettings_in.pinholeDistortion1)
        {
            leftHandSideStream_inout << " " << distortionCoefficient;
        }
        leftHandSideStream_inout << " ]" << std::endl;
    }

    if ((rightHandSideSettings_in.sensor == system::STEREO ||
         rightHandSideSettings_in.sensor == system::IMU_STEREO) &&
        (rightHandSideSettings_in.cameraModel !=
         Settings::CameraType::RECTIFIED))
    {
        leftHandSideStream_inout << "\t- Camera#2 parameters (";
        if (rightHandSideSettings_in.cameraModel ==
            Settings::CameraType::PINHOLE)
        {
            leftHandSideStream_inout << "Pinhole";
        }
        else
        {
            leftHandSideStream_inout << "Kannala-Brandt";
        }
        leftHandSideStream_inout << "): [";
        std::size_t originalCalibration2Count{};
        if (rightHandSideSettings_in.p_originalCalibration2->size(
                originalCalibration2Count) !=
            camera_models::geometric_camera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: size returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (std::size_t calibrationIndex = 0;
             calibrationIndex < originalCalibration2Count;
             calibrationIndex++)
        {
            float originalCalibration2Parameter{};
            if (rightHandSideSettings_in.p_originalCalibration2->getParameter(
                    calibrationIndex,
                    originalCalibration2Parameter) !=
                camera_models::geometric_camera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            leftHandSideStream_inout << " " << originalCalibration2Parameter;
        }
        leftHandSideStream_inout << " ]" << std::endl;

        if (!rightHandSideSettings_in.pinholeDistortion2.empty())
        {
            leftHandSideStream_inout
                << "\t- Camera#2 distortion parameters: [ ";
            for (float distortionCoefficient :
                 rightHandSideSettings_in.pinholeDistortion2)
            {
                leftHandSideStream_inout << " " << distortionCoefficient;
            }
            leftHandSideStream_inout << " ]" << std::endl;
        }
    }

    leftHandSideStream_inout
        << "\t- Original frame size: [ "
        << rightHandSideSettings_in.originalImageSize.width << ","
        << rightHandSideSettings_in.originalImageSize.height << " ]"
        << std::endl;
    leftHandSideStream_inout
        << "\t- Current frame size: [ "
        << rightHandSideSettings_in.newImageSize.width << ","
        << rightHandSideSettings_in.newImageSize.height << " ]" << std::endl;

    if (rightHandSideSettings_in.isRectificationNeeded)
    {
        leftHandSideStream_inout
            << "\t- Camera#1 parameters after rectification: [";
        std::size_t calibration1RectifiedCount{};
        if (rightHandSideSettings_in.p_calibration1->size(
                calibration1RectifiedCount) !=
            camera_models::geometric_camera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: size returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (std::size_t calibrationIndex = 0;
             calibrationIndex < calibration1RectifiedCount;
             calibrationIndex++)
        {
            float calibration1RectifiedParameter{};
            if (rightHandSideSettings_in.p_calibration1->getParameter(
                    calibrationIndex,
                    calibration1RectifiedParameter) !=
                camera_models::geometric_camera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            leftHandSideStream_inout << " " << calibration1RectifiedParameter;
        }
        leftHandSideStream_inout << " ]" << std::endl;

        if (rightHandSideSettings_in.sensor == system::STEREO ||
            rightHandSideSettings_in.sensor == system::IMU_STEREO)
        {
            leftHandSideStream_inout
                << "\t- Camera#2 parameters after rectification: [";
            std::size_t calibration2RectifiedCount{};
            if (rightHandSideSettings_in.p_calibration2->size(
                    calibration2RectifiedCount) !=
                camera_models::geometric_camera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: size returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            for (std::size_t calibrationIndex = 0;
                 calibrationIndex < calibration2RectifiedCount;
                 calibrationIndex++)
            {
                float calibration2RectifiedParameter{};
                if (rightHandSideSettings_in.p_calibration2->getParameter(
                        calibrationIndex,
                        calibration2RectifiedParameter) !=
                    camera_models::geometric_camera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                leftHandSideStream_inout << " "
                                         << calibration2RectifiedParameter;
            }
            leftHandSideStream_inout << " ]" << std::endl;
        }
    }
    else if (rightHandSideSettings_in.isFirstResizeNeeded)
    {
        leftHandSideStream_inout << "\t- Camera#1 parameters after resize: [";
        std::size_t calibration1ResizedCount{};
        if (rightHandSideSettings_in.p_calibration1->size(
                calibration1ResizedCount) !=
            camera_models::geometric_camera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: size returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (std::size_t calibrationIndex = 0;
             calibrationIndex < calibration1ResizedCount;
             calibrationIndex++)
        {
            float calibration1ResizedParameter{};
            if (rightHandSideSettings_in.p_calibration1->getParameter(
                    calibrationIndex,
                    calibration1ResizedParameter) !=
                camera_models::geometric_camera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            leftHandSideStream_inout << " " << calibration1ResizedParameter;
        }
        leftHandSideStream_inout << " ]" << std::endl;

        if ((rightHandSideSettings_in.sensor == system::STEREO ||
             rightHandSideSettings_in.sensor == system::IMU_STEREO) &&
            rightHandSideSettings_in.cameraModel ==
                Settings::CameraType::KANNALA_BRANDT)
        {
            leftHandSideStream_inout
                << "\t- Camera#2 parameters after resize: [";
            std::size_t calibration2ResizedCount{};
            if (rightHandSideSettings_in.p_calibration2->size(
                    calibration2ResizedCount) !=
                camera_models::geometric_camera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: size returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            for (std::size_t calibrationIndex = 0;
                 calibrationIndex < calibration2ResizedCount;
                 calibrationIndex++)
            {
                float calibration2ResizedParameter{};
                if (rightHandSideSettings_in.p_calibration2->getParameter(
                        calibrationIndex,
                        calibration2ResizedParameter) !=
                    camera_models::geometric_camera::GeometricCameraStatus::
                        GEOMETRIC_CAMERA_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getParameter returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                leftHandSideStream_inout << " " << calibration2ResizedParameter;
            }
            leftHandSideStream_inout << " ]" << std::endl;
        }
    }

    /* Frame rate */
    leftHandSideStream_inout
        << "\t- Sequence FPS: " << rightHandSideSettings_in.framesPerSecond
        << std::endl;

    /* Stereo stuff */
    if (rightHandSideSettings_in.sensor == system::STEREO ||
        rightHandSideSettings_in.sensor == system::IMU_STEREO)
    {
        leftHandSideStream_inout << "\t- Stereo baseline: "
                                 << rightHandSideSettings_in.stereoBaseline
                                 << std::endl;
        leftHandSideStream_inout << "\t- Stereo depth threshold : "
                                 << rightHandSideSettings_in.depthThreshold
                                 << std::endl;

        if (rightHandSideSettings_in.cameraModel ==
            Settings::CameraType::KANNALA_BRANDT)
        {
            std::vector<int> camera1OverlappingArea =
                static_cast<camera_models::kannala_brandt8::KannalaBrandt8 *>(
                    rightHandSideSettings_in.p_calibration1.get())
                    ->lappingArea;
            std::vector<int> camera2OverlappingArea =
                static_cast<camera_models::kannala_brandt8::KannalaBrandt8 *>(
                    rightHandSideSettings_in.p_calibration2.get())
                    ->lappingArea;
            leftHandSideStream_inout << "\t- Camera 1 overlapping area: [ "
                                     << camera1OverlappingArea[0] << " , "
                                     << camera1OverlappingArea[1] << " ]"
                                     << std::endl;
            leftHandSideStream_inout << "\t- Camera 2 overlapping area: [ "
                                     << camera2OverlappingArea[0] << " , "
                                     << camera2OverlappingArea[1] << " ]"
                                     << std::endl;
        }
    }

    /* IMU parameters */
    if (rightHandSideSettings_in.sensor == system::IMU_MONOCULAR ||
        rightHandSideSettings_in.sensor == system::IMU_STEREO ||
        rightHandSideSettings_in.sensor == system::IMU_RGBD)
    {
        leftHandSideStream_inout
            << "\t- Gyro noise: " << rightHandSideSettings_in.gyroNoise
            << std::endl;
        leftHandSideStream_inout << "\t- Accelerometer noise: "
                                 << rightHandSideSettings_in.accelNoise
                                 << std::endl;
        leftHandSideStream_inout
            << "\t- Gyro walk: " << rightHandSideSettings_in.gyroWalkNoise
            << std::endl;
        leftHandSideStream_inout << "\t- Accelerometer walk: "
                                 << rightHandSideSettings_in.accelWalkNoise
                                 << std::endl;
        leftHandSideStream_inout
            << "\t- IMU frequency: " << rightHandSideSettings_in.imuSampleRate
            << std::endl;
        leftHandSideStream_inout << "\t- IMU threshold: "
                                 << rightHandSideSettings_in.imuErrorThreshold
                                 << std::endl;
    }

    /* RGB-D parameters */
    if (rightHandSideSettings_in.sensor == system::RGBD ||
        rightHandSideSettings_in.sensor == system::IMU_RGBD)
    {
        leftHandSideStream_inout << "\t- RGB-D depth map factor: "
                                 << rightHandSideSettings_in.depthMapScale
                                 << std::endl;
        leftHandSideStream_inout << "\t- Stereo depth threshold: "
                                 << rightHandSideSettings_in.depthThreshold
                                 << std::endl;
        leftHandSideStream_inout << "\t- Metric close depth: "
                                 << rightHandSideSettings_in.stereoBaseline *
                                        rightHandSideSettings_in.depthThreshold
                                 << std::endl;
    }

    /* ORB parameters */
    leftHandSideStream_inout
        << "\t- Features per image: " << rightHandSideSettings_in.featureCount
        << std::endl;
    leftHandSideStream_inout
        << "\t- ORB scale factor: " << rightHandSideSettings_in.orbScaleFactor
        << std::endl;
    leftHandSideStream_inout << "\t- ORB number of scales: "
                             << rightHandSideSettings_in.pyramidLevels
                             << std::endl;
    leftHandSideStream_inout << "\t- Initial FAST threshold: "
                             << rightHandSideSettings_in.initialFastThreshold
                             << std::endl;
    leftHandSideStream_inout << "\t- Min FAST threshold: "
                             << rightHandSideSettings_in.minimumFastThreshold
                             << std::endl;

    return leftHandSideStream_inout;
}

} /* namespace vs_graphs::core::utils::settings */
