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
 * @file            newParameterLoader.cc
 *
 * @brief           Implements Tracking::newParameterLoader(), declared in
 *                  Tracking.h.
 */

#include "FrameDrawer.h"
#include "System.h"
#include "Tracking.h"

#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus
    Tracking::newParameterLoader(utils::settings::Settings *p_settings_inout)
{
    camera_models::geometriccamera::GeometricCamera *p_settingsCamera1 =
        nullptr;
    if (p_settings_inout->camera1(p_settingsCamera1) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: camera1 returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    p_camera = p_settingsCamera1;
    camera_models::geometriccamera::GeometricCamera *p_atlasCamera = nullptr;
    if (p_atlas->addCamera(p_camera, p_atlasCamera) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addCamera returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_camera = p_atlasCamera;

    bool settingsNeedToUndistort{};
    if (p_settings_inout->needToUndistort(settingsNeedToUndistort) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: needToUndistort returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (settingsNeedToUndistort)
    {
        cv::Mat settingsCamera1DistortionCoef{};
        if (p_settings_inout->camera1DistortionCoef(
                settingsCamera1DistortionCoef) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: camera1DistortionCoef returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        distortionCoefficients = settingsCamera1DistortionCoef;
    }
    else
    {
        distortionCoefficients = cv::Mat::zeros(4, 1, CV_32F);
    }

    // TODO: missing image scaling and rectification
    imageScale = 1.0f;

    calibrationMatrix = cv::Mat::eye(3, 3, CV_32F);
    float cameraParameter{};
    if (p_camera->getParameter(0, cameraParameter) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    calibrationMatrix.at<float>(0, 0) = cameraParameter;
    float cameraParameter2{};
    if (p_camera->getParameter(1, cameraParameter2) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    calibrationMatrix.at<float>(1, 1) = cameraParameter2;
    float cameraParameter3{};
    if (p_camera->getParameter(2, cameraParameter3) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    calibrationMatrix.at<float>(0, 2) = cameraParameter3;
    float cameraParameter4{};
    if (p_camera->getParameter(3, cameraParameter4) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    calibrationMatrix.at<float>(1, 2) = cameraParameter4;

    calibrationMatrixEigen.setIdentity();
    float cameraParameter5{};
    if (p_camera->getParameter(0, cameraParameter5) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    calibrationMatrixEigen(0, 0) = cameraParameter5;
    float cameraParameter6{};
    if (p_camera->getParameter(1, cameraParameter6) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    calibrationMatrixEigen(1, 1) = cameraParameter6;
    float cameraParameter7{};
    if (p_camera->getParameter(2, cameraParameter7) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    calibrationMatrixEigen(0, 2) = cameraParameter7;
    float cameraParameter8{};
    if (p_camera->getParameter(3, cameraParameter8) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParameter returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    calibrationMatrixEigen(1, 2) = cameraParameter8;

    utils::settings::Settings::CameraType settingsCameraType{};
    if (((sensor == System::STEREO || sensor == System::IMU_STEREO ||
          sensor == System::IMU_RGBD)) &&
        p_settings_inout->cameraType(settingsCameraType) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: cameraType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if ((sensor == System::STEREO || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        settingsCameraType ==
            utils::settings::Settings::CameraType::KANNALA_BRANDT)
    {
        camera_models::geometriccamera::GeometricCamera *p_settingsCamera2 =
            nullptr;
        if (p_settings_inout->camera2(p_settingsCamera2) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: camera2 returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_camera2 = p_settingsCamera2;
        camera_models::geometriccamera::GeometricCamera *p_atlasCamera2 =
            nullptr;
        if (p_atlas->addCamera(p_camera2, p_atlasCamera2) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addCamera returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_camera2 = p_atlasCamera2;

        Sophus::SE3f settingsLeftToRightTransform{};
        if (p_settings_inout->getLeftToRightTransform(
                settingsLeftToRightTransform) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getLeftToRightTransform returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        poseTlr = settingsLeftToRightTransform;

        p_frameDrawer->shouldDrawBothImages = true;
    }

    if (sensor == System::STEREO || sensor == System::RGBD ||
        sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        double settingsBaselineFocal{};
        if (p_settings_inout->getBaselineFocal(settingsBaselineFocal) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getBaselineFocal returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        mbf = settingsBaselineFocal;
        double settingsB{};
        if (p_settings_inout->b(settingsB) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: b returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        double settingsThDepth{};
        if (p_settings_inout->thDepth(settingsThDepth) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: thDepth returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        depthThreshold = settingsB * settingsThDepth;
    }

    if (sensor == System::RGBD || sensor == System::IMU_RGBD)
    {
        double settingsDepthMapFactor{};
        if (p_settings_inout->depthMapFactor(settingsDepthMapFactor) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: depthMapFactor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        depthMapFactor = settingsDepthMapFactor;
        if (std::fabs(depthMapFactor) < 1e-5)
            depthMapFactor = 1;
        else
            depthMapFactor = 1.0f / depthMapFactor;
    }

    minFrames = 0;
    double settingsFramesPerSecond{};
    if (p_settings_inout->getFramesPerSecond(settingsFramesPerSecond) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getFramesPerSecond returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    maxFrames = settingsFramesPerSecond;
    bool settingsIsRgbEnabled{};
    if (p_settings_inout->isRgbEnabled(settingsIsRgbEnabled) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isRgbEnabled returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    isRgbEnabled = settingsIsRgbEnabled;

    // ORB parameters
    int featureCount{};
    if (p_settings_inout->nFeatures(featureCount) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: nFeatures returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    int levelCount{};
    if (p_settings_inout->nLevels(levelCount) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: nLevels returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    double initialThresholdFastValue{};
    if (p_settings_inout->initThFAST(initialThresholdFastValue) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: initThFAST returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    int    initialThresholdFast = static_cast<int>(initialThresholdFastValue);
    double minimumThresholdFastValue{};
    if (p_settings_inout->getMinimumFastThreshold(minimumThresholdFastValue) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMinimumFastThreshold returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    int    minimumThresholdFast = static_cast<int>(minimumThresholdFastValue);
    double scaleFactorValue{};
    if (p_settings_inout->scaleFactor(scaleFactorValue) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: scaleFactor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    float scaleFactor = static_cast<float>(scaleFactorValue);

    p_orbExtractorLeft = std::make_unique<ORBextractor>(featureCount,
                                                        scaleFactor,
                                                        levelCount,
                                                        initialThresholdFast,
                                                        minimumThresholdFast);

    if (sensor == System::STEREO || sensor == System::IMU_STEREO)
        p_orbExtractorRight =
            std::make_unique<ORBextractor>(featureCount,
                                           scaleFactor,
                                           levelCount,
                                           initialThresholdFast,
                                           minimumThresholdFast);

    if (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR)
        p_iniOrbExtractor =
            std::make_unique<ORBextractor>(5 * featureCount,
                                           scaleFactor,
                                           levelCount,
                                           initialThresholdFast,
                                           minimumThresholdFast);

    // Adaptive FAST threshold initialization
    lastFrameFeatures        = 0;
    consecutiveLowFeatures   = 0;
    baseInitialFastThreshold = initialThresholdFast;
    baseMinimumFastThreshold = minimumThresholdFast;

    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
    {
        Sophus::SE3f extrinsicPose_cameraToBody{};
        if (p_settings_inout->Tbc(extrinsicPose_cameraToBody) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: Tbc returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        double settingsImuFrequency{};
        if (p_settings_inout->imuFrequency(settingsImuFrequency) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: imuFrequency returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        imuFrequency = settingsImuFrequency;
        double settingsImuThreshold{};
        if (p_settings_inout->imuThreshold(settingsImuThreshold) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: imuThreshold returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        imuThresh = settingsImuThreshold;
        bool settingsInsertKFsWhenLost{};
        if (p_settings_inout->insertKFsWhenLost(settingsInsertKFsWhenLost) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: insertKFsWhenLost returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        shouldInsertKeyFramesWhenLost = settingsInsertKFsWhenLost;
        bool settingsFastInit{};
        if (p_settings_inout->fastInit(settingsFastInit) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: fastInit returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        isFastInitEnabled = settingsFastInit;
        imuPeriod         = 1.0 / static_cast<double>(imuFrequency);
        double noiseGyroValue{};
        if (p_settings_inout->noiseGyro(noiseGyroValue) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: noiseGyro returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        float  Ng = static_cast<float>(noiseGyroValue);
        double noiseAccValue{};
        if (p_settings_inout->noiseAcc(noiseAccValue) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: noiseAcc returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        float  Na = static_cast<float>(noiseAccValue);
        double gyroWalkValue{};
        if (p_settings_inout->gyroWalk(gyroWalkValue) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: gyroWalk returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        float  gwCount = static_cast<float>(gyroWalkValue);
        double accWalkValue{};
        if (p_settings_inout->accWalk(accWalkValue) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: accWalk returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        float awCount = static_cast<float>(accWalkValue);

        const float sf = std::sqrt(imuFrequency);
        p_imuCalibration =
            std::make_unique<IMU::Calib>(extrinsicPose_cameraToBody,
                                         Ng * sf,
                                         Na * sf,
                                         gwCount / sf,
                                         awCount / sf);

        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
    }

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
