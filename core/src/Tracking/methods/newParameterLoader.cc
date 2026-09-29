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

#include "FrameDrawer.h"
#include "System.h"
#include "Tracking.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{

void Tracking::newParameterLoader(utils::settings::Settings *p_settings_inout)
{
    camera_models::geometriccamera::GeometricCamera *p_settingsCamera1 =
        nullptr;
    if (p_settings_inout->camera1(p_settingsCamera1) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // camera1 cannot fail; continue as before.
    }
    p_camera = p_settingsCamera1;
    p_camera = p_atlas->addCamera(p_camera);

    bool settingsNeedToUndistort{};
    if (p_settings_inout->needToUndistort(settingsNeedToUndistort) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // needToUndistort cannot fail; continue as before.
    }
    if (settingsNeedToUndistort)
    {
        cv::Mat settingsCamera1DistortionCoef{};
        if (p_settings_inout->camera1DistortionCoef(
                settingsCamera1DistortionCoef) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // camera1DistortionCoef cannot fail; continue as before.
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
        // getParameter cannot fail; continue as before.
    }
    calibrationMatrix.at<float>(0, 0) = cameraParameter;
    float cameraParameter2{};
    if (p_camera->getParameter(1, cameraParameter2) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // getParameter cannot fail; continue as before.
    }
    calibrationMatrix.at<float>(1, 1) = cameraParameter2;
    float cameraParameter3{};
    if (p_camera->getParameter(2, cameraParameter3) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // getParameter cannot fail; continue as before.
    }
    calibrationMatrix.at<float>(0, 2) = cameraParameter3;
    float cameraParameter4{};
    if (p_camera->getParameter(3, cameraParameter4) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // getParameter cannot fail; continue as before.
    }
    calibrationMatrix.at<float>(1, 2) = cameraParameter4;

    calibrationMatrixEigen.setIdentity();
    float cameraParameter5{};
    if (p_camera->getParameter(0, cameraParameter5) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // getParameter cannot fail; continue as before.
    }
    calibrationMatrixEigen(0, 0) = cameraParameter5;
    float cameraParameter6{};
    if (p_camera->getParameter(1, cameraParameter6) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // getParameter cannot fail; continue as before.
    }
    calibrationMatrixEigen(1, 1) = cameraParameter6;
    float cameraParameter7{};
    if (p_camera->getParameter(2, cameraParameter7) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // getParameter cannot fail; continue as before.
    }
    calibrationMatrixEigen(0, 2) = cameraParameter7;
    float cameraParameter8{};
    if (p_camera->getParameter(3, cameraParameter8) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        // getParameter cannot fail; continue as before.
    }
    calibrationMatrixEigen(1, 2) = cameraParameter8;

    utils::settings::Settings::CameraType settingsCameraType{};
    if (((sensor == System::STEREO || sensor == System::IMU_STEREO ||
          sensor == System::IMU_RGBD)) &&
        p_settings_inout->cameraType(settingsCameraType) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // cameraType cannot fail; continue as before.
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
            // camera2 cannot fail; continue as before.
        }
        p_camera2 = p_settingsCamera2;
        p_camera2 = p_atlas->addCamera(p_camera2);

        Sophus::SE3f settingsLeftToRightTransform{};
        if (p_settings_inout->getLeftToRightTransform(
                settingsLeftToRightTransform) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // getLeftToRightTransform cannot fail; continue as before.
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
            // getBaselineFocal cannot fail; continue as before.
        }
        mbf = settingsBaselineFocal;
        double settingsB{};
        if (p_settings_inout->b(settingsB) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // b cannot fail; continue as before.
        }
        double settingsThDepth{};
        if (p_settings_inout->thDepth(settingsThDepth) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // thDepth cannot fail; continue as before.
        }
        depthThreshold = settingsB * settingsThDepth;
    }

    if (sensor == System::RGBD || sensor == System::IMU_RGBD)
    {
        double settingsDepthMapFactor{};
        if (p_settings_inout->depthMapFactor(settingsDepthMapFactor) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // depthMapFactor cannot fail; continue as before.
        }
        depthMapFactor = settingsDepthMapFactor;
        if (fabs(depthMapFactor) < 1e-5)
            depthMapFactor = 1;
        else
            depthMapFactor = 1.0f / depthMapFactor;
    }

    minFrames = 0;
    double settingsFramesPerSecond{};
    if (p_settings_inout->getFramesPerSecond(settingsFramesPerSecond) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // getFramesPerSecond cannot fail; continue as before.
    }
    maxFrames = settingsFramesPerSecond;
    bool settingsIsRgbEnabled{};
    if (p_settings_inout->isRgbEnabled(settingsIsRgbEnabled) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // isRgbEnabled cannot fail; continue as before.
    }
    isRgbEnabled = settingsIsRgbEnabled;

    // ORB parameters
    int featureCount{};
    if (p_settings_inout->nFeatures(featureCount) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // nFeatures cannot fail; continue as before.
    }
    int levelCount{};
    if (p_settings_inout->nLevels(levelCount) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // nLevels cannot fail; continue as before.
    }
    double initialThresholdFastValue{};
    if (p_settings_inout->initThFAST(initialThresholdFastValue) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // initThFAST cannot fail; continue as before.
    }
    int    initialThresholdFast = static_cast<int>(initialThresholdFastValue);
    double minimumThresholdFastValue{};
    if (p_settings_inout->getMinimumFastThreshold(minimumThresholdFastValue) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // getMinimumFastThreshold cannot fail; continue as before.
    }
    int    minimumThresholdFast = static_cast<int>(minimumThresholdFastValue);
    double scaleFactorValue{};
    if (p_settings_inout->scaleFactor(scaleFactorValue) !=
        utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        // scaleFactor cannot fail; continue as before.
    }
    float scaleFactor = static_cast<float>(scaleFactorValue);

    p_orbExtractorLeft = new ORBextractor(featureCount,
                                          scaleFactor,
                                          levelCount,
                                          initialThresholdFast,
                                          minimumThresholdFast);

    if (sensor == System::STEREO || sensor == System::IMU_STEREO)
        p_orbExtractorRight = new ORBextractor(featureCount,
                                               scaleFactor,
                                               levelCount,
                                               initialThresholdFast,
                                               minimumThresholdFast);

    if (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR)
        p_iniOrbExtractor = new ORBextractor(5 * featureCount,
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
        Sophus::SE3f Tbc{};
        if (p_settings_inout->Tbc(Tbc) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // Tbc cannot fail; continue as before.
        }
        double settingsImuFrequency{};
        if (p_settings_inout->imuFrequency(settingsImuFrequency) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // imuFrequency cannot fail; continue as before.
        }
        imuFrequency = settingsImuFrequency;
        double settingsImuThreshold{};
        if (p_settings_inout->imuThreshold(settingsImuThreshold) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // imuThreshold cannot fail; continue as before.
        }
        imuThresh = settingsImuThreshold;
        bool settingsInsertKFsWhenLost{};
        if (p_settings_inout->insertKFsWhenLost(settingsInsertKFsWhenLost) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // insertKFsWhenLost cannot fail; continue as before.
        }
        shouldInsertKeyFramesWhenLost = settingsInsertKFsWhenLost;
        bool settingsFastInit{};
        if (p_settings_inout->fastInit(settingsFastInit) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // fastInit cannot fail; continue as before.
        }
        isFastInitEnabled = settingsFastInit;
        imuPeriod         = 1.0 / static_cast<double>(imuFrequency);
        double noiseGyroValue{};
        if (p_settings_inout->noiseGyro(noiseGyroValue) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // noiseGyro cannot fail; continue as before.
        }
        float  Ng = static_cast<float>(noiseGyroValue);
        double noiseAccValue{};
        if (p_settings_inout->noiseAcc(noiseAccValue) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // noiseAcc cannot fail; continue as before.
        }
        float  Na = static_cast<float>(noiseAccValue);
        double gyroWalkValue{};
        if (p_settings_inout->gyroWalk(gyroWalkValue) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // gyroWalk cannot fail; continue as before.
        }
        float  gwCount = static_cast<float>(gyroWalkValue);
        double accWalkValue{};
        if (p_settings_inout->accWalk(accWalkValue) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            // accWalk cannot fail; continue as before.
        }
        float awCount = static_cast<float>(accWalkValue);

        const float sf = sqrt(imuFrequency);
        p_imuCalibration =
            new IMU::Calib(Tbc, Ng * sf, Na * sf, gwCount / sf, awCount / sf);

        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
    }
}

} // namespace core
} // namespace vs_graphs
