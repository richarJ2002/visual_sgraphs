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

#include "Tracking.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{

void Tracking::newParameterLoader(utils::settings::Settings *p_settings_inout)
{
    p_camera = p_settings_inout->camera1();
    p_camera = p_atlas->addCamera(p_camera);

    if (p_settings_inout->needToUndistort())
    {
        distortionCoefficients = p_settings_inout->camera1DistortionCoef();
    }
    else
    {
        distortionCoefficients = cv::Mat::zeros(4, 1, CV_32F);
    }

    // TODO: missing image scaling and rectification
    imageScale = 1.0f;

    calibrationMatrix                 = cv::Mat::eye(3, 3, CV_32F);
    calibrationMatrix.at<float>(0, 0) = p_camera->getParameter(0);
    calibrationMatrix.at<float>(1, 1) = p_camera->getParameter(1);
    calibrationMatrix.at<float>(0, 2) = p_camera->getParameter(2);
    calibrationMatrix.at<float>(1, 2) = p_camera->getParameter(3);

    calibrationMatrixEigen.setIdentity();
    calibrationMatrixEigen(0, 0) = p_camera->getParameter(0);
    calibrationMatrixEigen(1, 1) = p_camera->getParameter(1);
    calibrationMatrixEigen(0, 2) = p_camera->getParameter(2);
    calibrationMatrixEigen(1, 2) = p_camera->getParameter(3);

    if ((sensor == System::STEREO || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        p_settings_inout->cameraType() ==
            utils::settings::Settings::CameraType::KANNALA_BRANDT)
    {
        p_camera2 = p_settings_inout->camera2();
        p_camera2 = p_atlas->addCamera(p_camera2);

        poseTlr = p_settings_inout->getLeftToRightTransform();

        p_frameDrawer->shouldDrawBothImages = true;
    }

    if (sensor == System::STEREO || sensor == System::RGBD ||
        sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        mbf            = p_settings_inout->getBaselineFocal();
        depthThreshold = p_settings_inout->b() * p_settings_inout->thDepth();
    }

    if (sensor == System::RGBD || sensor == System::IMU_RGBD)
    {
        depthMapFactor = p_settings_inout->depthMapFactor();
        if (fabs(depthMapFactor) < 1e-5)
            depthMapFactor = 1;
        else
            depthMapFactor = 1.0f / depthMapFactor;
    }

    minFrames    = 0;
    maxFrames    = p_settings_inout->getFramesPerSecond();
    isRgbEnabled = p_settings_inout->isRgbEnabled();

    // ORB parameters
    int   featureCount         = p_settings_inout->nFeatures();
    int   levelCount           = p_settings_inout->nLevels();
    int   initialThresholdFast = p_settings_inout->initThFAST();
    int   minimumThresholdFast = p_settings_inout->getMinimumFastThreshold();
    float scaleFactor          = p_settings_inout->scaleFactor();

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
        Sophus::SE3f Tbc              = p_settings_inout->Tbc();
        imuFrequency                  = p_settings_inout->imuFrequency();
        imuThresh                     = p_settings_inout->imuThreshold();
        shouldInsertKeyFramesWhenLost = p_settings_inout->insertKFsWhenLost();
        isFastInitEnabled             = p_settings_inout->fastInit();
        imuPeriod                     = 1.0 / static_cast<double>(imuFrequency);
        float Ng                      = p_settings_inout->noiseGyro();
        float Na                      = p_settings_inout->noiseAcc();
        float gwCount                 = p_settings_inout->gyroWalk();
        float awCount                 = p_settings_inout->accWalk();

        const float sf = sqrt(imuFrequency);
        p_imuCalibration =
            new IMU::Calib(Tbc, Ng * sf, Na * sf, gwCount / sf, awCount / sf);

        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
    }
}

} // namespace core
} // namespace vs_graphs
