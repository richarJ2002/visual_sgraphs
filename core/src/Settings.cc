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

#include "Settings.h"

#include "CameraModels/KannalaBrandt8.h"
#include "CameraModels/Pinhole.h"

#include "System.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>

#include <iostream>

using namespace std;

namespace vs_graphs
{
namespace core
{

template <>
float Settings::readParameter<float>(cv::FileStorage   &storage_in,
                                     const std::string &name_in,
                                     bool              &found_out,
                                     const bool         required_in)
{
    cv::FileNode node = storage_in[name_in];
    if (node.empty())
    {
        if (required_in)
        {
            VSLAM_LOG_ERROR(
                "\t- Required parameter '%s' does not exist! Aborting...\n",
                name_in.c_str());
            exit(-1);
        }
        else
        {
            VSLAM_LOG_WARN("\t- Skipping optional parameter '%s' ...\n",
                           name_in.c_str());
            found_out = false;
            return 0.0f;
        }
    }
    else if (!node.isReal())
    {
        VSLAM_LOG_ERROR(
            "\t- Parameter '%s' is not a real number! Aborting...\n",
            name_in.c_str());
        exit(-1);
    }
    else
    {
        found_out = true;
        return node.real();
    }
}

template <>
int Settings::readParameter<int>(cv::FileStorage   &storage_in,
                                 const std::string &name_in,
                                 bool              &found_out,
                                 const bool         required_in)
{
    cv::FileNode node = storage_in[name_in];
    if (node.empty())
    {
        if (required_in)
        {
            VSLAM_LOG_ERROR(
                "\t- Required parameter '%s' does not exist! Aborting...\n",
                name_in.c_str());
            exit(-1);
        }
        else
        {
            VSLAM_LOG_WARN("\t- Skipping optional parameter '%s' ...\n",
                           name_in.c_str());
            found_out = false;
            return 0;
        }
    }
    else if (!node.isInt())
    {
        VSLAM_LOG_ERROR("\t- Parameter '%s' is not an integer! Aborting...\n",
                        name_in.c_str());
        exit(-1);
    }
    else
    {
        found_out = true;
        return node.operator int();
    }
}

template <>
string Settings::readParameter<string>(cv::FileStorage   &storage_in,
                                       const std::string &name_in,
                                       bool              &found_out,
                                       const bool         required_in)
{
    cv::FileNode node = storage_in[name_in];
    if (node.empty())
    {
        if (required_in)
        {
            VSLAM_LOG_ERROR(
                "\t- Required parameter '%s' does not exist! Aborting...\n",
                name_in.c_str());
            exit(-1);
        }
        else
        {
            VSLAM_LOG_WARN("\t- Skipping optional parameter '%s' ...\n",
                           name_in.c_str());
            found_out = false;
            return string();
        }
    }
    else if (!node.isString())
    {
        VSLAM_LOG_ERROR("\t- Parameter '%s' is not a string! Aborting...\n",
                        name_in.c_str());
        exit(-1);
    }
    else
    {
        found_out = true;
        return node.string();
    }
}

template <>
cv::Mat Settings::readParameter<cv::Mat>(cv::FileStorage   &storage_in,
                                         const std::string &name_in,
                                         bool              &found_out,
                                         const bool         required_in)
{
    cv::FileNode node = storage_in[name_in];
    if (node.empty())
    {
        if (required_in)
        {
            VSLAM_LOG_ERROR(
                "\t- Required parameter '%s' does not exist! Aborting...\n",
                name_in.c_str());
            exit(-1);
        }
        else
        {
            VSLAM_LOG_WARN("\t- Skipping optional parameter '%s' ...\n",
                           name_in.c_str());
            found_out = false;
            return cv::Mat();
        }
    }
    else
    {
        found_out = true;
        return node.mat();
    }
}

Settings::Settings(const std::string &configFilePath_in, const int &sensor_in) :
    undistortNeeded(false),
    rectifyNeeded(false),
    resize1Needed(false),
    resize2Needed(false)
{
    sensor = sensor_in;

    // Open settings file
    cv::FileStorage storage_in(configFilePath_in, cv::FileStorage::READ);
    if (!storage_in.isOpened())
    {
        VSLAM_LOG_ERROR("\n[Settings] Could not open the configuration file at "
                        "'%s'! Aborting...\n",
                        configFilePath_in.c_str());
        exit(-1);
    }
    else
        VSLAM_LOG_INFO("\n[Settings] Loading configurations from '%s'...\n",
                       configFilePath_in.c_str());

    // Read Camera#1 (monocular, stereo or RGB-D)
    readCamera1(storage_in);
    VSLAM_LOG_INFO("[Settings] Camera#1 settings loaded!\n");

    // Read Camera#2 (stereo)
    if (sensor == System::STEREO || sensor == System::IMU_STEREO)
    {
        readCamera2(storage_in);
        VSLAM_LOG_INFO("[Settings] Camera#2 settings loaded!\n");
    }

    // Read image info
    readImageInfo(storage_in);
    VSLAM_LOG_INFO("[Settings] Camera info loaded!\n");

    // Read IMU params
    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
    {
        readIMU(storage_in);
        VSLAM_LOG_INFO("[Settings] IMU calibration settings loaded!\n");
    }

    if (sensor == System::RGBD || sensor == System::IMU_RGBD)
    {
        readRGBD(storage_in);
        VSLAM_LOG_INFO("[Settings] RGB-D settings loaded!\n");
    }

    // Read ORB parameters
    readORB(storage_in);
    VSLAM_LOG_INFO("[Settings] ORB settings loaded!\n");

    // Read Viewer parameters
    readViewer(storage_in);
    VSLAM_LOG_INFO("[Settings] Viewer settings loaded!\n");

    // Read Atlas parameters
    readLoadAndSave(storage_in);
    VSLAM_LOG_INFO("[Settings] ATLAS settings loaded!\n");

    // Read other parameters
    readOtherParameters(storage_in);
    VSLAM_LOG_INFO("[Settings] Misc. parameters loaded!\n");

    if (rectifyNeeded)
    {
        precomputeRectificationMaps();
        VSLAM_LOG_INFO("[Settings] Computed rectification maps!\n");
    }
}

void Settings::readCamera1(cv::FileStorage &storage_in)
{
    // Variables
    bool               found;
    std::vector<float> vCalibration;

    // Camera model
    std::string cameraModelName =
        readParameter<std::string>(storage_in, "Camera.type", found);

    if (cameraModelName == "PinHole")
    {
        cameraModel = CameraType::PINHOLE;

        // Intrinsic parameters
        float fx     = readParameter<float>(storage_in, "Camera1.fx", found);
        float fy     = readParameter<float>(storage_in, "Camera1.fy", found);
        float cx     = readParameter<float>(storage_in, "Camera1.cx", found);
        float cy     = readParameter<float>(storage_in, "Camera1.cy", found);
        vCalibration = {fx, fy, cx, cy};

        calibration1         = new camera_models::Pinhole(vCalibration);
        originalCalibration1 = new camera_models::Pinhole(vCalibration);

        // Check if the Pinhole is distorted
        readParameter<float>(storage_in, "Camera1.k1", found, false);
        if (found)
        {
            readParameter<float>(storage_in, "Camera1.k3", found, false);
            if (found)
            {
                pinholeDistortion1.resize(5);
                pinholeDistortion1[4] =
                    readParameter<float>(storage_in, "Camera1.k3", found);
            }
            else
                pinholeDistortion1.resize(4);
            pinholeDistortion1[0] =
                readParameter<float>(storage_in, "Camera1.k1", found);
            pinholeDistortion1[1] =
                readParameter<float>(storage_in, "Camera1.k2", found);
            pinholeDistortion1[2] =
                readParameter<float>(storage_in, "Camera1.p1", found);
            pinholeDistortion1[3] =
                readParameter<float>(storage_in, "Camera1.p2", found);
        }

        // Check if we need to correct distortion from the images
        if ((sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR) &&
            pinholeDistortion1.size() != 0)
            undistortNeeded = true;
    }
    else if (cameraModelName == "Rectified")
    {
        cameraModel = CameraType::RECTIFIED;

        // Intrinsic parameters
        float fx     = readParameter<float>(storage_in, "Camera1.fx", found);
        float fy     = readParameter<float>(storage_in, "Camera1.fy", found);
        float cx     = readParameter<float>(storage_in, "Camera1.cx", found);
        float cy     = readParameter<float>(storage_in, "Camera1.cy", found);
        vCalibration = {fx, fy, cx, cy};

        calibration1         = new camera_models::Pinhole(vCalibration);
        originalCalibration1 = new camera_models::Pinhole(vCalibration);
    }
    else if (cameraModelName == "camera_models::KannalaBrandt8")
    {
        cameraModel = CameraType::KANNALA_BRANDT;

        // Read intrinsic parameters
        float fx = readParameter<float>(storage_in, "Camera1.fx", found);
        float fy = readParameter<float>(storage_in, "Camera1.fy", found);
        float cx = readParameter<float>(storage_in, "Camera1.cx", found);
        float cy = readParameter<float>(storage_in, "Camera1.cy", found);

        float k0 = readParameter<float>(storage_in, "Camera1.k1", found);
        float k1 = readParameter<float>(storage_in, "Camera1.k2", found);
        float k2 = readParameter<float>(storage_in, "Camera1.k3", found);
        float k3 = readParameter<float>(storage_in, "Camera1.k4", found);

        vCalibration         = {fx, fy, cx, cy, k0, k1, k2, k3};
        calibration1         = new camera_models::KannalaBrandt8(vCalibration);
        originalCalibration1 = new camera_models::KannalaBrandt8(vCalibration);

        if (sensor == System::STEREO || sensor == System::IMU_STEREO)
        {
            int colBegin = readParameter<int>(storage_in,
                                              "Camera1.overlappingBegin",
                                              found);
            int colEnd =
                readParameter<int>(storage_in, "Camera1.overlappingEnd", found);
            std::vector<int> vOverlapping = {colBegin, colEnd};
            static_cast<camera_models::KannalaBrandt8 *>(calibration1)
                ->lappingArea = vOverlapping;
        }
    }
    else
    {
        VSLAM_LOG_ERROR("[Settings] Could not find Camera#1 settings for '%s'! "
                        "Exiting ...\n",
                        cameraModelName.c_str());
        exit(-1);
    }
}

void Settings::readCamera2(cv::FileStorage &storage_in)
{
    bool          found;
    vector<float> vCalibration;
    if (cameraModel == CameraType::PINHOLE)
    {
        rectifyNeeded = true;

        // Read intrinsic parameters
        float fx = readParameter<float>(storage_in, "Camera2.fx", found);
        float fy = readParameter<float>(storage_in, "Camera2.fy", found);
        float cx = readParameter<float>(storage_in, "Camera2.cx", found);
        float cy = readParameter<float>(storage_in, "Camera2.cy", found);

        vCalibration = {fx, fy, cx, cy};

        calibration2         = new camera_models::Pinhole(vCalibration);
        originalCalibration2 = new camera_models::Pinhole(vCalibration);

        // Check if it is a distorted Pinhole
        readParameter<float>(storage_in, "Camera2.k1", found, false);
        if (found)
        {
            readParameter<float>(storage_in, "Camera2.k3", found, false);
            if (found)
            {
                pinholeDistortion2.resize(5);
                pinholeDistortion2[4] =
                    readParameter<float>(storage_in, "Camera2.k3", found);
            }
            else
            {
                pinholeDistortion2.resize(4);
            }
            pinholeDistortion2[0] =
                readParameter<float>(storage_in, "Camera2.k1", found);
            pinholeDistortion2[1] =
                readParameter<float>(storage_in, "Camera2.k2", found);
            pinholeDistortion2[2] =
                readParameter<float>(storage_in, "Camera2.p1", found);
            pinholeDistortion2[3] =
                readParameter<float>(storage_in, "Camera2.p2", found);
        }
    }
    else if (cameraModel == CameraType::KANNALA_BRANDT)
    {
        // Read intrinsic parameters
        float fx = readParameter<float>(storage_in, "Camera2.fx", found);
        float fy = readParameter<float>(storage_in, "Camera2.fy", found);
        float cx = readParameter<float>(storage_in, "Camera2.cx", found);
        float cy = readParameter<float>(storage_in, "Camera2.cy", found);

        float k0 = readParameter<float>(storage_in, "Camera2.k1", found);
        float k1 = readParameter<float>(storage_in, "Camera2.k2", found);
        float k2 = readParameter<float>(storage_in, "Camera2.k3", found);
        float k3 = readParameter<float>(storage_in, "Camera2.k4", found);

        vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

        calibration2         = new camera_models::KannalaBrandt8(vCalibration);
        originalCalibration2 = new camera_models::KannalaBrandt8(vCalibration);

        int colBegin =
            readParameter<int>(storage_in, "Camera2.overlappingBegin", found);
        int colEnd =
            readParameter<int>(storage_in, "Camera2.overlappingEnd", found);
        vector<int> vOverlapping = {colBegin, colEnd};

        static_cast<camera_models::KannalaBrandt8 *>(calibration2)
            ->lappingArea = vOverlapping;
    }

    // Load stereo extrinsic calibration
    if (cameraModel == CameraType::RECTIFIED)
    {
        stereoBaseline = readParameter<float>(storage_in, "Stereo.b", found);
        baselineFocal  = stereoBaseline * calibration1->getParameter(0);
    }
    else
    {
        cv::Mat cvTlr =
            readParameter<cv::Mat>(storage_in, "Stereo.T_c1_c2", found);
        stereoTransform = Converter::toSophus(cvTlr);

        // TODO: also search for Trl and invert if necessary

        stereoBaseline = stereoTransform.translation().norm();
        baselineFocal  = stereoBaseline * calibration1->getParameter(0);
    }

    depthThreshold = readParameter<float>(storage_in, "Stereo.ThDepth", found);
}

void Settings::readImageInfo(cv::FileStorage &storage_in)
{
    bool found;
    // Read original and desired image dimensions
    int  originalRows = readParameter<int>(storage_in, "Camera.height", found);
    int  originalCols = readParameter<int>(storage_in, "Camera.width", found);
    originalImageSize.width  = originalCols;
    originalImageSize.height = originalRows;

    newImageSize = originalImageSize;
    int newHeigh =
        readParameter<int>(storage_in, "Camera.newHeight", found, false);
    if (found)
    {
        resize1Needed       = true;
        newImageSize.height = newHeigh;

        if (!rectifyNeeded)
        {
            // Update calibration
            float scaleRowFactor =
                (float)newImageSize.height / (float)originalImageSize.height;
            calibration1->setParameter(calibration1->getParameter(1) *
                                           scaleRowFactor,
                                       1);
            calibration1->setParameter(calibration1->getParameter(3) *
                                           scaleRowFactor,
                                       3);

            if ((sensor == System::STEREO || sensor == System::IMU_STEREO) &&
                cameraModel != CameraType::RECTIFIED)
            {
                calibration2->setParameter(calibration2->getParameter(1) *
                                               scaleRowFactor,
                                           1);
                calibration2->setParameter(calibration2->getParameter(3) *
                                               scaleRowFactor,
                                           3);
            }
        }
    }

    int newWidth =
        readParameter<int>(storage_in, "Camera.newWidth", found, false);
    if (found)
    {
        resize1Needed      = true;
        newImageSize.width = newWidth;

        if (!rectifyNeeded)
        {
            // Update calibration
            float scaleColFactor =
                (float)newImageSize.width / (float)originalImageSize.width;
            calibration1->setParameter(calibration1->getParameter(0) *
                                           scaleColFactor,
                                       0);
            calibration1->setParameter(calibration1->getParameter(2) *
                                           scaleColFactor,
                                       2);

            if ((sensor == System::STEREO || sensor == System::IMU_STEREO) &&
                cameraModel != CameraType::RECTIFIED)
            {
                calibration2->setParameter(calibration2->getParameter(0) *
                                               scaleColFactor,
                                           0);
                calibration2->setParameter(calibration2->getParameter(2) *
                                               scaleColFactor,
                                           2);

                if (cameraModel == CameraType::KANNALA_BRANDT)
                {
                    static_cast<camera_models::KannalaBrandt8 *>(calibration1)
                        ->lappingArea[0] *= scaleColFactor;
                    static_cast<camera_models::KannalaBrandt8 *>(calibration1)
                        ->lappingArea[1] *= scaleColFactor;

                    static_cast<camera_models::KannalaBrandt8 *>(calibration2)
                        ->lappingArea[0] *= scaleColFactor;
                    static_cast<camera_models::KannalaBrandt8 *>(calibration2)
                        ->lappingArea[1] *= scaleColFactor;
                }
            }
        }
    }

    framesPerSecond = readParameter<int>(storage_in, "Camera.fps", found);
    rgbEnabled      = (bool)readParameter<int>(storage_in, "Camera.RGB", found);
}

void Settings::readIMU(cv::FileStorage &storage_in)
{
    bool found;
    accelWalkNoise = readParameter<float>(storage_in, "IMU.AccWalk", found);
    accelNoise     = readParameter<float>(storage_in, "IMU.NoiseAcc", found);
    gyroWalkNoise  = readParameter<float>(storage_in, "IMU.GyroWalk", found);
    gyroNoise      = readParameter<float>(storage_in, "IMU.NoiseGyro", found);
    imuErrorThreshold =
        readParameter<float>(storage_in, "IMU.Threshold", found);
    imuSampleRate = readParameter<float>(storage_in, "IMU.Frequency", found);

    cv::Mat cvTbc = readParameter<cv::Mat>(storage_in, "IMU.T_b_c1", found);
    bodyToCamera  = Converter::toSophus(cvTbc);

    readParameter<int>(storage_in, "IMU.InsertKFsWhenLost", found, false);
    if (found)
        insertKeyframesWhenLost =
            (bool)readParameter<int>(storage_in,
                                     "IMU.InsertKFsWhenLost",
                                     found,
                                     false);
    else
        insertKeyframesWhenLost = true;

    fastInitEnabled =
        readParameter<int>(storage_in, "IMU.FastInit", found, false) != 0;
}

void Settings::readRGBD(cv::FileStorage &storage_in)
{
    bool found;

    depthMapScale =
        readParameter<float>(storage_in, "RGBD.DepthMapFactor", found);
    depthThreshold = readParameter<float>(storage_in, "Stereo.ThDepth", found);
    stereoBaseline = readParameter<float>(storage_in, "Stereo.b", found);
    baselineFocal  = stereoBaseline * calibration1->getParameter(0);
    nearThreshold  = readParameter<float>(storage_in, "RGBD.NearThresh", found);
    farThreshold   = readParameter<float>(storage_in, "RGBD.FarThresh", found);

    // set distance threshold in the system params
    types::SystemParams::getParams()->pointcloud.distanceThresh =
        std::make_pair(nearThreshold, farThreshold);
}

void Settings::readORB(cv::FileStorage &storage_in)
{
    bool found;

    featureCount =
        readParameter<int>(storage_in, "ORBextractor.nFeatures", found);
    orbScaleFactor =
        readParameter<float>(storage_in, "ORBextractor.scaleFactor", found);
    pyramidLevels =
        readParameter<int>(storage_in, "ORBextractor.nLevels", found);
    initialFastThreshold =
        readParameter<int>(storage_in, "ORBextractor.iniThFAST", found);
    minimumFastThreshold =
        readParameter<int>(storage_in, "ORBextractor.minThFAST", found);
}

void Settings::readViewer(cv::FileStorage &storage_in)
{
    bool found;

    viewerKeyFrameSize =
        readParameter<float>(storage_in, "Viewer.KeyFrameSize", found);
    viewerKeyFrameLineWidth =
        readParameter<float>(storage_in, "Viewer.KeyFrameLineWidth", found);
    viewerGraphLineWidth =
        readParameter<float>(storage_in, "Viewer.GraphLineWidth", found);
    viewerPointSize =
        readParameter<float>(storage_in, "Viewer.PointSize", found);
    viewerCameraSize =
        readParameter<float>(storage_in, "Viewer.CameraSize", found);
    viewerCameraLineWidth =
        readParameter<float>(storage_in, "Viewer.CameraLineWidth", found);
    viewerViewPointX =
        readParameter<float>(storage_in, "Viewer.ViewpointX", found);
    viewerViewPointY =
        readParameter<float>(storage_in, "Viewer.ViewpointY", found);
    viewerViewPointZ =
        readParameter<float>(storage_in, "Viewer.ViewpointZ", found);
    viewerViewPointF =
        readParameter<float>(storage_in, "Viewer.ViewpointF", found);
    viewerImageScale =
        readParameter<float>(storage_in, "Viewer.imageViewScale", found, false);

    if (!found)
        viewerImageScale = 1.0f;
}

void Settings::readLoadAndSave(cv::FileStorage &storage_in)
{
    bool found;

    atlasLoadPath = readParameter<string>(storage_in,
                                          "System.LoadAtlasFromFile",
                                          found,
                                          false);
    atlasSavePath = readParameter<string>(storage_in,
                                          "System.SaveAtlasToFile",
                                          found,
                                          false);
}

void Settings::readOtherParameters(cv::FileStorage &storage_in)
{
    bool found;

    farPointsThreshold =
        readParameter<float>(storage_in, "System.thFarPoints", found, false);
}

void Settings::precomputeRectificationMaps()
{
    // Precompute rectification maps, new calibrations, ...
    cv::Mat K1 = static_cast<camera_models::Pinhole *>(calibration1)->toK();
    K1.convertTo(K1, CV_64F);
    cv::Mat K2 = static_cast<camera_models::Pinhole *>(calibration2)->toK();
    K2.convertTo(K2, CV_64F);

    cv::Mat cvTlr;
    cv::eigen2cv(stereoTransform.inverse().matrix3x4(), cvTlr);
    cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
    R12.convertTo(R12, CV_64F);
    cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
    t12.convertTo(t12, CV_64F);

    cv::Mat R_r1_u1, R_r2_u2;
    cv::Mat P1, P2, Q;

    cv::stereoRectify(K1,
                      camera1DistortionCoef(),
                      K2,
                      camera2DistortionCoef(),
                      newImageSize,
                      R12,
                      t12,
                      R_r1_u1,
                      R_r2_u2,
                      P1,
                      P2,
                      Q,
                      cv::CALIB_ZERO_DISPARITY,
                      -1,
                      newImageSize);
    cv::initUndistortRectifyMap(K1,
                                camera1DistortionCoef(),
                                R_r1_u1,
                                P1.rowRange(0, 3).colRange(0, 3),
                                newImageSize,
                                CV_32F,
                                rectifyMap1Left,
                                rectifyMap2Left);
    cv::initUndistortRectifyMap(K2,
                                camera2DistortionCoef(),
                                R_r2_u2,
                                P2.rowRange(0, 3).colRange(0, 3),
                                newImageSize,
                                CV_32F,
                                rectifyMap1Right,
                                rectifyMap2Right);

    // Update calibration
    calibration1->setParameter(P1.at<double>(0, 0), 0);
    calibration1->setParameter(P1.at<double>(1, 1), 1);
    calibration1->setParameter(P1.at<double>(0, 2), 2);
    calibration1->setParameter(P1.at<double>(1, 2), 3);

    calibration2->setParameter(P2.at<double>(0, 0), 0);
    calibration2->setParameter(P2.at<double>(1, 1), 1);
    calibration2->setParameter(P2.at<double>(0, 2), 2);
    calibration2->setParameter(P2.at<double>(1, 2), 3);

    // Update bf
    baselineFocal = stereoBaseline * P1.at<double>(0, 0);

    // Update relative pose between camera 1 and IMU if necessary
    if (sensor == System::IMU_STEREO)
    {
        Eigen::Matrix3f eigenR_r1_u1;
        cv::cv2eigen(R_r1_u1, eigenR_r1_u1);
        Sophus::SE3f T_r1_u1(eigenR_r1_u1, Eigen::Vector3f::Zero());
        bodyToCamera = bodyToCamera * T_r1_u1.inverse();
    }
}

std::ostream &operator<<(std::ostream &output, const Settings &settings)
{
    // Camera#1
    output << "\t- Camera#1 parameters (";
    if (settings.cameraModel == Settings::CameraType::PINHOLE ||
        settings.cameraModel == Settings::CameraType::RECTIFIED)
        output << "camera_models::Pinhole";
    else
        output << "Kannala-Brandt";
    output << "): [";
    for (size_t i = 0; i < settings.originalCalibration1->size(); i++)
        output << " " << settings.originalCalibration1->getParameter(i);
    output << " ]" << endl;

    if (!settings.pinholeDistortion1.empty())
    {
        output << "\t- Camera#1 distortion parameters: [ ";
        for (float d : settings.pinholeDistortion1)
            output << " " << d;
        output << " ]" << endl;
    }

    if ((settings.sensor == System::STEREO ||
         settings.sensor == System::IMU_STEREO) &&
        (settings.cameraModel != Settings::CameraType::RECTIFIED))
    {
        output << "\t- Camera#2 parameters (";
        if (settings.cameraModel == Settings::CameraType::PINHOLE)
            output << "camera_models::Pinhole";
        else
            output << "Kannala-Brandt";
        output << "): [";
        for (size_t i = 0; i < settings.originalCalibration2->size(); i++)
            output << " " << settings.originalCalibration2->getParameter(i);
        output << " ]" << endl;

        if (!settings.pinholeDistortion2.empty())
        {
            output << "\t- Camera#2 distortion parameters: [ ";
            for (float d : settings.pinholeDistortion2)
                output << " " << d;
            output << " ]" << endl;
        }
    }

    output << "\t- Original frame size: [ " << settings.originalImageSize.width
           << "," << settings.originalImageSize.height << " ]" << endl;
    output << "\t- Current frame size: [ " << settings.newImageSize.width << ","
           << settings.newImageSize.height << " ]" << endl;

    if (settings.rectifyNeeded)
    {
        output << "\t- Camera#1 parameters after rectification: [";
        for (size_t i = 0; i < settings.calibration1->size(); i++)
            output << " " << settings.calibration1->getParameter(i);
        output << " ]" << endl;

        if (settings.sensor == System::STEREO ||
            settings.sensor == System::IMU_STEREO)
        {
            output << "\t- Camera#2 parameters after rectification: [";
            for (size_t i = 0; i < settings.calibration2->size(); i++)
                output << " " << settings.calibration2->getParameter(i);
            output << " ]" << endl;
        }
    }
    else if (settings.resize1Needed)
    {
        output << "\t- Camera#1 parameters after resize: [";
        for (size_t i = 0; i < settings.calibration1->size(); i++)
            output << " " << settings.calibration1->getParameter(i);
        output << " ]" << endl;

        if ((settings.sensor == System::STEREO ||
             settings.sensor == System::IMU_STEREO) &&
            settings.cameraModel == Settings::CameraType::KANNALA_BRANDT)
        {
            output << "\t- Camera#2 parameters after resize: [";
            for (size_t i = 0; i < settings.calibration2->size(); i++)
                output << " " << settings.calibration2->getParameter(i);
            output << " ]" << endl;
        }
    }

    // Frame rate
    output << "\t- Sequence FPS: " << settings.framesPerSecond << endl;

    // Stereo stuff
    if (settings.sensor == System::STEREO ||
        settings.sensor == System::IMU_STEREO)
    {
        output << "\t- Stereo baseline: " << settings.stereoBaseline << endl;
        output << "\t- Stereo depth threshold : " << settings.depthThreshold
               << endl;

        if (settings.cameraModel == Settings::CameraType::KANNALA_BRANDT)
        {
            auto vOverlapping1 = static_cast<camera_models::KannalaBrandt8 *>(
                                     settings.calibration1)
                                     ->lappingArea;
            auto vOverlapping2 = static_cast<camera_models::KannalaBrandt8 *>(
                                     settings.calibration2)
                                     ->lappingArea;
            output << "\t- Camera 1 overlapping area: [ " << vOverlapping1[0]
                   << " , " << vOverlapping1[1] << " ]" << endl;
            output << "\t- Camera 2 overlapping area: [ " << vOverlapping2[0]
                   << " , " << vOverlapping2[1] << " ]" << endl;
        }
    }

    // IMU parameters
    if (settings.sensor == System::IMU_MONOCULAR ||
        settings.sensor == System::IMU_STEREO ||
        settings.sensor == System::IMU_RGBD)
    {
        output << "\t- Gyro noise: " << settings.gyroNoise << endl;
        output << "\t- Accelerometer noise: " << settings.accelNoise << endl;
        output << "\t- Gyro walk: " << settings.gyroWalkNoise << endl;
        output << "\t- Accelerometer walk: " << settings.accelWalkNoise << endl;
        output << "\t- IMU frequency: " << settings.imuSampleRate << endl;
        output << "\t- IMU threshold: " << settings.imuErrorThreshold << endl;
    }

    // RGB-D parameters
    if (settings.sensor == System::RGBD || settings.sensor == System::IMU_RGBD)
    {
        output << "\t- RGB-D depth map factor: " << settings.depthMapScale
               << endl;
        output << "\t- Stereo depth threshold: " << settings.depthThreshold
               << endl;
        output << "\t- Metric close depth: "
               << settings.stereoBaseline * settings.depthThreshold << endl;
    }

    // ORB parameters
    output << "\t- Features per image: " << settings.featureCount << endl;
    output << "\t- ORB scale factor: " << settings.orbScaleFactor << endl;
    output << "\t- ORB number of scales: " << settings.pyramidLevels << endl;
    output << "\t- Initial FAST threshold: " << settings.initialFastThreshold
           << endl;
    output << "\t- Min FAST threshold: " << settings.minimumFastThreshold
           << endl;

    return output;
}
} // namespace core
} // namespace vs_graphs
