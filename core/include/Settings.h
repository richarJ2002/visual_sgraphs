/**
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

#ifndef VS_GRAPHS_CORE_SETTINGS_H
#define VS_GRAPHS_CORE_SETTINGS_H

// Flag to activate the measurement of time in each process (track,localmap,
// place recognition). #define REGISTER_TIMES

#include "CameraModels/GeometricCamera.h"

#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

namespace vs_graphs
{
namespace core
{

class System;

class Settings
{
  public:
    /*
     * Enum for the different camera types implemented
     */
    enum class CameraType : std::uint8_t
    {
        PINHOLE        = 0U,
        RECTIFIED      = 1U,
        KANNALA_BRANDT = 2U
    };

    /*
     * Delete default constructor
     */
    Settings() = delete;

    /*
     * Constructor from file
     */
    Settings(const std::string &configFilePath_in, const int &sensor_in);

    /*
     * Ostream operator overloading to dump settings to the terminal
     */
    friend std::ostream &operator<<(std::ostream &output, const Settings &s);

    /*
     * Getter methods
     */
    CameraType cameraType()
    {
        return cameraModel;
    }
    camera_models::GeometricCamera *camera1()
    {
        return calibration1;
    }
    camera_models::GeometricCamera *camera2()
    {
        return calibration2;
    }
    cv::Mat camera1DistortionCoef()
    {
        return cv::Mat(pinholeDistortion1.size(),
                       1,
                       CV_32F,
                       pinholeDistortion1.data());
    }
    cv::Mat camera2DistortionCoef()
    {
        return cv::Mat(pinholeDistortion2.size(),
                       1,
                       CV_32F,
                       pinholeDistortion2.data());
    }

    Sophus::SE3f Tlr()
    {
        return stereoTransform;
    }
    double bf()
    {
        return baselineFocal;
    }
    double b()
    {
        return stereoBaseline;
    }
    double thDepth()
    {
        return depthThreshold;
    }

    bool needToUndistort()
    {
        return undistortNeeded;
    }

    cv::Size newImSize()
    {
        return newImageSize;
    }
    double fps()
    {
        return framesPerSecond;
    }
    bool rgb()
    {
        return rgbEnabled;
    }
    bool needToResize()
    {
        return resize1Needed;
    }
    bool needToRectify()
    {
        return rectifyNeeded;
    }

    // IMU parameters
    double accWalk()
    {
        return accelWalkNoise;
    }
    double gyroWalk()
    {
        return gyroWalkNoise;
    }
    double noiseAcc()
    {
        return accelNoise;
    }
    double imuFrequency()
    {
        return imuSampleRate;
    }
    double imuThreshold()
    {
        return imuErrorThreshold;
    }
    double noiseGyro()
    {
        return gyroNoise;
    }
    Sophus::SE3f Tbc()
    {
        return bodyToCamera;
    }
    bool insertKFsWhenLost()
    {
        return insertKeyframesWhenLost;
    }
    bool fastInit() const
    {
        return fastInitEnabled;
    }

    double depthMapFactor()
    {
        return depthMapScale;
    }

    int nFeatures()
    {
        return featureCount;
    }
    int nLevels()
    {
        return pyramidLevels;
    }
    double initThFAST()
    {
        return initialFastThreshold;
    }
    double minThFAST()
    {
        return minimumFastThreshold;
    }
    double scaleFactor()
    {
        return orbScaleFactor;
    }

    double keyFrameSize()
    {
        return viewerKeyFrameSize;
    }
    double keyFrameLineWidth()
    {
        return viewerKeyFrameLineWidth;
    }
    double graphLineWidth()
    {
        return viewerGraphLineWidth;
    }
    double pointSize()
    {
        return viewerPointSize;
    }
    double cameraSize()
    {
        return viewerCameraSize;
    }
    double cameraLineWidth()
    {
        return viewerCameraLineWidth;
    }
    double viewPointX()
    {
        return viewerViewPointX;
    }
    double viewPointY()
    {
        return viewerViewPointY;
    }
    double viewPointZ()
    {
        return viewerViewPointZ;
    }
    double viewPointF()
    {
        return viewerViewPointF;
    }
    double imageViewerScale()
    {
        return viewerImageScale;
    }

    std::string atlasLoadFile()
    {
        return atlasLoadPath;
    }
    std::string atlasSaveFile()
    {
        return atlasSavePath;
    }

    double thFarPoints()
    {
        return farPointsThreshold;
    }

    cv::Mat M1l()
    {
        return rectifyMap1Left;
    }
    cv::Mat M2l()
    {
        return rectifyMap2Left;
    }
    cv::Mat M1r()
    {
        return rectifyMap1Right;
    }
    cv::Mat M2r()
    {
        return rectifyMap2Right;
    }

  private:
    template <typename T>
    T readParameter(cv::FileStorage   &storage_in,
                    const std::string &name_in,
                    bool              &found_out,
                    const bool         required_in = true)
    {
        cv::FileNode node = storage_in[name_in];
        if (node.empty())
        {
            if (required_in)
            {
                std::cerr << name_in
                          << " required parameter does not exist, aborting..."
                          << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name_in << " optional parameter does not exist..."
                          << std::endl;
                found_out = false;
                return T();
            }
        }
        else
        {
            found_out = true;
            return (T)node;
        }
    }

    void readCamera1(cv::FileStorage &storage_in);
    void readCamera2(cv::FileStorage &storage_in);
    void readImageInfo(cv::FileStorage &storage_in);
    void readIMU(cv::FileStorage &storage_in);
    void readRGBD(cv::FileStorage &storage_in);
    void readORB(cv::FileStorage &storage_in);
    void readViewer(cv::FileStorage &storage_in);
    void readLoadAndSave(cv::FileStorage &storage_in);
    void readOtherParameters(cv::FileStorage &storage_in);

    void precomputeRectificationMaps();

    int        sensor;
    CameraType cameraModel; // Camera type

    /*
     * Visual stuff
     */
    camera_models::GeometricCamera    *calibration1, *calibration2; // Camera calibration
    camera_models::GeometricCamera    *originalCalibration1, *originalCalibration2;
    std::vector<double> pinholeDistortion1, pinholeDistortion2;

    cv::Size originalImageSize, newImageSize;
    double   framesPerSecond;
    bool     rgbEnabled;

    bool undistortNeeded;
    bool rectifyNeeded;
    bool resize1Needed, resize2Needed;

    Sophus::SE3f stereoTransform;
    double       depthThreshold;
    double       baselineFocal, stereoBaseline;

    /*
     * Rectification stuff
     */
    cv::Mat rectifyMap1Left, rectifyMap2Left;
    cv::Mat rectifyMap1Right, rectifyMap2Right;

    /*
     * Inertial stuff
     */
    double       gyroNoise, accelNoise;
    double       gyroWalkNoise, accelWalkNoise;
    double       imuSampleRate;
    double       imuErrorThreshold;
    Sophus::SE3f bodyToCamera;
    bool         insertKeyframesWhenLost;
    bool         fastInitEnabled{false};

    /*
     * RGBD stuff
     */
    double depthMapScale;
    double nearThreshold, farThreshold;

    /*
     * ORB stuff
     */
    int    featureCount;
    double orbScaleFactor;
    int    pyramidLevels;
    int    initialFastThreshold, minimumFastThreshold;

    /*
     * Viewer stuff
     */
    double viewerKeyFrameSize;
    double viewerKeyFrameLineWidth;
    double viewerGraphLineWidth;
    double viewerPointSize;
    double viewerCameraSize;
    double viewerCameraLineWidth;
    double viewerViewPointX, viewerViewPointY, viewerViewPointZ, viewerViewPointF;
    double viewerImageScale;

    /*
     * Save & load maps
     */
    std::string atlasLoadPath, atlasSavePath;

    /*
     * Other stuff
     */
    double farPointsThreshold;
};
} // namespace core
} // namespace vs_graphs;

#endif // VS_GRAPHS_CORE_SETTINGS_H
