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
 * @file         Settings.h
 *
 * @brief        Declares the estimator settings loaded from file.
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

/*!
 * @brief        Estimator settings loaded once from a configuration
 *               file.
 */
class Settings
{
  public:
    /*!
     * @brief        Camera types implemented by the estimator.
     */
    enum class CameraType : std::uint8_t
    {
        /*!
         * @brief        Pinhole camera model.
         */
        PINHOLE = 0U,
        /*!
         * @brief        Rectified stereo camera model.
         */
        RECTIFIED = 1U,
        /*!
         * @brief        Kannala-Brandt fisheye camera model.
         */
        KANNALA_BRANDT = 2U
    };

    /*!
     * @brief        Deleted default constructor; settings always
     *               come from a file.
     */
    Settings() = delete;

    /*!
     * @brief        Loads every setting section from the given
     *               file.
     *
     *              Terminates the process when the file cannot be
     *              opened.
     *
     * @param[in]    configFilePath_in
     *               Path of the configuration file to load.
     * @param[in]    sensor_in
     *               Sensor type; selects stereo-only sections.
     */
    Settings(const std::string &configFilePath_in, const int &sensor_in);

    /*!
     * @brief        Appends the settings dump to the stream.
     *
     * @param[in,out] output
     *                Stream receiving the dump.
     * @param[in]    s
     *               Settings to dump.
     *
     * @return       The output stream.
     */
    friend std::ostream &operator<<(std::ostream &output, const Settings &s);

    /*!
     * @brief        Returns the configured camera type.
     *
     * @return       Active camera model tag.
     */
    CameraType cameraType()
    {
        return cameraModel;
    }
    /*!
     * @brief        Returns the first calibrated camera.
     *
     * @return       Non-owning pointer to the first calibration.
     */
    camera_models::GeometricCamera *camera1()
    {
        return calibration1;
    }
    /*!
     * @brief        Returns the second calibrated camera.
     *
     * @return       Non-owning pointer to the second calibration.
     */
    camera_models::GeometricCamera *camera2()
    {
        return calibration2;
    }
    /*!
     * @brief        Returns the first-camera distortion header.
     *
     * @return       Matrix header over the stored coefficients;
     *               valid while the settings live.
     */
    cv::Mat camera1DistortionCoef()
    {
        return cv::Mat(pinholeDistortion1.size(),
                       1,
                       CV_32F,
                       pinholeDistortion1.data());
    }
    /*!
     * @brief        Returns the second-camera distortion header.
     *
     * @return       Matrix header over the stored coefficients;
     *               valid while the settings live.
     */
    cv::Mat camera2DistortionCoef()
    {
        return cv::Mat(pinholeDistortion2.size(),
                       1,
                       CV_32F,
                       pinholeDistortion2.data());
    }

    /*!
     * @brief        Returns the left-to-right stereo transform.
     *
     * @return       Stereo extrinsic in single precision.
     */
    Sophus::SE3f getLeftToRightTransform()
    {
        return stereoTransform;
    }
    /*!
     * @brief        Returns the baseline-focal product.
     *
     * @return       Product in pixel-metres.
     */
    double getBaselineFocal()
    {
        return baselineFocal;
    }
    /*!
     * @brief        Returns the stereo baseline.
     *
     * @return       Baseline in metres.
     */
    double b()
    {
        return stereoBaseline;
    }
    /*!
     * @brief        Returns the close-depth threshold.
     *
     * @return       Threshold in metres.
     */
    double thDepth()
    {
        return depthThreshold;
    }

    /*!
     * @brief        Reports whether frames need undistortion.
     *
     * @return       True when undistortion maps were precomputed.
     */
    bool needToUndistort()
    {
        return undistortNeeded;
    }

    /*!
     * @brief        Returns the undistorted image size.
     *
     * @return       Target size in pixels.
     */
    cv::Size newImSize()
    {
        return newImageSize;
    }
    /*!
     * @brief        Returns the camera frame rate.
     *
     * @return       Frames per second in hertz.
     */
    double getFramesPerSecond()
    {
        return framesPerSecond;
    }
    /*!
     * @brief        Reports whether the image stream carries colour.
     *
     * @return       True for RGB input.
     */
    bool isRgbEnabled()
    {
        return rgbEnabled;
    }
    /*!
     * @brief        Reports whether frames need resizing.
     *
     * @return       True when a resize step is configured.
     */
    bool needToResize()
    {
        return resize1Needed;
    }
    /*!
     * @brief        Reports whether frames need rectification.
     *
     * @return       True when rectification maps were precomputed.
     */
    bool needToRectify()
    {
        return rectifyNeeded;
    }

    // IMU parameters
    /*!
     * @brief        Returns the accelerometer random-walk density.
     *
     * @return       Configured walk noise.
     */
    double accWalk()
    {
        return accelWalkNoise;
    }
    /*!
     * @brief        Returns the gyroscope random-walk density.
     *
     * @return       Configured walk noise.
     */
    double gyroWalk()
    {
        return gyroWalkNoise;
    }
    /*!
     * @brief        Returns the accelerometer noise density.
     *
     * @return       Configured measurement noise.
     */
    double noiseAcc()
    {
        return accelNoise;
    }
    /*!
     * @brief        Returns the IMU sample rate.
     *
     * @return       Samples per second in hertz.
     */
    double imuFrequency()
    {
        return imuSampleRate;
    }
    /*!
     * @brief        Returns the IMU error acceptance threshold.
     *
     * @return       Configured threshold.
     */
    double imuThreshold()
    {
        return imuErrorThreshold;
    }
    /*!
     * @brief        Returns the gyroscope noise density.
     *
     * @return       Configured measurement noise.
     */
    double noiseGyro()
    {
        return gyroNoise;
    }
    /*!
     * @brief        Returns the body-to-camera transform.
     *
     * @return       Extrinsic in single precision.
     */
    Sophus::SE3f Tbc()
    {
        return bodyToCamera;
    }
    /*!
     * @brief        Reports whether keyframes are inserted while
     *               lost.
     *
     * @return       True when insertion while lost is enabled.
     */
    bool insertKFsWhenLost()
    {
        return insertKeyframesWhenLost;
    }
    /*!
     * @brief        Reports whether fast IMU initialization is
     *               enabled.
     *
     * @return       True when fast initialization is enabled.
     */
    bool fastInit() const
    {
        return fastInitEnabled;
    }

    /*!
     * @brief        Returns the depth-map scale factor.
     *
     * @return       Raw values are divided by this factor to reach
     *               metres.
     */
    double depthMapFactor()
    {
        return depthMapScale;
    }

    /*!
     * @brief        Returns the ORB feature budget.
     *
     * @return       Target number of features.
     */
    int nFeatures()
    {
        return featureCount;
    }
    /*!
     * @brief        Returns the ORB pyramid depth.
     *
     * @return       Configured level count.
     */
    int nLevels()
    {
        return pyramidLevels;
    }
    /*!
     * @brief        Returns the initial FAST threshold.
     *
     * @return       Configured extraction threshold.
     */
    double initThFAST()
    {
        return initialFastThreshold;
    }
    /*!
     * @brief        Returns the minimum FAST threshold.
     *
     * @return       Configured retry threshold.
     */
    double getMinimumFastThreshold()
    {
        return minimumFastThreshold;
    }
    /*!
     * @brief        Returns the ORB scale step.
     *
     * @return       Configured pyramid scale factor.
     */
    double scaleFactor()
    {
        return orbScaleFactor;
    }

    /*!
     * @brief        Returns the viewer keyframe size.
     *
     * @return       Configured marker size.
     */
    double keyFrameSize()
    {
        return viewerKeyFrameSize;
    }
    /*!
     * @brief        Returns the viewer keyframe line width.
     *
     * @return       Configured line width.
     */
    double keyFrameLineWidth()
    {
        return viewerKeyFrameLineWidth;
    }
    /*!
     * @brief        Returns the viewer graph line width.
     *
     * @return       Configured line width.
     */
    double graphLineWidth()
    {
        return viewerGraphLineWidth;
    }
    /*!
     * @brief        Returns the viewer point size.
     *
     * @return       Configured marker size.
     */
    double pointSize()
    {
        return viewerPointSize;
    }
    /*!
     * @brief        Returns the viewer camera size.
     *
     * @return       Configured marker size.
     */
    double cameraSize()
    {
        return viewerCameraSize;
    }
    /*!
     * @brief        Returns the viewer camera line width.
     *
     * @return       Configured line width.
     */
    double cameraLineWidth()
    {
        return viewerCameraLineWidth;
    }
    /*!
     * @brief        Returns the viewer viewpoint x coordinate.
     *
     * @return       Configured coordinate.
     */
    double viewPointX()
    {
        return viewerViewPointX;
    }
    /*!
     * @brief        Returns the viewer viewpoint y coordinate.
     *
     * @return       Configured coordinate.
     */
    double viewPointY()
    {
        return viewerViewPointY;
    }
    /*!
     * @brief        Returns the viewer viewpoint z coordinate.
     *
     * @return       Configured coordinate.
     */
    double viewPointZ()
    {
        return viewerViewPointZ;
    }
    /*!
     * @brief        Returns the viewer viewpoint focal length.
     *
     * @return       Configured focal value.
     */
    double viewPointF()
    {
        return viewerViewPointF;
    }
    /*!
     * @brief        Returns the image viewer scale.
     *
     * @return       Configured display scale.
     */
    double imageViewerScale()
    {
        return viewerImageScale;
    }

    /*!
     * @brief        Returns the atlas load path.
     *
     * @return       Configured file path, possibly empty.
     */
    std::string atlasLoadFile()
    {
        return atlasLoadPath;
    }
    /*!
     * @brief        Returns the atlas save path.
     *
     * @return       Configured file path, possibly empty.
     */
    std::string atlasSaveFile()
    {
        return atlasSavePath;
    }

    /*!
     * @brief        Returns the far-point threshold.
     *
     * @return       Threshold in metres.
     */
    double thFarPoints()
    {
        return farPointsThreshold;
    }

    /*!
     * @brief        Returns the left x-rectification map.
     *
     * @return       Stored map sharing its pixel data.
     */
    cv::Mat M1l()
    {
        return rectifyMap1Left;
    }
    /*!
     * @brief        Returns the left y-rectification map.
     *
     * @return       Stored map sharing its pixel data.
     */
    cv::Mat M2l()
    {
        return rectifyMap2Left;
    }
    /*!
     * @brief        Returns the right x-rectification map.
     *
     * @return       Stored map sharing its pixel data.
     */
    cv::Mat M1r()
    {
        return rectifyMap1Right;
    }
    /*!
     * @brief        Returns the right y-rectification map.
     *
     * @return       Stored map sharing its pixel data.
     */
    cv::Mat M2r()
    {
        return rectifyMap2Right;
    }

  private:
    /*!
     * @brief        Reads one typed parameter from file storage.
     *
     *              Terminates the process when a required parameter
     *              is missing.
     *
     * @param[in]    storage_in
     *               Open storage holding the parameters.
     * @param[in]    name_in
     *               Parameter name to read.
     * @param[out]   found_out
     *               True when the parameter exists.
     * @param[in]    required_in
     *               True to require the parameter.
     *
     * @return       Parameter value, or a default value when an
     *               optional parameter is missing.
     */
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

    /*!
     * @brief        Reads the first-camera section.
     *
     * @param[in]    storage_in
     *               Open storage holding the parameters.
     */
    void readCamera1(cv::FileStorage &storage_in);
    /*!
     * @brief        Reads the second-camera section.
     *
     * @param[in]    storage_in
     *               Open storage holding the parameters.
     */
    void readCamera2(cv::FileStorage &storage_in);
    /*!
     * @brief        Reads the image-size and frame-rate section.
     *
     * @param[in]    storage_in
     *               Open storage holding the parameters.
     */
    void readImageInfo(cv::FileStorage &storage_in);
    /*!
     * @brief        Reads the inertial section.
     *
     * @param[in]    storage_in
     *               Open storage holding the parameters.
     */
    void readIMU(cv::FileStorage &storage_in);
    /*!
     * @brief        Reads the RGB-D section.
     *
     * @param[in]    storage_in
     *               Open storage holding the parameters.
     */
    void readRGBD(cv::FileStorage &storage_in);
    /*!
     * @brief        Reads the ORB extractor section.
     *
     * @param[in]    storage_in
     *               Open storage holding the parameters.
     */
    void readORB(cv::FileStorage &storage_in);
    /*!
     * @brief        Reads the viewer section.
     *
     * @param[in]    storage_in
     *               Open storage holding the parameters.
     */
    void readViewer(cv::FileStorage &storage_in);
    /*!
     * @brief        Reads the map load and save section.
     *
     * @param[in]    storage_in
     *               Open storage holding the parameters.
     */
    void readLoadAndSave(cv::FileStorage &storage_in);
    /*!
     * @brief        Reads the remaining miscellaneous parameters.
     *
     * @param[in]    storage_in
     *               Open storage holding the parameters.
     */
    void readOtherParameters(cv::FileStorage &storage_in);

    /*!
     * @brief        Precomputes the undistortion and rectification
     *               maps.
     */
    void precomputeRectificationMaps();

    /*!
     * @brief        Sensor type selecting stereo-only sections.
     */
    int        sensor;
    /*!
     * @brief        Active camera model tag.
     */
    CameraType cameraModel; // Camera type

    /*
     * Visual stuff
     */
    /*!
     * @brief        Owned first and second camera calibrations.
     */
    camera_models::GeometricCamera *calibration1,
        *calibration2; // Camera calibration
    /*!
     * @brief        Owned pre-rectification camera calibrations.
     */
    camera_models::GeometricCamera *originalCalibration1, *originalCalibration2;
    /*!
     * @brief        Pinhole distortion coefficients per camera.
     */
    std::vector<double>             pinholeDistortion1, pinholeDistortion2;

    /*!
     * @brief        Original and undistorted image sizes in pixels.
     */
    cv::Size originalImageSize, newImageSize;
    /*!
     * @brief        Camera frame rate in hertz.
     */
    double   framesPerSecond;
    /*!
     * @brief        True for RGB input.
     */
    bool     rgbEnabled;

    /*!
     * @brief        Processing steps required by the calibration.
     */
    bool undistortNeeded;
    /*!
     * @brief        True when rectification maps were precomputed.
     */
    bool rectifyNeeded;
    /*!
     * @brief        True when a resize step is configured.
     */
    bool resize1Needed, resize2Needed;

    /*!
     * @brief        Left-to-right stereo transform.
     */
    Sophus::SE3f stereoTransform;
    /*!
     * @brief        Close-depth threshold in metres.
     */
    double       depthThreshold;
    /*!
     * @brief        Baseline-focal product and stereo baseline in
     *               metres.
     */
    double       baselineFocal, stereoBaseline;

    /*
     * Rectification stuff
     */
    /*!
     * @brief        Left undistortion and rectification maps.
     */
    cv::Mat rectifyMap1Left, rectifyMap2Left;
    /*!
     * @brief        Right undistortion and rectification maps.
     */
    cv::Mat rectifyMap1Right, rectifyMap2Right;

    /*
     * Inertial stuff
     */
    /*!
     * @brief        Gyroscope and accelerometer noise densities.
     */
    double       gyroNoise, accelNoise;
    /*!
     * @brief        Gyroscope and accelerometer random-walk
     *               densities.
     */
    double       gyroWalkNoise, accelWalkNoise;
    /*!
     * @brief        IMU sample rate in hertz.
     */
    double       imuSampleRate;
    /*!
     * @brief        IMU error acceptance threshold.
     */
    double       imuErrorThreshold;
    /*!
     * @brief        Body-to-camera transform.
     */
    Sophus::SE3f bodyToCamera;
    /*!
     * @brief        True to insert keyframes while lost.
     */
    bool         insertKeyframesWhenLost;
    /*!
     * @brief        True when fast IMU initialization is enabled.
     */
    bool         fastInitEnabled{false};

    /*
     * RGBD stuff
     */
    /*!
     * @brief        Raw depth values are divided by this factor to
     *               reach metres.
     */
    double depthMapScale;
    /*!
     * @brief        Near and far depth limits in metres.
     */
    double nearThreshold, farThreshold;

    /*
     * ORB stuff
     */
    /*!
     * @brief        Target number of ORB features.
     */
    int    featureCount;
    /*!
     * @brief        ORB pyramid scale step.
     */
    double orbScaleFactor;
    /*!
     * @brief        ORB pyramid level count.
     */
    int    pyramidLevels;
    /*!
     * @brief        FAST thresholds used at extraction and retry.
     */
    int    initialFastThreshold, minimumFastThreshold;

    /*
     * Viewer stuff
     */
    /*!
     * @brief        Viewer keyframe marker size.
     */
    double viewerKeyFrameSize;
    /*!
     * @brief        Viewer keyframe line width.
     */
    double viewerKeyFrameLineWidth;
    /*!
     * @brief        Viewer graph line width.
     */
    double viewerGraphLineWidth;
    /*!
     * @brief        Viewer point marker size.
     */
    double viewerPointSize;
    /*!
     * @brief        Viewer camera marker size.
     */
    double viewerCameraSize;
    /*!
     * @brief        Viewer camera line width.
     */
    double viewerCameraLineWidth;
    /*!
     * @brief        Viewer viewpoint coordinates and focal value.
     */
    double viewerViewPointX, viewerViewPointY, viewerViewPointZ,
        viewerViewPointF;
    /*!
     * @brief        Image viewer display scale.
     */
    double viewerImageScale;

    /*
     * Save & load maps
     */
    /*!
     * @brief        Atlas load and save file paths.
     */
    std::string atlasLoadPath, atlasSavePath;

    /*
     * Other stuff
     */
    /*!
     * @brief        Far-point threshold in metres.
     */
    double farPointsThreshold;
};
} // namespace core
} // namespace vs_graphs

#endif // VS_GRAPHS_CORE_SETTINGS_H
