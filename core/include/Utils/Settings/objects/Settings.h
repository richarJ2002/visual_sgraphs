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

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "Utils/Settings/objects/SettingsStatus.h"

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

namespace utils
{
namespace settings
{

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
     * @param[in]    configurationFilePath_in
     *               Path of the configuration file to load.
     * @param[in]    sensor_in
     *               Sensor type; selects stereo-only sections.
     */
    Settings(const std::string &configurationFilePath_in, const int &sensor_in);

    /*!
     * @brief        Appends the settings dump to the stream.
     *
     * @param[in,out] output_inout
     *                Stream receiving the dump.
     * @param[in]    s_in
     *               Settings to dump.
     *
     * @return       The output stream.
     */
    friend std::ostream &operator<<(std::ostream   &output_inout,
                                    const Settings &s_in);

    /*!
     * @brief        Returns the configured camera type.
     *
     * @param[out] cameraType_out Active camera model tag.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus cameraType(CameraType &cameraType_out)
    {
        cameraType_out = cameraModel;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the first calibrated camera.
     *
     * @param[out] p_camera1_out Non-owning pointer to the first calibration.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        camera1(camera_models::geometriccamera::GeometricCamera *&p_camera1_out)
    {
        p_camera1_out = p_calibration1;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the second calibrated camera.
     *
     * @param[out] p_camera2_out Non-owning pointer to the second calibration.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        camera2(camera_models::geometriccamera::GeometricCamera *&p_camera2_out)
    {
        p_camera2_out = p_calibration2;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the first-camera distortion header.
     *
     * @param[out] camera1DistortionCoef_out Matrix header over the stored
     * coefficients; valid while the settings live.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        camera1DistortionCoef(cv::Mat &camera1DistortionCoef_out)
    {
        camera1DistortionCoef_out = cv::Mat(pinholeDistortion1.size(),
                                            1,
                                            CV_32F,
                                            pinholeDistortion1.data());
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the second-camera distortion header.
     *
     * @param[out] camera2DistortionCoef_out Matrix header over the stored
     * coefficients; valid while the settings live.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        camera2DistortionCoef(cv::Mat &camera2DistortionCoef_out)
    {
        camera2DistortionCoef_out = cv::Mat(pinholeDistortion2.size(),
                                            1,
                                            CV_32F,
                                            pinholeDistortion2.data());
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the left-to-right stereo transform.
     *
     * @param[out] leftToRightTransform_out Stereo extrinsic in single
     * precision.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        getLeftToRightTransform(Sophus::SE3f &leftToRightTransform_out)
    {
        leftToRightTransform_out = stereoTransform;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the baseline-focal product.
     *
     * @param[out] baselineFocal_out Product in pixel-metres.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        getBaselineFocal(double &baselineFocal_out) const
    {
        baselineFocal_out = baselineFocal;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the stereo baseline.
     *
     * @param[out] b_out Baseline in metres.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus b(double &b_out) const
    {
        b_out = stereoBaseline;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the close-depth threshold.
     *
     * @param[out] thDepth_out Threshold in metres.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus thDepth(double &thDepth_out) const
    {
        thDepth_out = depthThreshold;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }

    /*!
     * @brief        Reports whether frames need undistortion.
     *
     * @param[out] needToUndistort_out True when undistortion maps were
     * precomputed.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        needToUndistort(bool &needToUndistort_out) const
    {
        needToUndistort_out = isUndistortionNeeded;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the undistorted image size.
     *
     * @param[out] newImSize_out Target size in pixels.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus newImSize(cv::Size &newImSize_out)
    {
        newImSize_out = newImageSize;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the camera frame rate.
     *
     * @param[out] framesPerSecond_out Frames per second in hertz.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        getFramesPerSecond(double &framesPerSecond_out) const
    {
        framesPerSecond_out = framesPerSecond;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Reports whether the image stream carries colour.
     *
     * @param[out] isRgbEnabled_out True for RGB input.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus isRgbEnabled(bool &isRgbEnabled_out) const
    {
        isRgbEnabled_out = isRgbInputEnabled;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Reports whether frames need resizing.
     *
     * @param[out] needToResize_out True when a resize step is configured.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus needToResize(bool &needToResize_out) const
    {
        needToResize_out = isFirstResizeNeeded;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Reports whether frames need rectification.
     *
     * @param[out] needToRectify_out True when rectification maps were
     * precomputed.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus needToRectify(bool &needToRectify_out) const
    {
        needToRectify_out = isRectificationNeeded;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }

    // IMU parameters
    /*!
     * @brief        Returns the accelerometer random-walk density.
     *
     * @param[out] accWalk_out Configured walk noise.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus accWalk(double &accWalk_out) const
    {
        accWalk_out = accelWalkNoise;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the gyroscope random-walk density.
     *
     * @param[out] gyroWalk_out Configured walk noise.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus gyroWalk(double &gyroWalk_out) const
    {
        gyroWalk_out = gyroWalkNoise;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the accelerometer noise density.
     *
     * @param[out] noiseAcc_out Configured measurement noise.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus noiseAcc(double &noiseAcc_out) const
    {
        noiseAcc_out = accelNoise;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the IMU sample rate.
     *
     * @param[out] imuFrequency_out Samples per second in hertz.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus imuFrequency(double &imuFrequency_out) const
    {
        imuFrequency_out = imuSampleRate;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the IMU error acceptance threshold.
     *
     * @param[out] imuThreshold_out Configured threshold.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus imuThreshold(double &imuThreshold_out) const
    {
        imuThreshold_out = imuErrorThreshold;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the gyroscope noise density.
     *
     * @param[out] noiseGyro_out Configured measurement noise.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus noiseGyro(double &noiseGyro_out) const
    {
        noiseGyro_out = gyroNoise;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the camera-to-body (IMU) transform read from
     *               IMU.T_b_c1.
     *
     * @param[out]   extrinsicPose_cameraToBody_out
     *               Extrinsic in single precision.
     *
     * @return       SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        Tbc(Sophus::SE3f &extrinsicPose_cameraToBody_out)
    {
        extrinsicPose_cameraToBody_out = extrinsicPose_cameraToBody;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Reports whether keyframes are inserted while
     *               lost.
     *
     * @param[out] insertKFsWhenLost_out True when insertion while lost is
     * enabled.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        insertKFsWhenLost(bool &insertKFsWhenLost_out) const
    {
        insertKFsWhenLost_out = shouldInsertKeyFramesWhenLost;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Reports whether fast IMU initialization is
     *               enabled.
     *
     * @param[out] fastInit_out True when fast initialization is enabled.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus fastInit(bool &fastInit_out) const
    {
        fastInit_out = isFastInitEnabled;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the depth-map scale factor.
     *
     * @param[out] depthMapFactor_out Raw values are divided by this factor to
     * reach metres.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        depthMapFactor(double &depthMapFactor_out) const
    {
        depthMapFactor_out = depthMapScale;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the ORB feature budget.
     *
     * @param[out] nFeatures_out Target number of features.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus nFeatures(int &nFeatures_out) const
    {
        nFeatures_out = featureCount;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the ORB pyramid depth.
     *
     * @param[out] nLevels_out Configured level count.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus nLevels(int &nLevels_out) const
    {
        nLevels_out = pyramidLevels;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the initial FAST threshold.
     *
     * @param[out] initThFAST_out Configured extraction threshold.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus initThFAST(double &initThFAST_out) const
    {
        initThFAST_out = initialFastThreshold;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the minimum FAST threshold.
     *
     * @param[out] minimumFastThreshold_out Configured retry threshold.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        getMinimumFastThreshold(double &minimumFastThreshold_out) const
    {
        minimumFastThreshold_out = minimumFastThreshold;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the ORB scale step.
     *
     * @param[out] scaleFactor_out Configured pyramid scale factor.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus scaleFactor(double &scaleFactor_out) const
    {
        scaleFactor_out = orbScaleFactor;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the viewer keyframe size.
     *
     * @param[out] keyFrameSize_out Configured marker size.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus keyFrameSize(double &keyFrameSize_out) const
    {
        keyFrameSize_out = viewerKeyFrameSize;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the viewer keyframe line width.
     *
     * @param[out] keyFrameLineWidth_out Configured line width.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        keyFrameLineWidth(double &keyFrameLineWidth_out) const
    {
        keyFrameLineWidth_out = viewerKeyFrameLineWidth;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the viewer graph line width.
     *
     * @param[out] graphLineWidth_out Configured line width.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        graphLineWidth(double &graphLineWidth_out) const
    {
        graphLineWidth_out = viewerGraphLineWidth;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the viewer point size.
     *
     * @param[out] pointSize_out Configured marker size.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus pointSize(double &pointSize_out) const
    {
        pointSize_out = viewerPointSize;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the viewer camera size.
     *
     * @param[out] cameraSize_out Configured marker size.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus cameraSize(double &cameraSize_out) const
    {
        cameraSize_out = viewerCameraSize;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the viewer camera line width.
     *
     * @param[out] cameraLineWidth_out Configured line width.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        cameraLineWidth(double &cameraLineWidth_out) const
    {
        cameraLineWidth_out = viewerCameraLineWidth;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the viewer viewpoint x coordinate.
     *
     * @param[out] viewPointX_out Configured coordinate.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus viewPointX(double &viewPointX_out) const
    {
        viewPointX_out = viewerViewPointX;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the viewer viewpoint y coordinate.
     *
     * @param[out] viewPointY_out Configured coordinate.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus viewPointY(double &viewPointY_out) const
    {
        viewPointY_out = viewerViewPointY;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the viewer viewpoint z coordinate.
     *
     * @param[out] viewPointZ_out Configured coordinate.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus viewPointZ(double &viewPointZ_out) const
    {
        viewPointZ_out = viewerViewPointZ;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the viewer viewpoint focal length.
     *
     * @param[out] viewPointF_out Configured focal value.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus viewPointF(double &viewPointF_out) const
    {
        viewPointF_out = viewerViewPointF;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the image viewer scale.
     *
     * @param[out] imageViewerScale_out Configured display scale.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus
        imageViewerScale(double &imageViewerScale_out) const
    {
        imageViewerScale_out = viewerImageScale;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the atlas load path.
     *
     * @param[out] atlasLoadFile_out Configured file path, possibly empty.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus atlasLoadFile(std::string &atlasLoadFile_out)
    {
        atlasLoadFile_out = atlasLoadPath;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the atlas save path.
     *
     * @param[out] atlasSaveFile_out Configured file path, possibly empty.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus atlasSaveFile(std::string &atlasSaveFile_out)
    {
        atlasSaveFile_out = atlasSavePath;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the far-point threshold.
     *
     * @param[out] thFarPoints_out Threshold in metres.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus thFarPoints(double &thFarPoints_out) const
    {
        thFarPoints_out = farPointsThreshold;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the left x-rectification map.
     *
     * @param[out] M1l_out Stored map sharing its pixel data.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus M1l(cv::Mat &M1l_out)
    {
        M1l_out = rectifyMap1Left;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the left y-rectification map.
     *
     * @param[out] M2l_out Stored map sharing its pixel data.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus M2l(cv::Mat &M2l_out)
    {
        M2l_out = rectifyMap2Left;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the right x-rectification map.
     *
     * @param[out] M1r_out Stored map sharing its pixel data.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus M1r(cv::Mat &M1r_out)
    {
        M1r_out = rectifyMap1Right;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the right y-rectification map.
     *
     * @param[out] M2r_out Stored map sharing its pixel data.
     * @return SETTINGS_STATUS_SUCCESS.
     */
    [[nodiscard]] SettingsStatus M2r(cv::Mat &M2r_out)
    {
        M2r_out = rectifyMap2Right;
        return SettingsStatus::SETTINGS_STATUS_SUCCESS;
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
     * @param[out]   parameter_out
     *               Parameter value, or a default value when an
     *               optional parameter is missing.
     * @param[in]    required_in
     *               True to require the parameter.
     *
     * @return       SETTINGS_STATUS_SUCCESS.
     */
    template <typename T>
    [[nodiscard]] SettingsStatus readParameter(cv::FileStorage   &storage_in,
                                               const std::string &name_in,
                                               bool              &found_out,
                                               T                 &parameter_out,
                                               const bool required_in = true)
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
                found_out     = false;
                parameter_out = T();
                return SettingsStatus::SETTINGS_STATUS_SUCCESS;
            }
        }
        else
        {
            found_out     = true;
            parameter_out = static_cast<T>(node);
            return SettingsStatus::SETTINGS_STATUS_SUCCESS;
        }
    }

    /*!
     * @brief        Reads the first-camera section.
     *
     * @param[in,out] storage_inout
     *               Open storage holding the parameters.
     */
    [[nodiscard]] SettingsStatus readCamera1(cv::FileStorage &storage_inout);
    /*!
     * @brief        Reads the second-camera section.
     *
     * @param[in,out] storage_inout
     *               Open storage holding the parameters.
     */
    [[nodiscard]] SettingsStatus readCamera2(cv::FileStorage &storage_inout);
    /*!
     * @brief        Reads the image-size and frame-rate section.
     *
     * @param[in,out] storage_inout
     *               Open storage holding the parameters.
     */
    [[nodiscard]] SettingsStatus readImageInfo(cv::FileStorage &storage_inout);
    /*!
     * @brief        Reads the inertial section.
     *
     * @param[in,out] storage_inout
     *               Open storage holding the parameters.
     */
    [[nodiscard]] SettingsStatus readIMU(cv::FileStorage &storage_inout);
    /*!
     * @brief        Reads the RGB-D section.
     *
     * @param[in,out] storage_inout
     *               Open storage holding the parameters.
     */
    [[nodiscard]] SettingsStatus readRGBD(cv::FileStorage &storage_inout);
    /*!
     * @brief        Reads the ORB extractor section.
     *
     * @param[in,out] storage_inout
     *               Open storage holding the parameters.
     */
    [[nodiscard]] SettingsStatus readORB(cv::FileStorage &storage_inout);
    /*!
     * @brief        Reads the viewer section.
     *
     * @param[in,out] storage_inout
     *               Open storage holding the parameters.
     */
    [[nodiscard]] SettingsStatus readViewer(cv::FileStorage &storage_inout);
    /*!
     * @brief        Reads the map load and save section.
     *
     * @param[in,out] storage_inout
     *               Open storage holding the parameters.
     */
    [[nodiscard]] SettingsStatus
        readLoadAndSave(cv::FileStorage &storage_inout);
    /*!
     * @brief        Reads the remaining miscellaneous parameters.
     *
     * @param[in,out] storage_inout
     *               Open storage holding the parameters.
     */
    [[nodiscard]] SettingsStatus
        readOtherParameters(cv::FileStorage &storage_inout);

    /*!
     * @brief        Precomputes the undistortion and rectification
     *               maps.
     */
    [[nodiscard]] SettingsStatus precomputeRectificationMaps();

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
    camera_models::geometriccamera::GeometricCamera *p_calibration1,
        *p_calibration2; // Camera calibration
    /*!
     * @brief        Owned pre-rectification camera calibrations.
     */
    camera_models::geometriccamera::GeometricCamera *p_originalCalibration1,
        *p_originalCalibration2;
    /*!
     * @brief        Pinhole distortion coefficients per camera.
     */
    std::vector<double> pinholeDistortion1, pinholeDistortion2;

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
    bool     isRgbInputEnabled;

    /*!
     * @brief        Processing steps required by the calibration.
     */
    bool isUndistortionNeeded;
    /*!
     * @brief        True when rectification maps were precomputed.
     */
    bool isRectificationNeeded;
    /*!
     * @brief        True when a resize step is configured.
     */
    bool isFirstResizeNeeded, isSecondResizeNeeded;

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
     * @brief        Camera-to-body (IMU) transform read from IMU.T_b_c1; for
     *               stereo-inertial it is re-expressed for the rectified
     *               camera 1.
     */
    Sophus::SE3f extrinsicPose_cameraToBody;
    /*!
     * @brief        True to insert keyframes while lost.
     */
    bool         shouldInsertKeyFramesWhenLost;
    /*!
     * @brief        True when fast IMU initialization is enabled.
     */
    bool         isFastInitEnabled{false};

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

/*
 * readParameter() is only ever used with the four types below, and each
 * one is explicitly specialized by its own file under
 * Utils/Settings/methods/readParameter so that the value is decoded and
 * type-checked through the matching cv::FileNode accessor. The
 * specializations are declared here, ahead of every call site, because a
 * translation unit that saw only the generic body would implicitly
 * instantiate it instead.
 */
template <>
[[nodiscard]] SettingsStatus
    Settings::readParameter<float>(cv::FileStorage   &storage_in,
                                   const std::string &name_in,
                                   bool              &found_out,
                                   float             &parameter_out,
                                   const bool         required_in);

template <>
[[nodiscard]] SettingsStatus
    Settings::readParameter<int>(cv::FileStorage   &storage_in,
                                 const std::string &name_in,
                                 bool              &found_out,
                                 int               &parameter_out,
                                 const bool         required_in);

template <>
[[nodiscard]] SettingsStatus
    Settings::readParameter<std::string>(cv::FileStorage   &storage_in,
                                         const std::string &name_in,
                                         bool              &found_out,
                                         std::string       &parameter_out,
                                         const bool         required_in);

template <>
[[nodiscard]] SettingsStatus
    Settings::readParameter<cv::Mat>(cv::FileStorage   &storage_in,
                                     const std::string &name_in,
                                     bool              &found_out,
                                     cv::Mat           &parameter_out,
                                     const bool         required_in);
} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs

#endif // VS_GRAPHS_CORE_SETTINGS_H
