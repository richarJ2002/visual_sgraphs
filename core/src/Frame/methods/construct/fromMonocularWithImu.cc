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

#include "Frame.h"

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "G2oTypes.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "StereoMatchOutlierRejection.h"
#include "Utils/Converter/objects/Converter.h"

#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

/* NOTE: out-of-line to break the Frame<->KeyFrame/MapPoint include cycle. */

Frame::Frame(const cv::Mat                                   &imageColor_in,
             const cv::Mat                                   &imageGray_in,
             const double                                    &timeStamp_in,
             ORBextractor                                    *p_extractor_in,
             ORBVocabulary                                   *p_vocabulary_in,
             camera_models::geometriccamera::GeometricCamera *p_camera_inout,
             cv::Mat                              &distanceCoefficients_in,
             const float                          &bf_in,
             const float                          &thresholdDepth_in,
             Frame                                *p_previousF_in,
             const IMU::Calib                     &imuCalibration_in,
             const std::vector<semantic::Marker *> markers_in) :
    p_poseImuConstraint(nullptr),
    isPoseAvailable(false),
    isVelocityAvailable(false),
    p_orbVocabulary(p_vocabulary_in),
    p_orbExtractorLeft(p_extractor_in),
    p_orbExtractorRight(static_cast<ORBextractor *>(nullptr)),
    timeStamp(timeStamp_in),
    calibrationMatrix(
        static_cast<camera_models::pinhole::Pinhole *>(p_camera_inout)->toK()),
    calibrationMatrixEigen(
        static_cast<camera_models::pinhole::Pinhole *>(p_camera_inout)->toK_()),
    distortionCoefficients(distanceCoefficients_in.clone()),
    mbf(bf_in),
    depthThreshold(thresholdDepth_in),
    imuCalibration(imuCalibration_in),
    p_imuPreintegrated(nullptr),
    p_previousFrame(p_previousF_in),
    p_imuPreintegratedFrame(nullptr),
    p_referenceKeyFrame(static_cast<KeyFrame *>(nullptr)),
    isFrameSet(false),
    hasImuPreintegration(false),
    p_camera(p_camera_inout),
    p_camera2(nullptr)
{
    // Setting the color image for Semantic Segmentation
    colorImg = imageColor_in.clone();

    // Frame ID
    id = nextId++;

    // Scale Level Info
    int orbExtractorLeftLevelCount{};
    if (p_orbExtractorLeft->getLevelCount(orbExtractorLeftLevelCount) !=
        ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLevelCount returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    scaleLevelCount = orbExtractorLeftLevelCount;
    float orbExtractorLeftScaleFactor{};
    if (p_orbExtractorLeft->getScaleFactor(orbExtractorLeftScaleFactor) !=
        ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getScaleFactor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    scaleFactor    = orbExtractorLeftScaleFactor;
    logScaleFactor = log(scaleFactor);
    std::vector<float> orbExtractorLeftScaleFactors{};
    if (p_orbExtractorLeft->getScaleFactors(orbExtractorLeftScaleFactors) !=
        ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getScaleFactors returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    scaleFactors = orbExtractorLeftScaleFactors;
    std::vector<float> orbExtractorLeftInverseScaleFactors{};
    if (p_orbExtractorLeft->getInverseScaleFactors(
            orbExtractorLeftInverseScaleFactors) !=
        ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getInverseScaleFactors returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    invScaleFactors = orbExtractorLeftInverseScaleFactors;
    std::vector<float> orbExtractorLeftScaleSigmaSquares{};
    if (p_orbExtractorLeft->getScaleSigmaSquares(
            orbExtractorLeftScaleSigmaSquares) !=
        ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getScaleSigmaSquares returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    levelSigmaSquared = orbExtractorLeftScaleSigmaSquares;
    std::vector<float> orbExtractorLeftInverseScaleSigmaSquares{};
    if (p_orbExtractorLeft->getInverseScaleSigmaSquares(
            orbExtractorLeftInverseScaleSigmaSquares) !=
        ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getInverseScaleSigmaSquares returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    invLevelSigmaSquared = orbExtractorLeftInverseScaleSigmaSquares;

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeStartExtOrb =
        std::chrono::steady_clock::now();
#endif
    if (extractOrbFeatures(0, imageGray_in, 0, 1000) !=
        FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: extractOrbFeatures returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeEndExtOrb =
        std::chrono::steady_clock::now();

    orbExtractionTime =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            timeEndExtOrb - timeStartExtOrb)
            .count();
#endif

    keyPointCount = keyPoints.size();

    if (keyPoints.empty())
        return;

    if (undistortKeyPoints() != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: undistortKeyPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    // Set no stereo information
    closeMapPointCount = 0;
    depths             = vector<float>(keyPointCount, -1);
    uRight             = vector<float>(keyPointCount, -1);

    // Initialize MapPoints
    mapPoints =
        vector<MapPoint *>(keyPointCount, static_cast<MapPoint *>(nullptr));

    // Initialize MapMarkers
    mapMarkers = markers_in;

    projectedPoints.clear();
    matchedPoints.clear();

    outlierFlags = vector<bool>(keyPointCount, false);

    // This is done only for the first Frame (or after a change in the
    // calibration)
    if (areInitialComputationsDone)
    {
        if (computeImageBounds(imageGray_in) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeImageBounds returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        gridElementWidthInverse = static_cast<float>(FRAME_GRID_COLS) /
                                  static_cast<float>(gridMaxX - gridMinX);
        gridElementHeightInverse = static_cast<float>(FRAME_GRID_ROWS) /
                                   static_cast<float>(gridMaxY - gridMinY);

        fx = static_cast<camera_models::pinhole::Pinhole *>(p_camera)
                 ->toK()
                 .at<float>(0, 0);
        fy = static_cast<camera_models::pinhole::Pinhole *>(p_camera)
                 ->toK()
                 .at<float>(1, 1);
        cx = static_cast<camera_models::pinhole::Pinhole *>(p_camera)
                 ->toK()
                 .at<float>(0, 2);
        cy = static_cast<camera_models::pinhole::Pinhole *>(p_camera)
                 ->toK()
                 .at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        areInitialComputationsDone = false;
    }

    mb = mbf / fx;

    // Set no stereo fisheye information
    leftKeyPointCount  = -1;
    rightKeyPointCount = -1;
    monoLeft           = -1;
    monoRight          = -1;
    leftToRightMatches = vector<int>(0);
    rightToLeftMatches = vector<int>(0);
    stereoPoints3D     = vector<Eigen::Vector3f>(0);

    if (assignFeaturesToGrid() != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: assignFeaturesToGrid returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    if (p_previousF_in)
    {
        bool previousFHasVelocity{};
        if (p_previousF_in->hasVelocity(previousFHasVelocity) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (previousFHasVelocity)
        {
            Eigen::Vector3f previousFGetVelocity{};
            if (p_previousF_in->getVelocity(previousFGetVelocity) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (setVelocity(previousFGetVelocity) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }
    else
    {
        velocityVw.setZero();
    }
}

} // namespace core
} // namespace vs_graphs
