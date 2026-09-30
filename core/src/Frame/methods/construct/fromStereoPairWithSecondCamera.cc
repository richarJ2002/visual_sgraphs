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
 * @file            fromStereoPairWithSecondCamera.cc
 *
 * @brief           Implements the Frame constructor
 *                  (fromStereoPairWithSecondCamera), declared in Frame.h.
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

Frame::Frame(const cv::Mat &imageColor_in,
             const cv::Mat &imageLeft_in,
             const cv::Mat &imageRight_in,
             const double  &timeStamp_in,
             ORBextractor  *p_extractorLeft_in,
             ORBextractor  *p_extractorRight_in,
             ORBVocabulary *p_vocabulary_in,
             cv::Mat       &K_in,
             cv::Mat       &distanceCoefficients_in,
             const float   &bf_in,
             const float   &thresholdDepth_in,
             camera_models::geometriccamera::GeometricCamera *p_camera_in,
             camera_models::geometriccamera::GeometricCamera *p_camera2_in,
             Sophus::SE3f                                    &Tlr_in,
             Frame                                           *p_previousF_in,
             const IMU::Calib                                &imuCalibration_in,
             const std::vector<semantic::Marker *>            markers_in) :
    p_poseImuConstraint(nullptr),
    isPoseAvailable(false),
    isVelocityAvailable(false),
    p_orbVocabulary(p_vocabulary_in),
    p_orbExtractorLeft(p_extractorLeft_in),
    p_orbExtractorRight(p_extractorRight_in),
    timeStamp(timeStamp_in),
    calibrationMatrix(K_in.clone()),
    distortionCoefficients(distanceCoefficients_in.clone()),
    mbf(bf_in),
    depthThreshold(thresholdDepth_in),
    imuCalibration(imuCalibration_in),
    p_imuPreintegrated(nullptr),
    p_previousFrame(p_previousF_in),
    p_imuPreintegratedFrame(nullptr),
    p_referenceKeyFrame(static_cast<KeyFrame *>(nullptr)),
    hasImuPreintegration(false),
    p_camera(p_camera_in),
    p_camera2(p_camera2_in)

{
    if (utils::converter::Converter::toMatrix3f(K_in, calibrationMatrixEigen) !=
        utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: toMatrix3f returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    imgLeft  = imageLeft_in.clone();
    imgRight = imageRight_in.clone();

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
    logScaleFactor = std::log(scaleFactor);
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
    std::thread threadLeft(
        &Frame::extractOrbFeatures,
        this,
        0,
        imageLeft_in,
        static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(p_camera)
            ->lappingArea[0],
        static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(p_camera)
            ->lappingArea[1]);
    std::thread threadRight(
        &Frame::extractOrbFeatures,
        this,
        1,
        imageRight_in,
        static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(p_camera2)
            ->lappingArea[0],
        static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(p_camera2)
            ->lappingArea[1]);
    threadLeft.join();
    threadRight.join();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeEndExtOrb =
        std::chrono::steady_clock::now();

    orbExtractionTime =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            timeEndExtOrb - timeStartExtOrb)
            .count();
#endif

    leftKeyPointCount  = keyPoints.size();
    rightKeyPointCount = keyPointsRight.size();

    keyPointCount = leftKeyPointCount + rightKeyPointCount;
    if (keyPointCount == 0)
        return;

    // This is done only for the first Frame (or after a change in the
    // calibration)
    if (areInitialComputationsDone)
    {
        if (computeImageBounds(imageLeft_in) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeImageBounds returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        gridElementWidthInverse =
            static_cast<float>(FRAME_GRID_COLS) / (gridMaxX - gridMinX);
        gridElementHeightInverse =
            static_cast<float>(FRAME_GRID_ROWS) / (gridMaxY - gridMinY);

        fx    = K_in.at<float>(0, 0);
        fy    = K_in.at<float>(1, 1);
        cx    = K_in.at<float>(0, 2);
        cy    = K_in.at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        areInitialComputationsDone = false;
    }

    mb = mbf / fx;

    // Sophus/Eigen
    poseTlr        = Tlr_in;
    poseTrl        = poseTlr.inverse();
    rotationRlr    = poseTlr.rotationMatrix();
    translationTlr = poseTlr.translation();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeStartStereoMatches =
        std::chrono::steady_clock::now();
#endif
    if (computeStereoFishEyeMatches() != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeStereoFishEyeMatches returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeEndStereoMatches =
        std::chrono::steady_clock::now();

    stereoMatchTime =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            timeEndStereoMatches - timeStartStereoMatches)
            .count();
#endif

    // Put all descriptors in the same matrix
    cv::vconcat(descriptors, descriptorsRight, descriptors);

    // Initialize MapPoints
    mapPoints = std::vector<MapPoint *>(keyPointCount,
                                        static_cast<MapPoint *>(nullptr));

    // Initialize MapMarkers
    mapMarkers = markers_in;

    outlierFlags = std::vector<bool>(keyPointCount, false);

    if (assignFeaturesToGrid() != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: assignFeaturesToGrid returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    if (undistortKeyPoints() != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: undistortKeyPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

} // namespace core
} // namespace vs_graphs
