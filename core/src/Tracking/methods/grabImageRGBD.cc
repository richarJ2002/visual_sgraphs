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
 * @file            grabImageRGBD.cc
 *
 * @brief           Implements Tracking::grabImageRGBD(), declared in
 *                  Tracking.h.
 */

#include "System.h"
#include "Tracking.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::grabImageRGBD(
    const cv::Mat                                &imageRgb_in,
    const cv::Mat                                &imageD_in,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pointcloud_in,
    const double                                 &timestamp_in,
    std::string                                   filename_in,
    const std::vector<semantic::Marker *>         markers_in,
    Sophus::SE3f                                 &cameraPose_out)
{
    // Adaptive FAST threshold: adjust before feature extraction
    if (adjustFASTThreshold() != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: adjustFASTThreshold returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    imageGray          = imageRgb_in;
    cv::Mat imageDepth = imageD_in;

    if (imageGray.channels() == 3)
    {
        if (isRgbEnabled)
            cv::cvtColor(imageGray, imageGray, cv::COLOR_RGB2GRAY);
        else
            cv::cvtColor(imageGray, imageGray, cv::COLOR_BGR2GRAY);
    }
    else if (imageGray.channels() == 4)
    {
        if (isRgbEnabled)
            cv::cvtColor(imageGray, imageGray, cv::COLOR_RGBA2GRAY);
        else
            cv::cvtColor(imageGray, imageGray, cv::COLOR_BGRA2GRAY);
    }

    if ((std::fabs(depthMapFactor - 1.0f) > 1e-5) ||
        imageDepth.type() != CV_32F)
        imageDepth.convertTo(imageDepth, CV_32F, depthMapFactor);

    // RGB-D
    if (sensor == System::RGBD)
        currentFrame = Frame(imageRgb_in,
                             imageGray,
                             imageDepth,
                             p_pointcloud_in,
                             timestamp_in,
                             p_orbExtractorLeft.get(),
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             nullptr,
                             IMU::Calib(),
                             markers_in);
    // RGB-D Intertial
    else if (sensor == System::IMU_RGBD)
        currentFrame = Frame(imageRgb_in,
                             imageGray,
                             imageDepth,
                             p_pointcloud_in,
                             timestamp_in,
                             p_orbExtractorLeft.get(),
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             &lastFrame,
                             *p_imuCalibration,
                             markers_in);

    currentFrame.fileName  = filename_in;
    currentFrame.datasetId = numDataset;

#ifdef REGISTER_TIMES
    orbExtractionTimes_ms.push_back(currentFrame.orbExtractionTime);
#endif

    if (track() != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: track returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    Sophus::SE3<float> currentFrameGetPose{};
    if (currentFrame.getPose(currentFrameGetPose) !=
        FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    cameraPose_out = currentFrameGetPose;
    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
