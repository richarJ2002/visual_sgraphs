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
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            trackStereo.cc
 *
 * @brief           Implements System::trackStereo(), declared in System.h.
 */

#include "LocalMapping.h"
#include "System.h"
#include "Tracking.h"

#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus
    System::trackStereo(const cv::Mat                        &imageLeft_in,
                        const cv::Mat                        &imageRight_in,
                        const double                         &timestamp_in,
                        Sophus::SE3f                         &cameraPose_out,
                        const std::vector<IMU::Point>        &imuMeas_in,
                        std::string                           filename_in,
                        const std::vector<semantic::Marker *> markers_in)
{
    if (sensor != STEREO && sensor != IMU_STEREO)
    {
        std::cerr
            << "ERROR: you called TrackStereo but input sensor was not set to "
               "Stereo nor Stereo-Inertial."
            << std::endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    bool    settingsNeedToRectify{};
    if ((p_settings) &&
        p_settings->needToRectify(settingsNeedToRectify) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: needToRectify returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool settingsNeedToResize{};
    if ((p_settings) &&
        p_settings->needToResize(settingsNeedToResize) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: needToResize returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_settings && settingsNeedToRectify)
    {
        cv::Mat M1l{};
        if (p_settings->M1l(M1l) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: M1l returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        cv::Mat M2l{};
        if (p_settings->M2l(M2l) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: M2l returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        cv::Mat M1r{};
        if (p_settings->M1r(M1r) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: M1r returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        cv::Mat M2r{};
        if (p_settings->M2r(M2r) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: M2r returned a failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }

        cv::remap(imageLeft_in, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imageRight_in, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if (p_settings && settingsNeedToResize)
    {
        cv::Size settingsNewImSize{};
        if (p_settings->newImSize(settingsNewImSize) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: newImSize returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        cv::resize(imageLeft_in, imLeftToFeed, settingsNewImSize);
        cv::Size settingsNewImSize2{};
        if (p_settings->newImSize(settingsNewImSize2) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: newImSize returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        cv::resize(imageRight_in, imRightToFeed, settingsNewImSize2);
    }
    else
    {
        imLeftToFeed  = imageLeft_in.clone();
        imRightToFeed = imageRight_in.clone();
    }

    if (applyPendingModeAndResetRequests() !=
        SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: applyPendingModeAndResetRequests returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    if (sensor == System::IMU_STEREO)
    {
        for (size_t imuMeasurementIndex = 0;
             imuMeasurementIndex < imuMeas_in.size();
             imuMeasurementIndex++)
        {
            if (p_tracker->grabImuData(imuMeas_in[imuMeasurementIndex]) !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: grabImuData returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    Sophus::SE3f cameraPose_worldToCamera{};
    if (p_tracker->grabImageStereo(imLeftToFeed,
                                   imRightToFeed,
                                   timestamp_in,
                                   filename_in,
                                   markers_in,
                                   cameraPose_worldToCamera) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: grabImageStereo returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    std::unique_lock<std::mutex> lock2(stateMutex);
    trackingState = p_tracker->state;
    int trackerMatchesInliers{};
    if (p_tracker->getMatchesInliers(trackerMatchesInliers) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMatchesInliers returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    trackingInliers         = trackerMatchesInliers;
    lastFrameTimestamp      = timestamp_in;
    trackedMapPoints        = p_tracker->currentFrame.mapPoints;
    trackedKeyPointsUn      = p_tracker->currentFrame.keyPointsUndistorted;
    currentCameraPose_world = cameraPose_worldToCamera.inverse();
    isCurrentCameraPoseValid =
        trackingState == Tracking::OK &&
        currentCameraPose_world.translation().allFinite() &&
        currentCameraPose_world.rotationMatrix().allFinite();

    cameraPose_out = cameraPose_worldToCamera;
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
