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
 * @file            trackMonocular.cc
 *
 * @brief           Implements System::trackMonocular(), declared in System.h.
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
    System::trackMonocular(const cv::Mat                        &image_in,
                           const double                         &timestamp_in,
                           Sophus::SE3f                         &cameraPose_out,
                           const std::vector<IMU::Point>        &imuMeas_in,
                           std::string                           filename_in,
                           const std::vector<semantic::Marker *> markers_in)
{
    // Multi-thread to prevent race conditions
    {
        std::unique_lock<std::mutex> lock(resetMutex);
        if (isShutdownRequested)
        {
            cameraPose_out = Sophus::SE3f();
            return SystemStatus::SYSTEM_STATUS_SUCCESS;
        }
    }

    // Check if the sensor is Monocular
    if (sensor != MONOCULAR && sensor != IMU_MONOCULAR)
    {
        std::cerr
            << "ERROR: you called TrackMonocular but input sensor was not set "
               "to Monocular nor Monocular-Inertial."
            << std::endl;
        exit(-1);
    }

    // Obtain the images
    cv::Mat imToFeed = image_in.clone();
    bool    settingsNeedToResize{};
    if ((p_settings) &&
        p_settings->needToResize(settingsNeedToResize) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: needToResize returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_settings && settingsNeedToResize)
    {
        cv::Mat  resizedImage;
        cv::Size settingsNewImSize{};
        if (p_settings->newImSize(settingsNewImSize) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: newImSize returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        cv::resize(image_in, resizedImage, settingsNewImSize);
        imToFeed = resizedImage;
    }

    if (applyPendingModeAndResetRequests() !=
        SystemStatus::SYSTEM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: applyPendingModeAndResetRequests returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    if (sensor == System::IMU_MONOCULAR)
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

    Sophus::SE3f pose_worldToCamera{};
    if (p_tracker->grabImageMonocular(imToFeed,
                                      timestamp_in,
                                      filename_in,
                                      markers_in,
                                      envRooms,
                                      pose_worldToCamera) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: grabImageMonocular returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::unique_lock<std::mutex> lock2(stateMutex);
    trackingState      = p_tracker->state;
    trackedMapPoints   = p_tracker->currentFrame.mapPoints;
    trackedKeyPointsUn = p_tracker->currentFrame.keyPointsUndistorted;

    cameraPose_out = pose_worldToCamera;
    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
