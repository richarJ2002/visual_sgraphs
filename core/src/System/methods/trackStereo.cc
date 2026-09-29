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

#include "LocalMapping.h"
#include "System.h"
#include "Tracking.h"

#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

Sophus::SE3f
    System::trackStereo(const cv::Mat                        &imageLeft_in,
                        const cv::Mat                        &imageRight_in,
                        const double                         &timestamp_in,
                        const vector<IMU::Point>             &imuMeas_in,
                        string                                filename_in,
                        const std::vector<semantic::Marker *> markers_in)
{
    if (sensor != STEREO && sensor != IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to "
                "Stereo nor Stereo-Inertial."
             << endl;
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

    // Check mode change
    {
        unique_lock<mutex> lock(modeMutex);
        if (isLocalizationModeActivationRequested)
        {
            p_localMapper->requestStop();

            // Wait until Local Mapping has effectively stopped
            while (!p_localMapper->isStopped())
            {
                usleep(1000);
            }

            p_tracker->informOnlyTracking(true);
            isLocalizationModeActivationRequested = false;
        }
        if (isLocalizationModeDeactivationRequested)
        {
            p_tracker->informOnlyTracking(false);
            p_localMapper->release();
            isLocalizationModeDeactivationRequested = false;
        }
    }

    {
        unique_lock<mutex> lock(resetMutex);
        if (isResetRequested)
        {
            ResetCause resetCause{};
            if (consumeResetCause(this, resetCause) !=
                ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: consumeResetCause returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            p_tracker->reset();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            isResetRequested          = false;
            isResetActiveMapRequested = false;
        }
        else if (isResetActiveMapRequested)
        {
            ResetCause resetCause2{};
            if (consumeResetCause(this, resetCause2) !=
                ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: consumeResetCause returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (reportResetAttribution(
                    resetCause2,
                    ResetAction::RESET_ACTIVE_MAP_EXECUTION) !=
                ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: reportResetAttribution returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            p_tracker->resetActiveMap();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            isResetActiveMapRequested = false;
        }
    }

    if (sensor == System::IMU_STEREO)
        for (size_t imuMeasurementIndex = 0;
             imuMeasurementIndex < imuMeas_in.size();
             imuMeasurementIndex++)
            p_tracker->grabImuData(imuMeas_in[imuMeasurementIndex]);

    Sophus::SE3f Tcw = p_tracker->grabImageStereo(imLeftToFeed,
                                                  imRightToFeed,
                                                  timestamp_in,
                                                  filename_in,
                                                  markers_in,
                                                  envRooms);

    unique_lock<mutex> lock2(stateMutex);
    trackingState           = p_tracker->state;
    trackingInliers         = p_tracker->getMatchesInliers();
    lastFrameTimestamp      = timestamp_in;
    trackedMapPoints        = p_tracker->currentFrame.mapPoints;
    trackedKeyPointsUn      = p_tracker->currentFrame.keyPointsUndistorted;
    currentCameraPose_World = Tcw.inverse();
    isCurrentCameraPoseValid =
        trackingState == Tracking::OK &&
        currentCameraPose_World.translation().allFinite() &&
        currentCameraPose_World.rotationMatrix().allFinite();

    return Tcw;
}

} // namespace core
} // namespace vs_graphs
