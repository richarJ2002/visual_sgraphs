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
#include "SemanticsManager.h"
#include "System.h"
#include "Tracking.h"

#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

Sophus::SE3f System::trackRGBD(
    const cv::Mat                                &colorImage_in,
    const cv::Mat                                &depthmap_in,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_mainCloud_in,
    const double                                 &timestamp_in,
    const vector<IMU::Point>                     &imuMeas_in,
    string                                        filename_in,
    const std::vector<semantic::Marker *>         markers_in)
{
    // Check if the sensor is correctly set as RGB-D
    if (sensor != RGBD && sensor != IMU_RGBD)
    {
        cerr << "[Error] Improper sensor-type is set for 'TrackRGBD'! Exiting "
                "..."
             << endl;
        exit(-1);
    }

    // Obtain the images
    cv::Mat imToFeed      = colorImage_in.clone();
    cv::Mat imDepthToFeed = depthmap_in.clone();
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
        cv::resize(colorImage_in, resizedImage, settingsNewImSize);
        imToFeed = resizedImage;
        cv::Size settingsNewImSize2{};
        if (p_settings->newImSize(settingsNewImSize2) !=
            utils::settings::SettingsStatus::SETTINGS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: newImSize returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        cv::resize(depthmap_in, imDepthToFeed, settingsNewImSize2);
    }

    applyPendingModeAndResetRequests();

    // Apply IMU measurements
    if (sensor == System::IMU_RGBD)
        for (size_t imuMeasurementIndex = 0;
             imuMeasurementIndex < imuMeas_in.size();
             imuMeasurementIndex++)
            p_tracker->grabImuData(imuMeas_in[imuMeasurementIndex]);

    // Track RGB-D images
    Sophus::SE3f Tcw = p_tracker->grabImageRGBD(imToFeed,
                                                imDepthToFeed,
                                                p_mainCloud_in,
                                                timestamp_in,
                                                filename_in,
                                                markers_in,
                                                envRooms);

    unique_lock<mutex> lock2(stateMutex);
    trackingState      = p_tracker->state;
    trackingInliers    = p_tracker->getMatchesInliers();
    lastFrameTimestamp = timestamp_in;
    trackedMapPoints   = p_tracker->currentFrame.mapPoints;
    trackedKeyPointsUn = p_tracker->currentFrame.keyPointsUndistorted;

    currentCameraPose_World = Tcw.inverse();
    isCurrentCameraPoseValid =
        trackingState == Tracking::OK &&
        currentCameraPose_World.translation().allFinite() &&
        currentCameraPose_World.rotationMatrix().allFinite();

    /* Feed the real per-frame tracking state to SemanticsManager's reset
     * anchor (lastKnownRoomId via onTrackingLost()/onTrackingRecovered()).
     * Previously the ONLY caller of these was GetMissionHealthSnapshot(),
     * itself only invoked from the get_mission_health ROS service -- which
     * nothing calls unless scripts/sim_lockstep_controller.py's opt-in
     * --lockstep mode is running. Every run without --lockstep therefore
     * left lastKnownRoomId at its unset default (-1) for the whole
     * mission: SemanticCandidates::generate() silently fell back to
     * unanchored scoring on every single reset, never told which room the
     * UAV was actually in when tracking was lost. This is the same
     * TrackRGBD() call every real frame already goes through, so it fires
     * at real tracking-loss/recovery cadence instead of only on-demand. */
    if (p_semanticsManager != nullptr)
    {
        if (trackingState == Tracking::LOST)
        {
            p_semanticsManager->onTrackingLost();
        }
        else
        {
            p_semanticsManager->onTrackingRecovered();
        }
    }

    /* Detect map restart for room-context carryover.
     * The SemanticsManager::Run() thread performs the actual room
     * matching once rooms exist in the new map; we only log here. */
    {
        Map *p_currentMap = p_atlas->getCurrentMap();
        if (p_currentMap)
        {
            long unsigned int mapId = p_currentMap->getId();
            if (isAwaitingFirstMap)
            {
                lastProcessedMapId = mapId;
                isAwaitingFirstMap = false;
            }
            else if (mapId != lastProcessedMapId)
            {
                const long unsigned int previousMapId = lastProcessedMapId;
                lastProcessedMapId                    = mapId;
                std::cout << "[System] Map restart detected (mapId: "
                          << previousMapId << " -> " << mapId << ")"
                          << std::endl;
                /* NOTE: matchRoomsToContext() is now called from
                 * SemanticsManager::Run() after room detection, not here. */
            }
        }
    }

    return Tcw;
}

} // namespace core
} // namespace vs_graphs
