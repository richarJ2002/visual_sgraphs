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

#include "System.h"

#include <opencv2/imgproc.hpp>

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
    if (p_settings && p_settings->needToResize())
    {
        cv::Mat resizedImage;
        cv::resize(colorImage_in, resizedImage, p_settings->newImSize());
        imToFeed = resizedImage;
        cv::resize(depthmap_in, imDepthToFeed, p_settings->newImSize());
    }

    // Check for mode change
    {
        unique_lock<mutex> lock(modeMutex);
        if (isLocalizationModeActivationRequested)
        {
            p_localMapper->requestStop();
            // Wait until Local Mapping has effectively stopped
            while (!p_localMapper->isStopped())
                usleep(1000);
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

    // Check reset
    {
        unique_lock<mutex> lock(resetMutex);
        if (isResetRequested)
        {
            (void)consumeResetCause(this);
            p_tracker->reset();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            isResetRequested          = false;
            isResetActiveMapRequested = false;
        }
        else if (isResetActiveMapRequested)
        {
            reportResetAttribution(consumeResetCause(this),
                                   ResetAction::RESET_ACTIVE_MAP_EXECUTION);
            p_tracker->resetActiveMap();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            isResetActiveMapRequested = false;
        }
    }

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
