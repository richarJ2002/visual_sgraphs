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

Sophus::SE3f
    System::trackMonocular(const cv::Mat                        &image_in,
                           const double                         &timestamp_in,
                           const vector<IMU::Point>             &imuMeas_in,
                           string                                filename_in,
                           const std::vector<semantic::Marker *> markers_in)
{
    // Multi-thread to prevent race conditions
    {
        unique_lock<mutex> lock(resetMutex);
        if (isShutdownRequested)
            return Sophus::SE3f();
    }

    // Check if the sensor is Monocular
    if (sensor != MONOCULAR && sensor != IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set "
                "to Monocular nor Monocular-Inertial."
             << endl;
        exit(-1);
    }

    // Obtain the images
    cv::Mat imToFeed = image_in.clone();
    if (p_settings && p_settings->needToResize())
    {
        cv::Mat resizedImage;
        cv::resize(image_in, resizedImage, p_settings->newImSize());
        imToFeed = resizedImage;
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

    if (sensor == System::IMU_MONOCULAR)
        for (size_t imuMeasurementIndex = 0;
             imuMeasurementIndex < imuMeas_in.size();
             imuMeasurementIndex++)
            p_tracker->grabImuData(imuMeas_in[imuMeasurementIndex]);

    Sophus::SE3f Tcw = p_tracker->grabImageMonocular(imToFeed,
                                                     timestamp_in,
                                                     filename_in,
                                                     markers_in,
                                                     envRooms);

    unique_lock<mutex> lock2(stateMutex);
    trackingState      = p_tracker->state;
    trackedMapPoints   = p_tracker->currentFrame.mapPoints;
    trackedKeyPointsUn = p_tracker->currentFrame.keyPointsUndistorted;

    return Tcw;
}

} // namespace core
} // namespace vs_graphs
