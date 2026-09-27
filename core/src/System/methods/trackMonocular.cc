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
    System::trackMonocular(const cv::Mat                        &im,
                           const double                         &timestamp,
                           const vector<IMU::Point>             &vImuMeas,
                           string                                filename,
                           const std::vector<semantic::Marker *> markers)
{
    // Multi-thread to prevent race conditions
    {
        unique_lock<mutex> lock(mMutexReset);
        if (shutdownRequested)
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
    cv::Mat imToFeed = im.clone();
    if (settings_ && settings_->needToResize())
    {
        cv::Mat resizedImage;
        cv::resize(im, resizedImage, settings_->newImSize());
        imToFeed = resizedImage;
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (activateLocalizationModeRequested)
        {
            p_localMapper->requestStop();

            // Wait until Local Mapping has effectively stopped
            while (!p_localMapper->isStopped())
            {
                usleep(1000);
            }

            p_tracker->informOnlyTracking(true);
            activateLocalizationModeRequested = false;
        }
        if (deactivateLocalizationModeRequested)
        {
            p_tracker->informOnlyTracking(false);
            p_localMapper->release();
            deactivateLocalizationModeRequested = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (resetRequested)
        {
            (void)consumeResetCause(this);
            p_tracker->reset();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            resetRequested          = false;
            resetActiveMapRequested = false;
        }
        else if (resetActiveMapRequested)
        {
            reportResetAttribution(consumeResetCause(this),
                                   ResetAction::RESET_ACTIVE_MAP_EXECUTION);
            p_tracker->resetActiveMap();
            resetCount.fetch_add(1U, std::memory_order_relaxed);
            resetActiveMapRequested = false;
        }
    }

    if (sensor == System::IMU_MONOCULAR)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            p_tracker->grabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = p_tracker->grabImageMonocular(imToFeed,
                                                     timestamp,
                                                     filename,
                                                     markers,
                                                     envRooms);

    unique_lock<mutex> lock2(mMutexState);
    trackingState      = p_tracker->state;
    trackedMapPoints   = p_tracker->currentFrame.mapPoints;
    trackedKeyPointsUn = p_tracker->currentFrame.keyPointsUndistorted;

    return Tcw;
}

} // namespace core
} // namespace vs_graphs
