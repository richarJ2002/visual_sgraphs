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

Sophus::SE3f System::trackStereo(const cv::Mat            &imLeft,
                                 const cv::Mat            &imRight,
                                 const double             &timestamp,
                                 const vector<IMU::Point> &vImuMeas,
                                 string                    filename,
                                 const std::vector<semantic::Marker *> markers)
{
    if (sensor != STEREO && sensor != IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to "
                "Stereo nor Stereo-Inertial."
             << endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if (settings_ && settings_->needToRectify())
    {
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if (settings_ && settings_->needToResize())
    {
        cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
        cv::resize(imRight, imRightToFeed, settings_->newImSize());
    }
    else
    {
        imLeftToFeed  = imLeft.clone();
        imRightToFeed = imRight.clone();
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

    if (sensor == System::IMU_STEREO)
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            p_tracker->grabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = p_tracker->grabImageStereo(imLeftToFeed,
                                                  imRightToFeed,
                                                  timestamp,
                                                  filename,
                                                  markers,
                                                  envRooms);

    unique_lock<mutex> lock2(mMutexState);
    trackingState           = p_tracker->state;
    trackingInliers         = p_tracker->getMatchesInliers();
    lastFrameTimestamp      = timestamp;
    trackedMapPoints        = p_tracker->currentFrame.mapPoints;
    trackedKeyPointsUn      = p_tracker->currentFrame.keyPointsUndistorted;
    currentCameraPose_World = Tcw.inverse();
    currentCameraPoseValid =
        trackingState == Tracking::OK &&
        currentCameraPose_World.translation().allFinite() &&
        currentCameraPose_World.rotationMatrix().allFinite();

    return Tcw;
}

} // namespace core
} // namespace vs_graphs
