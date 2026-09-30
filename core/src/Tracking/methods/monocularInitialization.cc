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

#include "Tracking.h"

#include "ORBmatcher.h"
#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::monocularInitialization()
{
    if (!isReadyToInitialize)
    {
        // Set Reference Frame
        if (currentFrame.keyPoints.size() > 100)
        {
            initialFrame = Frame(currentFrame);
            lastFrame    = Frame(currentFrame);
            previousMatchedPoints.resize(
                currentFrame.keyPointsUndistorted.size());
            for (size_t keyPointsUndistortedIndex = 0;
                 keyPointsUndistortedIndex <
                 currentFrame.keyPointsUndistorted.size();
                 keyPointsUndistortedIndex++)
                previousMatchedPoints[keyPointsUndistortedIndex] =
                    currentFrame.keyPointsUndistorted[keyPointsUndistortedIndex]
                        .pt;

            std::fill(iniMatches.begin(), iniMatches.end(), -1);

            if (sensor == System::IMU_MONOCULAR)
            {
                if (p_imuPreintegratedFromLastKF)
                {
                    delete p_imuPreintegratedFromLastKF;
                }
                p_imuPreintegratedFromLastKF =
                    new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
                currentFrame.p_imuPreintegrated = p_imuPreintegratedFromLastKF;
            }

            isReadyToInitialize = true;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }
    }
    else
    {
        if (((int)currentFrame.keyPoints.size() <= 100) ||
            ((sensor == System::IMU_MONOCULAR) &&
             (lastFrame.timeStamp - initialFrame.timeStamp > 1.0)))
        {
            isReadyToInitialize = false;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }

        // Find correspondences
        ORBmatcher matcher(0.9, true);
        int        nmatches{};
        if (matcher.searchForInitialization(initialFrame,
                                            currentFrame,
                                            previousMatchedPoints,
                                            iniMatches,
                                            nmatches,
                                            100) !=
            ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: searchForInitialization returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        // Check if there are enough correspondences
        if (nmatches < 100)
        {
            isReadyToInitialize = false;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }

        Sophus::SE3f Tcw;
        std::vector<bool>
            triangulatedFlags; // Triangulated Correspondences (mvIniMatches)

        if (p_camera->reconstructWithTwoViews(initialFrame.keyPointsUndistorted,
                                              currentFrame.keyPointsUndistorted,
                                              iniMatches,
                                              Tcw,
                                              iniP3D,
                                              triangulatedFlags))
        {
            for (size_t keyPointsUndistortedIndex = 0, iend = iniMatches.size();
                 keyPointsUndistortedIndex < iend;
                 keyPointsUndistortedIndex++)
            {
                if (iniMatches[keyPointsUndistortedIndex] >= 0 &&
                    !triangulatedFlags[keyPointsUndistortedIndex])
                {
                    iniMatches[keyPointsUndistortedIndex] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            // mInitialFrame.setPose(Sophus::SE3f());
            if (initialFrame.setPose(poseTc0w) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (currentFrame.setPose(Tcw * poseTc0w) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            if (createInitialMapMonocular() !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: createInitialMapMonocular returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
    }

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
