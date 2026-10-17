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
 * @file            monocularInitialization.cc
 *
 * @brief           Implements Tracking::monocularInitialization(), declared in
 *                  Tracking.h.
 */

#include "tracking/objects/TrackingClass.h"
#include "frame/frame.h"
#include "frame/objects/FrameStatusEnum.h"
#include "imu_types/bias/bias.h"
#include "imu_types/preintegrated/preintegrated.h"
#include "orb_matcher/objects/ORBmatcherStatusEnum.h"

#include "orb_matcher/orb_matcher.h"
#include "system/system.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace tracking
{

TrackingStatus Tracking::monocularInitialization()
{
    if (!isReadyToInitialize)
    {
        /* Set Reference frame::Frame */
        if (currentFrame.keyPoints.size() > 100)
        {
            initialFrame = frame::Frame(currentFrame);
            lastFrame    = frame::Frame(currentFrame);
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

            if (sensor == system::System::IMU_MONOCULAR)
            {
                if (p_imuPreintegratedFromLastKF)
                {
                    delete p_imuPreintegratedFromLastKF;
                }
                p_imuPreintegratedFromLastKF =
                    new imu_types::preintegrated::Preintegrated(imu_types::bias::Bias(), *p_imuCalibration);
                currentFrame.p_imuPreintegrated = p_imuPreintegratedFromLastKF;
            }

            isReadyToInitialize = true;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }
    }
    else
    {
        if ((static_cast<int>(currentFrame.keyPoints.size()) <= 100) ||
            ((sensor == system::System::IMU_MONOCULAR) &&
             (lastFrame.timeStamp - initialFrame.timeStamp > 1.0)))
        {
            isReadyToInitialize = false;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }

        /* Find correspondences */
        orb_matcher::ORBmatcher matcher(0.9, true);
        int        nmatches{};
        if (matcher.searchForInitialization(initialFrame,
                                            currentFrame,
                                            previousMatchedPoints,
                                            iniMatches,
                                            nmatches,
                                            100) !=
            orb_matcher::ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: searchForInitialization returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Check if there are enough correspondences */
        if (nmatches < 100)
        {
            isReadyToInitialize = false;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }

        Sophus::SE3f cameraPose_worldToCamera;
        std::vector<bool>
            triangulatedFlags; /* Triangulated Correspondences (mvIniMatches) */

        if (p_camera->reconstructWithTwoViews(initialFrame.keyPointsUndistorted,
                                              currentFrame.keyPointsUndistorted,
                                              iniMatches,
                                              cameraPose_worldToCamera,
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

            /* Set frame::Frame Poses */
            if (initialFrame.setPose(poseTc0w) !=
                frame::FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (currentFrame.setPose(cameraPose_worldToCamera * poseTc0w) !=
                frame::FrameStatus::FRAME_STATUS_SUCCESS)
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

} /* namespace tracking */
} /* namespace core */
} /* namespace vs_graphs */
