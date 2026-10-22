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
 * @file            trackWithMotionModel.cc
 *
 * @brief           Implements Tracking::trackWithMotionModel(), declared in
 *                  Tracking.h.
 */

#include "estimation/tracking/objects/TrackingClass.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/frame/objects/FrameStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"
#include "optimization/optimizer/objects/OptimizerStatusEnum.h"
#include "visual_features/orb_matcher/objects/ORBmatcherStatusEnum.h"
#include "execution/system/objects/VerboseStatusEnum.h"

#include "visual_features/orb_matcher/orb_matcher.h"
#include "optimization/optimizer/optimizer.h"
#include "execution/system/system.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace tracking
{

TrackingStatus Tracking::trackWithMotionModel(bool &isTracked_out)
{
    orb_matcher::ORBmatcher matcher(0.9, true);

    /*!
     * Update last frame pose according to its reference keyframe
     * Create "visual odometry" points if in Localization Mode
     */
    if (updateLastFrame() != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateLastFrame returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    bool atlasIsImuInitialized{};
    if (p_atlas->isImuInitialized(atlasIsImuInitialized) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (atlasIsImuInitialized &&
        (currentFrame.id > lastRelocFrameId + framesToResetIMU))
    {
        /*!
         * Predict state with IMU if it is initialized and it doesnt need reset
         */
        bool isPredicted{};
        if (predictStateIMU(isPredicted) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: predictStateIMU returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        isTracked_out = true;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }
    else
    {
        Sophus::SE3<float> lastFrameGetPose{};
        if (lastFrame.getPose(lastFrameGetPose) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (currentFrame.setPose(velocity * lastFrameGetPose) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    std::fill(currentFrame.mapPoints.begin(),
              currentFrame.mapPoints.end(),
              static_cast<map_point::MapPoint *>(nullptr));

    /* Project points seen in previous frame */
    int threshold;

    if (sensor == system::System::STEREO)
        threshold = 7;
    else
        threshold = 15;

    int nmatches{};
    if (matcher.searchByProjection(currentFrame,
                                   lastFrame,
                                   threshold,
                                   sensor == system::System::MONOCULAR ||
                                       sensor == system::System::IMU_MONOCULAR,
                                   nmatches) !=
        orb_matcher::ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: searchByProjection returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* If few matches, use progressively wider window searches. */
    int searchStep   = 1;
    int searchRadius = threshold;
    while (nmatches < 20 && searchRadius < motionModelMaxSearchRadius)
    {
        int expandedThreshold = static_cast<int>(std::ceil(
            threshold *
            (1.0F + searchStep * (motionModelSearchRadiusMultiplier - 1.0F))));
        expandedThreshold =
            std::min(expandedThreshold, motionModelMaxSearchRadius);
        if (expandedThreshold <= searchRadius)
        {
            break;
        }

        if (system::Verbose::printMess(
                "Not enough matches, wider window search (radius " +
                    std::to_string(expandedThreshold) + ")!!",
                system::Verbose::VERBOSITY_NORMAL) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::fill(currentFrame.mapPoints.begin(),
                  currentFrame.mapPoints.end(),
                  static_cast<map_point::MapPoint *>(nullptr));

        int matcherByProjection{};
        if (matcher.searchByProjection(currentFrame,
                                       lastFrame,
                                       expandedThreshold,
                                       sensor == system::System::MONOCULAR ||
                                           sensor ==
                                               system::System::IMU_MONOCULAR,
                                       matcherByProjection) !=
            orb_matcher::ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: searchByProjection returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        nmatches = matcherByProjection;
        if (system::Verbose::printMess("Matches with wider search: " +
                                   std::to_string(nmatches),
                               system::Verbose::VERBOSITY_NORMAL) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        searchRadius = expandedThreshold;
        searchStep++;
    }

    if (nmatches < 20)
    {
        if (system::Verbose::printMess("Not enough matches!!",
                               system::Verbose::VERBOSITY_NORMAL) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (sensor == system::System::IMU_MONOCULAR ||
            sensor == system::System::IMU_STEREO ||
            sensor == system::System::IMU_RGBD)
        {
            isTracked_out = true;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }
        else
        {
            isTracked_out = false;
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }
    }

    /* Optimize frame pose with all matches */
    int inlierCount{};
    if (optimizer::Optimizer::poseOptimization(&currentFrame, inlierCount) !=
        optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: poseOptimization returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Discard outliers */
    int nmatchesMap = 0;
    for (int keyPointIndex = 0; keyPointIndex < currentFrame.keyPointCount;
         keyPointIndex++)
    {
        if (currentFrame.mapPoints[keyPointIndex])
        {
            if (currentFrame.outlierFlags[keyPointIndex])
            {
                map_point::MapPoint *p_mapPoint =
                    currentFrame.mapPoints[keyPointIndex];

                currentFrame.mapPoints[keyPointIndex] =
                    static_cast<map_point::MapPoint *>(nullptr);
                currentFrame.outlierFlags[keyPointIndex] = false;
                if (keyPointIndex < currentFrame.leftKeyPointCount)
                {
                    p_mapPoint->isTrackedInView = false;
                }
                else
                {
                    p_mapPoint->isTrackedInRightView = false;
                }
                p_mapPoint->lastSeenFrameId = currentFrame.id;
                nmatches--;
            }
            else
            {
                int observationCount{};
                if (currentFrame.mapPoints[keyPointIndex]->getObservationCount(
                        observationCount) !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getObservationCount returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (observationCount > 0)
                {
                    nmatchesMap++;
                }
            }
        }
    }

    if (isTrackingOnlyMode)
    {
        isVisualOdometry = nmatchesMap < 10;
        isTracked_out    = nmatches > 20;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    if (sensor == system::System::IMU_MONOCULAR ||
        sensor == system::System::IMU_STEREO ||
        sensor == system::System::IMU_RGBD)
    {
        isTracked_out = true;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }
    else
    {
        isTracked_out = nmatchesMap >= 10;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }
}

} /* namespace tracking */
} /* namespace core */
} /* namespace vs_graphs */
