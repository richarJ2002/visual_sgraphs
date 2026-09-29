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
#include "Optimizer.h"
#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::trackWithMotionModel(bool &isTracked_out)
{
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    if (updateLastFrame() != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateLastFrame returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    bool atlasIsImuInitialized{};
    if (p_atlas->isImuInitialized(atlasIsImuInitialized) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (atlasIsImuInitialized &&
        (currentFrame.id > lastRelocFrameId + framesToResetIMU))
    {
        // Predict state with IMU if it is initialized and it doesnt need reset
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
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (currentFrame.setPose(velocity * lastFrameGetPose) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    fill(currentFrame.mapPoints.begin(),
         currentFrame.mapPoints.end(),
         static_cast<MapPoint *>(nullptr));

    // Project points seen in previous frame
    int threshold;

    if (sensor == System::STEREO)
        threshold = 7;
    else
        threshold = 15;

    int nmatches{};
    if (matcher.searchByProjection(
            currentFrame,
            lastFrame,
            threshold,
            sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR,
            nmatches) != ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: searchByProjection returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    // If few matches, use progressively wider window searches.
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

        if (Verbose::printMess(
                "Not enough matches, wider window search (radius " +
                    std::to_string(expandedThreshold) + ")!!",
                Verbose::VERBOSITY_NORMAL) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        fill(currentFrame.mapPoints.begin(),
             currentFrame.mapPoints.end(),
             static_cast<MapPoint *>(nullptr));

        int matcherByProjection{};
        if (matcher.searchByProjection(currentFrame,
                                       lastFrame,
                                       expandedThreshold,
                                       sensor == System::MONOCULAR ||
                                           sensor == System::IMU_MONOCULAR,
                                       matcherByProjection) !=
            ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: searchByProjection returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        nmatches = matcherByProjection;
        if (Verbose::printMess("Matches with wider search: " +
                                   to_string(nmatches),
                               Verbose::VERBOSITY_NORMAL) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
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
        if (Verbose::printMess("Not enough matches!!",
                               Verbose::VERBOSITY_NORMAL) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
            sensor == System::IMU_RGBD)
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

    // Optimize frame pose with all matches
    int inlierCount{};
    if (Optimizer::poseOptimization(&currentFrame, inlierCount) !=
        OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: poseOptimization returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    // Discard outliers
    int nmatchesMap = 0;
    for (int keyPointIndex = 0; keyPointIndex < currentFrame.keyPointCount;
         keyPointIndex++)
    {
        if (currentFrame.mapPoints[keyPointIndex])
        {
            if (currentFrame.outlierFlags[keyPointIndex])
            {
                MapPoint *p_mapPoint = currentFrame.mapPoints[keyPointIndex];

                currentFrame.mapPoints[keyPointIndex] =
                    static_cast<MapPoint *>(nullptr);
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
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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

    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
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

} // namespace core
} // namespace vs_graphs
