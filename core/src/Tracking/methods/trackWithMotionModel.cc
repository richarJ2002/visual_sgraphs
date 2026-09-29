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

bool Tracking::trackWithMotionModel()
{
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    updateLastFrame();

    if (p_atlas->isImuInitialized() &&
        (currentFrame.id > lastRelocFrameId + framesToResetIMU))
    {
        // Predict state with IMU if it is initialized and it doesnt need reset
        predictStateIMU();
        return true;
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

    int nmatches = matcher.searchByProjection(
        currentFrame,
        lastFrame,
        threshold,
        sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR);

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

        Verbose::printMess("Not enough matches, wider window search (radius " +
                               std::to_string(expandedThreshold) + ")!!",
                           Verbose::VERBOSITY_NORMAL);
        fill(currentFrame.mapPoints.begin(),
             currentFrame.mapPoints.end(),
             static_cast<MapPoint *>(nullptr));

        nmatches = matcher.searchByProjection(
            currentFrame,
            lastFrame,
            expandedThreshold,
            sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR);
        Verbose::printMess("Matches with wider search: " + to_string(nmatches),
                           Verbose::VERBOSITY_NORMAL);
        searchRadius = expandedThreshold;
        searchStep++;
    }

    if (nmatches < 20)
    {
        Verbose::printMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
            sensor == System::IMU_RGBD)
            return true;
        else
            return false;
    }

    // Optimize frame pose with all matches
    Optimizer::poseOptimization(&currentFrame);

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
        return nmatches > 20;
    }

    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap >= 10;
}

} // namespace core
} // namespace vs_graphs
