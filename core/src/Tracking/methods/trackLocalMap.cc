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

#include "LocalMapping.h"
#include "Optimizer.h"
#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

bool Tracking::trackLocalMap()
{

    // We have an estimation of the camera pose and some map points tracked in
    // the frame. We retrieve the local map and try to find matches to points in
    // the local map.
    trackedFr++;

    updateLocalMap();
    searchLocalPoints();

    // TOO check outliers before PO
    int aux1 = 0, aux2 = 0;
    for (int keyPointIndex = 0; keyPointIndex < currentFrame.keyPointCount;
         keyPointIndex++)
        if (currentFrame.mapPoints[keyPointIndex])
        {
            aux1++;
            if (currentFrame.outlierFlags[keyPointIndex])
                aux2++;
        }

    if (!p_atlas->isImuInitialized())
        Optimizer::poseOptimization(&currentFrame);
    else
    {
        if (currentFrame.id <= lastRelocFrameId + framesToResetIMU)
        {
            Verbose::printMess("TLM: PoseOptimization ",
                               Verbose::VERBOSITY_DEBUG);
            Optimizer::poseOptimization(&currentFrame);
        }
        else
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if (!isMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::printMess("TLM: PoseInertialOptimizationLastFrame ",
                                   Verbose::VERBOSITY_DEBUG);
                Optimizer::poseInertialOptimizationLastFrame(
                    &currentFrame); // ,
                                    // !mpLastKeyFrame->getMap()->getInertialBA1());
            }
            else
            {
                Verbose::printMess("TLM: PoseInertialOptimizationLastKeyFrame ",
                                   Verbose::VERBOSITY_DEBUG);
                Optimizer::poseInertialOptimizationLastKeyFrame(
                    &currentFrame); // ,
                                    // !mpLastKeyFrame->getMap()->getInertialBA1());
            }
        }
    }

    aux1 = 0, aux2 = 0;
    for (int keyPointIndex = 0; keyPointIndex < currentFrame.keyPointCount;
         keyPointIndex++)
        if (currentFrame.mapPoints[keyPointIndex])
        {
            aux1++;
            if (currentFrame.outlierFlags[keyPointIndex])
                aux2++;
        }

    matchesInliers = 0;

    // Update MapPoints Statistics
    int closeInlierCount = 0;
    int farInlierCount   = 0;
    for (int keyPointIndex = 0; keyPointIndex < currentFrame.keyPointCount;
         keyPointIndex++)
    {
        if (currentFrame.mapPoints[keyPointIndex])
        {
            if (!currentFrame.outlierFlags[keyPointIndex])
            {
                if (currentFrame.mapPoints[keyPointIndex]->increaseFound() !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: increaseFound returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (!isTrackingOnlyMode)
                {
                    int observationCount{};
                    if (currentFrame.mapPoints[keyPointIndex]
                            ->getObservationCount(observationCount) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getObservationCount returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (observationCount > 0)
                        matchesInliers++;
                }
                else
                    matchesInliers++;

                // Track close vs far inliers for adaptive acceptance
                if ((sensor == System::RGBD || sensor == System::IMU_RGBD ||
                     sensor == System::STEREO ||
                     sensor == System::IMU_STEREO) &&
                    keyPointIndex < (int)currentFrame.depths.size() &&
                    currentFrame.depths[keyPointIndex] > 0)
                {
                    if (currentFrame.depths[keyPointIndex] < depthThreshold)
                        closeInlierCount++;
                    else
                        farInlierCount++;
                }
            }
            else if (sensor == System::STEREO)
                currentFrame.mapPoints[keyPointIndex] =
                    static_cast<MapPoint *>(nullptr);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    p_localMapper->matchesInliers = matchesInliers;
    if (currentFrame.id < lastRelocFrameId + maxFrames && matchesInliers < 25)
        return false;

    if ((matchesInliers > 10) && (state == RECENTLY_LOST))
        return true;

    // AGGRESSIVE TRACKING ACCEPTANCE for corridors: Require only 5 close
    // inliers In featureless corridors, close points (walls/floor) are more
    // reliable than far points
    if (sensor == System::IMU_MONOCULAR)
    {
        // For IMU monocular, rely on IMU + minimum visual inliers - very
        // permissive LOWERED: 8->5 with IMU, 25->15 without IMU
        if ((matchesInliers < 5 && p_atlas->isImuInitialized()) ||
            (matchesInliers < 15 && !p_atlas->isImuInitialized()))
        {
            return false;
        }
        else
            return true;
    }
    else if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        // For IMU stereo/RGBD: require only 5 close inliers (AGGRESSIVE)
        // In corridors, close points (walls) provide strong geometric
        // constraints
        if (closeInlierCount >= 5 && matchesInliers >= 5)
            return true;
        else if (matchesInliers >= 10) // fallback with more total inliers
            return true;
        else
            return false;
    }
    else if (sensor == System::RGBD || sensor == System::STEREO)
    {
        // For visual-only stereo/RGBD: require only 5 close inliers
        // (AGGRESSIVE) Close points are more reliable in corridors (wall/floor
        // planes)
        if (closeInlierCount >= 5)
            return true;
        else if (closeInlierCount >= 3 && matchesInliers >= 10)
            return true;
        else if (matchesInliers >= 15)
            return true;
        else
            return false;
    }
    else
    {
        // Monocular: lowered threshold
        if (matchesInliers < 10)
            return false;
        else
            return true;
    }
}

} // namespace core
} // namespace vs_graphs
