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
#include "ORBmatcher.h"
#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void Tracking::searchLocalPoints()
{
    // Do not search map points already matched
    for (vector<MapPoint *>::iterator vit  = currentFrame.mapPoints.begin(),
                                      vend = currentFrame.mapPoints.end();
         vit != vend;
         vit++)
    {
        MapPoint *p_mapPoint = *vit;
        if (p_mapPoint)
        {
            bool mapPointIsBad{};
            if (p_mapPoint->isBad(mapPointIsBad) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (mapPointIsBad)
            {
                *vit = static_cast<MapPoint *>(nullptr);
            }
            else
            {
                if (p_mapPoint->increaseVisible() !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: increaseVisible returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                p_mapPoint->lastSeenFrameId      = currentFrame.id;
                p_mapPoint->isTrackedInView      = false;
                p_mapPoint->isTrackedInRightView = false;
            }
        }
    }

    int toMatchCount = 0;

    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit  = localMapPoints.begin(),
                                      vend = localMapPoints.end();
         vit != vend;
         vit++)
    {
        MapPoint *p_mapPoint = *vit;

        if (p_mapPoint->lastSeenFrameId == currentFrame.id)
            continue;
        bool mapPointIsBad2{};
        if (p_mapPoint->isBad(mapPointIsBad2) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad2)
            continue;
        // Project (this fills MapPoint variables for matching)
        bool currentFrameIsInFrustum{};
        if (currentFrame.isInFrustum(p_mapPoint,
                                     0.5,
                                     currentFrameIsInFrustum) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInFrustum returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (currentFrameIsInFrustum)
        {
            if (p_mapPoint->increaseVisible() !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: increaseVisible returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            toMatchCount++;
        }
        if (p_mapPoint->isTrackedInView)
        {
            currentFrame.projectedPoints[p_mapPoint->id] =
                cv::Point2f(p_mapPoint->trackProjX, p_mapPoint->trackProjY);
        }
    }

    if (toMatchCount > 0)
    {
        ORBmatcher matcher(0.8);
        int        threshold = 1;
        if (sensor == System::RGBD || sensor == System::IMU_RGBD)
            threshold = 3;
        if (p_atlas->isImuInitialized())
        {
            bool inertialBA2{};
            if (p_atlas->getCurrentMap()->getInertialBA2(inertialBA2) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getInertialBA2 returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (inertialBA2)
                threshold = 2;
            else
                threshold = 6;
        }
        else if (!p_atlas->isImuInitialized() &&
                 (sensor == System::IMU_MONOCULAR ||
                  sensor == System::IMU_STEREO || sensor == System::IMU_RGBD))
        {
            threshold = 10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if (currentFrame.id < lastRelocFrameId + 2)
            threshold = 5;

        if (state == LOST ||
            state == RECENTLY_LOST) // Lost for less than 1 second
            threshold = 15;         // 15

        // AGGRESSIVE: Even wider search during degraded tracking in corridors
        // If we have very few inliers, expand search radius significantly
        if (matchesInliers < 30 && matchesInliers > 0)
        {
            threshold = std::min(threshold * 3, motionModelMaxSearchRadius);
            Verbose::printMess("[Tracking] Expanded search radius to " +
                                   std::to_string(threshold) + " (inliers: " +
                                   std::to_string(matchesInliers) + ")",
                               Verbose::VERBOSITY_NORMAL);
        }

        // DEPTH-AIDED TRACKING: For RGB-D, use depth to guide matching window
        // In low-texture corridors, constrain search using known depth
        const bool isDepthGuided =
            (sensor == System::RGBD || sensor == System::IMU_RGBD) &&
            currentFrame.depths.size() > 0;
        matcher.searchByProjection(currentFrame,
                                   localMapPoints,
                                   threshold,
                                   p_localMapper->shouldSkipFarPoints,
                                   p_localMapper->farPointsThreshold,
                                   isDepthGuided
                                       ? std::optional<float>(depthThreshold)
                                       : std::nullopt);
    }
}

} // namespace core
} // namespace vs_graphs
