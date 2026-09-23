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
        MapPoint *pMP = *vit;
        if (pMP)
        {
            if (pMP->isBad())
            {
                *vit = static_cast<MapPoint *>(nullptr);
            }
            else
            {
                pMP->increaseVisible();
                pMP->lastSeenFrameId = currentFrame.mnId;
                pMP->trackInView     = false;
                pMP->trackInViewR    = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit  = localMapPoints.begin(),
                                      vend = localMapPoints.end();
         vit != vend;
         vit++)
    {
        MapPoint *pMP = *vit;

        if (pMP->lastSeenFrameId == currentFrame.mnId)
            continue;
        if (pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if (currentFrame.isInFrustum(pMP, 0.5))
        {
            pMP->increaseVisible();
            nToMatch++;
        }
        if (pMP->trackInView)
        {
            currentFrame.projectedPoints[pMP->mnId] =
                cv::Point2f(pMP->trackProjX, pMP->trackProjY);
        }
    }

    if (nToMatch > 0)
    {
        ORBmatcher matcher(0.8);
        int        th = 1;
        if (sensor == System::RGBD || sensor == System::IMU_RGBD)
            th = 3;
        if (p_atlas->isImuInitialized())
        {
            if (p_atlas->getCurrentMap()->getInertialBA2())
                th = 2;
            else
                th = 6;
        }
        else if (!p_atlas->isImuInitialized() &&
                 (sensor == System::IMU_MONOCULAR ||
                  sensor == System::IMU_STEREO || sensor == System::IMU_RGBD))
        {
            th = 10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if (currentFrame.mnId < lastRelocFrameId + 2)
            th = 5;

        if (state == LOST ||
            state == RECENTLY_LOST) // Lost for less than 1 second
            th = 15;                // 15

        // AGGRESSIVE: Even wider search during degraded tracking in corridors
        // If we have very few inliers, expand search radius significantly
        if (matchesInliers < 30 && matchesInliers > 0)
        {
            th = std::min(th * 3, motionModelMaxSearchRadius);
            Verbose::printMess(
                "[Tracking] Expanded search radius to " + std::to_string(th) +
                    " (inliers: " + std::to_string(matchesInliers) + ")",
                Verbose::VERBOSITY_NORMAL);
        }

        // DEPTH-AIDED TRACKING: For RGB-D, use depth to guide matching window
        // In low-texture corridors, constrain search using known depth
        if ((sensor == System::RGBD || sensor == System::IMU_RGBD) &&
            currentFrame.depths.size() > 0)
        {
            // Depth-guided search: reduce search radius for points with
            // reliable depth This helps in repetitive corridors where visual
            // appearance is ambiguous
            matcher.searchByProjectionWithDepth(
                currentFrame,
                localMapPoints,
                th,
                p_localMapper->farPoints,
                p_localMapper->farPointsThreshold,
                depthThreshold);
        }
        else
        {
            matcher.searchByProjection(currentFrame,
                                       localMapPoints,
                                       th,
                                       p_localMapper->farPoints,
                                       p_localMapper->farPointsThreshold);
        }
    }
}

} // namespace core
} // namespace vs_graphs
