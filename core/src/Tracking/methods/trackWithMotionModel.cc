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
        (currentFrame.mnId > lastRelocFrameId + framesToResetIMU))
    {
        // Predict state with IMU if it is initialized and it doesnt need reset
        predictStateIMU();
        return true;
    }
    else
    {
        currentFrame.setPose(velocity * lastFrame.getPose());
    }

    fill(currentFrame.mapPoints.begin(),
         currentFrame.mapPoints.end(),
         static_cast<MapPoint *>(nullptr));

    // Project points seen in previous frame
    int th;

    if (sensor == System::STEREO)
        th = 7;
    else
        th = 15;

    int nmatches = matcher.searchByProjection(
        currentFrame,
        lastFrame,
        th,
        sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR);

    // If few matches, use progressively wider window searches.
    int searchStep   = 1;
    int searchRadius = th;
    while (nmatches < 20 && searchRadius < motionModelMaxSearchRadius)
    {
        int expandedTh = static_cast<int>(std::ceil(
            th * (1.0F +
                  searchStep * (mfMotionModelSearchRadiusMultiplier - 1.0F))));
        expandedTh     = std::min(expandedTh, motionModelMaxSearchRadius);
        if (expandedTh <= searchRadius)
        {
            break;
        }

        Verbose::printMess("Not enough matches, wider window search (radius " +
                               std::to_string(expandedTh) + ")!!",
                           Verbose::VERBOSITY_NORMAL);
        fill(currentFrame.mapPoints.begin(),
             currentFrame.mapPoints.end(),
             static_cast<MapPoint *>(nullptr));

        nmatches = matcher.searchByProjection(
            currentFrame,
            lastFrame,
            expandedTh,
            sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR);
        Verbose::printMess("Matches with wider search: " + to_string(nmatches),
                           Verbose::VERBOSITY_NORMAL);
        searchRadius = expandedTh;
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
    for (int i = 0; i < currentFrame.N; i++)
    {
        if (currentFrame.mapPoints[i])
        {
            if (currentFrame.outlierFlags[i])
            {
                MapPoint *pMP = currentFrame.mapPoints[i];

                currentFrame.mapPoints[i]    = static_cast<MapPoint *>(nullptr);
                currentFrame.outlierFlags[i] = false;
                if (i < currentFrame.Nleft)
                {
                    pMP->trackInView = false;
                }
                else
                {
                    pMP->trackInViewR = false;
                }
                pMP->lastSeenFrameId = currentFrame.mnId;
                nmatches--;
            }
            else if (currentFrame.mapPoints[i]->getObservationCount() > 0)
                nmatchesMap++;
        }
    }

    if (onlyTracking)
    {
        visualOdometry = nmatchesMap < 10;
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
