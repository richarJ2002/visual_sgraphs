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

#include <iostream>

namespace vs_graphs
{
namespace core
{

bool Tracking::trackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    currentFrame.computeBagOfWords();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher         matcher(0.7, true);
    vector<MapPoint *> mapPointMatches;

    int nmatches =
        matcher.searchByBoW(p_referenceKF, currentFrame, mapPointMatches);

    if (nmatches < 8)
    {
        std::cout << "[Tracking] Warning: Less than 8 features matched!"
                  << std::endl;
        return false;
    }

    currentFrame.mapPoints = mapPointMatches;
    currentFrame.setPose(lastFrame.getPose());

    // mCurrentFrame.printPointDistribution();

    Optimizer::poseOptimization(&currentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int keyPointIndex = 0; keyPointIndex < currentFrame.keyPointCount;
         keyPointIndex++)
    {
        // if(i >= mCurrentFrame.Nleft) break;
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
                p_mapPoint->isTrackedInView = false;
                p_mapPoint->lastSeenFrameId = currentFrame.id;
                nmatches--;
            }
            else if (currentFrame.mapPoints[keyPointIndex]
                         ->getObservationCount() > 0)
                nmatchesMap++;
        }
    }

    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap >= 10;
}

} // namespace core
} // namespace vs_graphs
