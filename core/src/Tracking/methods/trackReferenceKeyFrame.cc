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
    vector<MapPoint *> vpMapPointMatches;

    int nmatches =
        matcher.searchByBoW(p_referenceKF, currentFrame, vpMapPointMatches);

    if (nmatches < 8)
    {
        std::cout << "[Tracking] Warning: Less than 8 features matched!"
                  << std::endl;
        return false;
    }

    currentFrame.mapPoints = vpMapPointMatches;
    currentFrame.setPose(lastFrame.getPose());

    // mCurrentFrame.printPointDistribution();

    Optimizer::poseOptimization(&currentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < currentFrame.N; i++)
    {
        // if(i >= mCurrentFrame.Nleft) break;
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
                pMP->trackInView     = false;
                pMP->lastSeenFrameId = currentFrame.mnId;
                nmatches--;
            }
            else if (currentFrame.mapPoints[i]->getObservationCount() > 0)
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
