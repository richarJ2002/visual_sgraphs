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

#include "MapPoint.h"

#include "ORBmatcher.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void MapPoint::updateNormalAndDepth()
{
    map<KeyFrame *, tuple<int, int>> observedKeyFrames;
    KeyFrame                        *p_localReferenceKeyFrame;
    Eigen::Vector3f                  Pos;
    {
        unique_lock<mutex> lock1(featuresMutex);
        unique_lock<mutex> lock2(positionMutex);
        if (isFlaggedBad)
            return;
        observedKeyFrames        = observations;
        p_localReferenceKeyFrame = p_referenceKeyFrame;
        Pos                      = worldPos;
    }

    if (observedKeyFrames.empty())
        return;

    Eigen::Vector3f normal;
    normal.setZero();
    int n = 0;
    for (map<KeyFrame *, tuple<int, int>>::iterator
             mit  = observedKeyFrames.begin(),
             mend = observedKeyFrames.end();
         mit != mend;
         mit++)
    {
        KeyFrame *p_keyFrame = mit->first;

        tuple<int, int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if (leftIndex != -1)
        {
            Eigen::Vector3f Owi     = p_keyFrame->getCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal                  = normal + normali / normali.norm();
            n++;
        }
        if (rightIndex != -1)
        {
            Eigen::Vector3f Owi     = p_keyFrame->getRightCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal                  = normal + normali / normali.norm();
            n++;
        }
    }

    Eigen::Vector3f PC = Pos - p_localReferenceKeyFrame->getCameraCenter();
    const float     distance = PC.norm();

    tuple<int, int> indexes   = observedKeyFrames[p_localReferenceKeyFrame];
    int             leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    int             level;
    if (p_localReferenceKeyFrame->leftKeyPointCount == -1)
    {
        level =
            p_localReferenceKeyFrame->keyPointsUndistorted[leftIndex].octave;
    }
    else if (leftIndex != -1)
    {
        level = p_localReferenceKeyFrame->keyPoints[leftIndex].octave;
    }
    else
    {
        level =
            p_localReferenceKeyFrame
                ->keyPointsRight[rightIndex -
                                 p_localReferenceKeyFrame->leftKeyPointCount]
                .octave;
    }

    // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =
        p_localReferenceKeyFrame->scaleFactors[level];
    const int levelCount = p_localReferenceKeyFrame->scaleLevelCount;

    {
        unique_lock<mutex> lock3(positionMutex);
        maxDistance = distance * levelScaleFactor;
        minDistance = maxDistance /
                      p_localReferenceKeyFrame->scaleFactors[levelCount - 1];
        normalVector = normal / n;
    }
}

} // namespace core
} // namespace vs_graphs
