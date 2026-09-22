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
    KeyFrame                        *pRefKF;
    Eigen::Vector3f                  Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if (mbBad)
            return;
        observedKeyFrames = observations;
        pRefKF            = p_referenceKeyFrame;
        Pos               = worldPos;
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
        KeyFrame *pKF = mit->first;

        tuple<int, int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if (leftIndex != -1)
        {
            Eigen::Vector3f Owi     = pKF->getCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal                  = normal + normali / normali.norm();
            n++;
        }
        if (rightIndex != -1)
        {
            Eigen::Vector3f Owi     = pKF->getRightCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal                  = normal + normali / normali.norm();
            n++;
        }
    }

    Eigen::Vector3f PC   = Pos - pRefKF->getCameraCenter();
    const float     dist = PC.norm();

    tuple<int, int> indexes   = observedKeyFrames[pRefKF];
    int             leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    int             level;
    if (pRefKF->Nleft == -1)
    {
        level = pRefKF->keyPointsUndistorted[leftIndex].octave;
    }
    else if (leftIndex != -1)
    {
        level = pRefKF->keyPoints[leftIndex].octave;
    }
    else
    {
        level = pRefKF->keyPointsRight[rightIndex - pRefKF->Nleft].octave;
    }

    // const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor = pRefKF->scaleFactors[level];
    const int   nLevels          = pRefKF->scaleLevelCount;

    {
        unique_lock<mutex> lock3(mMutexPos);
        maxDistance  = dist * levelScaleFactor;
        minDistance  = maxDistance / pRefKF->scaleFactors[nLevels - 1];
        normalVector = normal / n;
    }
}

} // namespace core
} // namespace vs_graphs
