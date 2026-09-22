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

void MapPoint::replace(MapPoint *pMP)
{
    if (pMP->mnId == this->mnId)
        return;

    int                              nvisible, nfound;
    map<KeyFrame *, tuple<int, int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs = observations;
        observations.clear();
        mbBad      = true;
        nvisible   = visibleCount;
        nfound     = foundCount;
        p_replaced = pMP;
    }

    for (map<KeyFrame *, tuple<int, int>>::iterator mit  = obs.begin(),
                                                    mend = obs.end();
         mit != mend;
         mit++)
    {
        // Replace measurement in keyframe
        KeyFrame *pKF = mit->first;

        tuple<int, int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if (!pMP->isInKeyFrame(pKF))
        {
            if (leftIndex != -1)
            {
                pKF->replaceMapPointMatch(leftIndex, pMP);
                pMP->addObservation(pKF, leftIndex);
            }
            if (rightIndex != -1)
            {
                pKF->replaceMapPointMatch(rightIndex, pMP);
                pMP->addObservation(pKF, rightIndex);
            }
        }
        else
        {
            if (leftIndex != -1)
            {
                pKF->eraseMapPointMatch(leftIndex);
            }
            if (rightIndex != -1)
            {
                pKF->eraseMapPointMatch(rightIndex);
            }
        }
    }
    pMP->increaseFound(nfound);
    pMP->increaseVisible(nvisible);
    pMP->computeDistinctiveDescriptors();

    p_map->eraseMapPoint(this);
}

} // namespace core
} // namespace vs_graphs
