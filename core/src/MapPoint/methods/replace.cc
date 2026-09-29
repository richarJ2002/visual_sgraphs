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

#include "Map.h"
#include "ORBmatcher.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void MapPoint::replace(MapPoint *p_mapPoint_inout)
{
    if (p_mapPoint_inout->id == this->id)
        return;

    int                              nvisible, nfound;
    map<KeyFrame *, tuple<int, int>> observation;
    {
        unique_lock<mutex> lock1(featuresMutex);
        unique_lock<mutex> lock2(positionMutex);
        observation = observations;
        observations.clear();
        isFlaggedBad = true;
        nvisible     = visibleCount;
        nfound       = foundCount;
        p_replaced   = p_mapPoint_inout;
    }

    for (map<KeyFrame *, tuple<int, int>>::iterator mit  = observation.begin(),
                                                    mend = observation.end();
         mit != mend;
         mit++)
    {
        // Replace measurement in keyframe
        KeyFrame *p_keyFrame = mit->first;

        tuple<int, int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if (!p_mapPoint_inout->isInKeyFrame(p_keyFrame))
        {
            if (leftIndex != -1)
            {
                p_keyFrame->replaceMapPointMatch(leftIndex, p_mapPoint_inout);
                p_mapPoint_inout->addObservation(p_keyFrame, leftIndex);
            }
            if (rightIndex != -1)
            {
                p_keyFrame->replaceMapPointMatch(rightIndex, p_mapPoint_inout);
                p_mapPoint_inout->addObservation(p_keyFrame, rightIndex);
            }
        }
        else
        {
            if (leftIndex != -1)
            {
                p_keyFrame->eraseMapPointMatch(leftIndex);
            }
            if (rightIndex != -1)
            {
                p_keyFrame->eraseMapPointMatch(rightIndex);
            }
        }
    }
    p_mapPoint_inout->increaseFound(nfound);
    p_mapPoint_inout->increaseVisible(nvisible);
    p_mapPoint_inout->computeDistinctiveDescriptors();

    p_map->eraseMapPoint(this);
}

} // namespace core
} // namespace vs_graphs
