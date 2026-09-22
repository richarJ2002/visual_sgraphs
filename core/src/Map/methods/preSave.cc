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

#include "Map.h"

#include <algorithm>
#include <iterator>
#include <mutex>

namespace vs_graphs
{
namespace core
{

void Map::PreSave(
    std::set<camera_models::geometriccamera::GeometricCamera *> &spCams)
{
    int nMPWithoutObs = 0;

    std::set<MapPoint *> tmp_mspMapPoints1;
    tmp_mspMapPoints1.insert(mapPoints.begin(), mapPoints.end());

    for (MapPoint *pMPi : tmp_mspMapPoints1)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        if (pMPi->getObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame *, std::tuple<int, int>> observations =
            pMPi->getObservations();
        for (map<KeyFrame *, std::tuple<int, int>>::iterator
                 it  = observations.begin(),
                 end = observations.end();
             it != end;
             ++it)
        {
            if (it->first->getMap() != this || it->first->isBad())
            {
                pMPi->eraseObservation(it->first);
            }
        }
    }

    // Saves the id of KF origins
    backupKeyFrameOriginIds.clear();
    backupKeyFrameOriginIds.reserve(keyFrameOrigins.size());
    for (int i = 0, numEl = keyFrameOrigins.size(); i < numEl; ++i)
    {
        backupKeyFrameOriginIds.push_back(keyFrameOrigins[i]->mnId);
    }

    // Backup of MapPoints
    backupMapPoints.clear();

    std::set<MapPoint *> tmp_mspMapPoints2;
    tmp_mspMapPoints2.insert(mapPoints.begin(), mapPoints.end());

    for (MapPoint *pMPi : tmp_mspMapPoints2)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        backupMapPoints.push_back(pMPi);
        pMPi->PreSave(keyFrames, mapPoints);
    }

    // Backup of KeyFrames
    backupKeyFrames.clear();
    for (KeyFrame *pKFi : keyFrames)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        backupKeyFrames.push_back(pKFi);
        pKFi->PreSave(keyFrames, mapPoints, spCams);
    }

    backupInitialKeyFrameId = -1;
    if (p_initialKeyFrame)
    {
        backupInitialKeyFrameId = p_initialKeyFrame->mnId;
    }

    backupLowerKeyFrameId = -1;
    if (p_lowerIdKeyFrame)
    {
        backupLowerKeyFrameId = p_lowerIdKeyFrame->mnId;
    }
}

} // namespace core
} // namespace vs_graphs
