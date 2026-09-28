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

void Map::preSave(
    std::set<camera_models::geometriccamera::GeometricCamera *> &cams_inout)
{
    int mapPointWithoutObservationCount = 0;

    std::set<MapPoint *> temporaryMspMapPoints1;
    temporaryMspMapPoints1.insert(mapPoints.begin(), mapPoints.end());

    for (MapPoint *p_mapPoint : temporaryMspMapPoints1)
    {
        if (!p_mapPoint || p_mapPoint->isBad())
            continue;

        if (p_mapPoint->getObservations().size() == 0)
        {
            mapPointWithoutObservationCount++;
        }
        map<KeyFrame *, std::tuple<int, int>> observations =
            p_mapPoint->getObservations();
        for (map<KeyFrame *, std::tuple<int, int>>::iterator
                 observationIt = observations.begin(),
                 end           = observations.end();
             observationIt != end;
             ++observationIt)
        {
            if (observationIt->first->getMap() != this ||
                observationIt->first->isBad())
            {
                p_mapPoint->eraseObservation(observationIt->first);
            }
        }
    }

    // Saves the id of KF origins
    backupKeyFrameOriginIds.clear();
    backupKeyFrameOriginIds.reserve(keyFrameOrigins.size());
    for (int elIndex = 0, elCount = keyFrameOrigins.size(); elIndex < elCount;
         ++elIndex)
    {
        backupKeyFrameOriginIds.push_back(keyFrameOrigins[elIndex]->id);
    }

    // Backup of MapPoints
    backupMapPoints.clear();

    std::set<MapPoint *> temporaryMspMapPoints2;
    temporaryMspMapPoints2.insert(mapPoints.begin(), mapPoints.end());

    for (MapPoint *p_mapPoint : temporaryMspMapPoints2)
    {
        if (!p_mapPoint || p_mapPoint->isBad())
            continue;

        backupMapPoints.push_back(p_mapPoint);
        p_mapPoint->preSave(keyFrames, mapPoints);
    }

    // Backup of KeyFrames
    backupKeyFrames.clear();
    for (KeyFrame *p_keyFrame : keyFrames)
    {
        if (!p_keyFrame || p_keyFrame->isBad())
            continue;

        backupKeyFrames.push_back(p_keyFrame);
        p_keyFrame->preSave(keyFrames, mapPoints, cams_inout);
    }

    backupInitialKeyFrameId = -1;
    if (p_initialKeyFrame)
    {
        backupInitialKeyFrameId = p_initialKeyFrame->id;
    }

    backupLowerKeyFrameId = -1;
    if (p_lowerIdKeyFrame)
    {
        backupLowerKeyFrameId = p_lowerIdKeyFrame->id;
    }
}

} // namespace core
} // namespace vs_graphs
