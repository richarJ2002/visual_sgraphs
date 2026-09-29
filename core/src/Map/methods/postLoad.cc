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

#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "MapPoint.h"

#include <algorithm>
#include <iterator>
#include <mutex>

namespace vs_graphs
{
namespace core
{

void Map::postLoad(
    KeyFrameDatabase *p_keyFrameDatabase_inout,
    ORBVocabulary *
        p_orbVocabulary_in /*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/
    ,
    map<unsigned int, camera_models::geometriccamera::GeometricCamera *>
        &cams_inout)
{
    std::copy(backupMapPoints.begin(),
              backupMapPoints.end(),
              std::inserter(mapPoints, mapPoints.begin()));
    std::copy(backupKeyFrames.begin(),
              backupKeyFrames.end(),
              std::inserter(keyFrames, keyFrames.begin()));

    map<long unsigned int, MapPoint *> mapPointId;
    for (MapPoint *p_mapPoint : mapPoints)
    {
        if (!p_mapPoint || p_mapPoint->isBad())
            continue;

        p_mapPoint->updateMap(this);
        mapPointId[p_mapPoint->id] = p_mapPoint;
    }

    map<long unsigned int, KeyFrame *> keyFrameId;
    for (KeyFrame *p_keyFrame : keyFrames)
    {
        if (!p_keyFrame || p_keyFrame->isBad())
            continue;

        p_keyFrame->updateMap(this);
        p_keyFrame->setORBVocabulary(p_orbVocabulary_in);
        p_keyFrame->setKeyFrameDatabase(p_keyFrameDatabase_inout);
        keyFrameId[p_keyFrame->id] = p_keyFrame;
    }

    // References reconstruction between different instances
    for (MapPoint *p_mapPoint : mapPoints)
    {
        if (!p_mapPoint || p_mapPoint->isBad())
            continue;

        p_mapPoint->postLoad(keyFrameId, mapPointId);
    }

    for (KeyFrame *p_keyFrame : keyFrames)
    {
        if (!p_keyFrame || p_keyFrame->isBad())
            continue;

        p_keyFrame->postLoad(keyFrameId, mapPointId, cams_inout);
        p_keyFrameDatabase_inout->add(p_keyFrame);
    }

    if (backupInitialKeyFrameId != -1)
    {
        p_initialKeyFrame = keyFrameId[backupInitialKeyFrameId];
    }

    if (backupLowerKeyFrameId != -1)
    {
        p_lowerIdKeyFrame = keyFrameId[backupLowerKeyFrameId];
    }

    keyFrameOrigins.clear();
    keyFrameOrigins.reserve(backupKeyFrameOriginIds.size());
    for (std::size_t backupKeyFrameOriginIdIndex = 0;
         backupKeyFrameOriginIdIndex < backupKeyFrameOriginIds.size();
         ++backupKeyFrameOriginIdIndex)
    {
        keyFrameOrigins.push_back(
            keyFrameId[backupKeyFrameOriginIds[backupKeyFrameOriginIdIndex]]);
    }

    backupMapPoints.clear();
}

} // namespace core
} // namespace vs_graphs
