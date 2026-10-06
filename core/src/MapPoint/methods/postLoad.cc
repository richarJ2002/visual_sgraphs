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

/*!
 * @file            postLoad.cc
 *
 * @brief           Implements MapPoint::postLoad(), declared in MapPoint.h.
 */

#include "MapPoint.h"

#include "ORBmatcher.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

MapPointStatus
    MapPoint::postLoad(std::map<long unsigned int, KeyFrame *> &keyFrameId_in,
                       std::map<long unsigned int, MapPoint *> &mapPointId_in)
{
    p_referenceKeyFrame = keyFrameId_in[backupRefKeyFrameId];
    if (!p_referenceKeyFrame)
    {
        std::cout << "ERROR: MP without KF reference " << backupRefKeyFrameId
                  << "; Num obs: " << observationCount << std::endl;
    }
    p_replaced = static_cast<MapPoint *>(nullptr);
    if (backupReplacedId >= 0)
    {
        std::map<long unsigned int, MapPoint *>::iterator mapPointIdIt =
            mapPointId_in.find(backupReplacedId);
        if (mapPointIdIt != mapPointId_in.end())
            p_replaced = mapPointIdIt->second;
    }

    observations.clear();

    for (std::map<long unsigned int, int>::const_iterator
             mapPointIdIt = backupObservationIds1.begin(),
             end          = backupObservationIds1.end();
         mapPointIdIt != end;
         ++mapPointIdIt)
    {
        KeyFrame *p_keyFrame = keyFrameId_in[mapPointIdIt->first];
        std::map<long unsigned int, int>::const_iterator it2 =
            backupObservationIds2.find(mapPointIdIt->first);
        /* A missing right index means the right camera did not see it. */
        const int rightIndex =
            it2 != backupObservationIds2.end() ? it2->second : -1;
        std::tuple<int, int> indexes =
            std::tuple<int, int>(mapPointIdIt->second, rightIndex);
        if (p_keyFrame)
        {
            observations[p_keyFrame] = indexes;
        }
    }

    backupObservationIds1.clear();
    backupObservationIds2.clear();

    return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
