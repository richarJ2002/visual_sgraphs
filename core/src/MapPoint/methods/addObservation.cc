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

MapPointStatus MapPoint::addObservation(KeyFrame *p_keyFrame_inout,
                                        int       index_in)
{
    std::unique_lock<std::mutex> lock(featuresMutex);
    std::tuple<int, int>         indexes;

    if (observations.count(p_keyFrame_inout))
    {
        indexes = observations[p_keyFrame_inout];
    }
    else
    {
        indexes = std::tuple<int, int>(-1, -1);
    }

    if (p_keyFrame_inout->leftKeyPointCount != -1 &&
        index_in >= p_keyFrame_inout->leftKeyPointCount)
    {
        std::get<1>(indexes) = index_in;
    }
    else
    {
        std::get<0>(indexes) = index_in;
    }

    observations[p_keyFrame_inout] = indexes;

    if (!p_keyFrame_inout->p_camera2 && p_keyFrame_inout->uRight[index_in] >= 0)
        observationCount += 2;
    else
        observationCount++;

    return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
