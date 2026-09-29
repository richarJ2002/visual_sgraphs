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

MapPointStatus MapPoint::predictScale(const float &currentDistance_in,
                                      Frame       *p_pF_in,
                                      int         &scaleLevel_out)
{
    if (currentDistance_in == 0.0f)
    {
        scaleLevel_out = 0;
        return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
    }

    float ratio;
    {
        unique_lock<mutex> lock(positionMutex);
        ratio = maxDistance / currentDistance_in;
    }

    int scaleCount = ceil(log(ratio) / p_pF_in->logScaleFactor);
    if (scaleCount < 0)
        scaleCount = 0;
    else if (scaleCount >= p_pF_in->scaleLevelCount)
        scaleCount = p_pF_in->scaleLevelCount - 1;

    scaleLevel_out = scaleCount;
    return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
