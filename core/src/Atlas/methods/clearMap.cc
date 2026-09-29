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

#include "Atlas.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void Atlas::clearMap()
{
    unique_lock<mutex> atlasLock(atlasMutex);
    /* Same-map reset (Tracking::ResetActiveMap) wipes rooms/floors/passages
     * from the live Map object without creating a new Map. Snapshot first so
     * the bootstrap recovery path can recreate the same stable identities
     * afterwards; otherwise the next cycle allocates fresh RoomN/FloorM. */
    exportRoomContextFromCurrentMap();
    if (p_activeMap->clear() != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clear returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    /* A same-map clear keeps the map id, so the visualization/voxblox
     * revision token would not observe the reset and stale markers and
     * clouds would persist alongside the fresh map. A clear is at least as
     * big a change as the loop-closure corrections this index exists for. */
    if (p_activeMap->informNewBigChange() != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: informNewBigChange returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

} // namespace core
} // namespace vs_graphs
