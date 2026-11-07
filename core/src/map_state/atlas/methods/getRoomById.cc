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
 * @file            getRoomById.cc
 *
 * @brief           Implements Atlas::getRoomById(), declared in
 * atlas/objects/AtlasClass.h.
 */

#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/scene_graph/Room/Room.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::getRoomById(int roomId_in, semantic::Room *&p_room_out)
{
    std::unique_lock<std::mutex> lock(atlasMutex);

    /* Init variable to extract room by the id from the current active map */
    semantic::Room *p_roomInCurrentMap = nullptr;

    /* From the current active map, extract the room by ID */
    if ((p_activeMap != nullptr) &&
        p_activeMap->getRoomById(roomId_in, p_roomInCurrentMap) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomById returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Output room, if the room is valid */
    if ((p_activeMap == nullptr) && p_roomInCurrentMap == nullptr)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: Current active map is nullptr, and current room is nullptr",
            __func__);
    }

    /* Output the room, even if it is nullptr */
    p_room_out = p_roomInCurrentMap;

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
