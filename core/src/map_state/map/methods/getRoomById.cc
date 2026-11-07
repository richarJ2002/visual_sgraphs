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
 * @brief           Implements Map::getRoomById(), declared in
 * map/objects/MapClass.h.
 */

#include "map_state/map/objects/MapClass.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus Map::getRoomById(int roomId_in, semantic::Room *&p_roomById_out)
{
    std::unique_lock<std::mutex> lock(mapMutex);
    p_roomById_out = nullptr;

    /* Detected rooms come before marker-based rooms, as in getAllRooms() */
    std::vector<semantic::Room *> allRooms{};
    allRooms.insert(allRooms.end(), detectedRooms.begin(), detectedRooms.end());
    allRooms.insert(allRooms.end(),
                    markerBasedRooms.begin(),
                    markerBasedRooms.end());
    for (semantic::Room *p_room : allRooms)
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int roomId{};
        if ((p_room != nullptr && !roomIsBad) &&
            p_room->getId(roomId) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad && roomId == roomId_in)
        {
            p_roomById_out = p_room;
            break;
        }
    }
    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
