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
 * @file            indexAbsorbedRoomWalls.cc
 *
 * @brief           Implements indexAbsorbedRoomWalls(), declared in
 *                  graph/merging/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/merging/private_functions/private_functions.h"

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void indexAbsorbedRoomWalls(SemanticsManager *p_semanticsManager_inout,
                            map::Map         *p_currentMap_inout)
{
    /* Rebuild imported room-wall index entries before fusion. */
    unsigned long currentMapId{};
    if (p_currentMap_inout->getId(currentMapId) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> currentMapAllRooms{};
    if (p_semanticsManager_inout->getMapRoomsInIdOrder(currentMapId,
                                                       currentMapAllRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: the semantics manager has no scene graph for map %lu; "
            "continuing without its rooms.",
            __func__,
            currentMapId);
    }
    for (semantic::Room *p_room : currentMapAllRooms)
    {
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad)
        {
            continue;
        }

        std::vector<geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (geometric::Plane *p_wall : roomWalls)
        {
            bool wallIsBad{};
            if ((p_wall != nullptr) &&
                p_wall->isBad(wallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_wall != nullptr && !wallIsBad)
            {
                if (p_currentMap_inout->addRoomWallPlane(p_wall) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addRoomWallPlane returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
