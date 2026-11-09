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
 * @file            copyRoomContextForMap.cc
 *
 * @brief           Implements Atlas::copyRoomContextForMap(), declared in
 *                  Atlas.h.
 */

#include "../private_functions/private_functions.h"
#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::copyRoomContextForMap(
    map::Map                                   *p_map_in,
    std::vector<semantic::RoomContextSnapshot> &roomContexts_out)
{
    std::vector<semantic::RoomContextSnapshot> snapshots;
    if (p_map_in == nullptr)
    {
        roomContexts_out = snapshots;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
    unsigned long mapIdValue{};
    if (p_map_in->getId(mapIdValue) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> rooms{};
    if (p_semanticsManager->getMapDetectedRoomsInIdOrder(mapIdValue, rooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapDetectedRoomsInIdOrder found no scene map for "
                     "the map; continuing without its detected rooms.",
                     __func__);
    }
    std::vector<semantic::Room *> candidateRooms{};
    if (p_semanticsManager->getMapCandidateRoomsInIdOrder(mapIdValue,
                                                          candidateRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapCandidateRoomsInIdOrder found no scene map "
                     "for the map; continuing without its candidate rooms.",
                     __func__);
    }
    rooms.insert(rooms.end(), candidateRooms.begin(), candidateRooms.end());
    for (semantic::Room *p_room : rooms)
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
            continue;
        semantic::RoomContextSnapshot snapshot;
        captureRoomContext(p_room, snapshot);
        snapshots.push_back(std::move(snapshot));
    }
    roomContexts_out = snapshots;
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
