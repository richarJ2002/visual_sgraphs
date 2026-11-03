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
 * @file            exportRoomContextFromCurrentMap.cc
 *
 * @brief           Implements Atlas::exportRoomContextFromCurrentMap(),
 *                  declared in atlas/objects/AtlasClass.h.
 */

#include "../private_functions/private_functions.h"
#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantic/floor/floor.h"
#include "scene_understanding/semantic/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/semantic/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/semantic/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantic/passage/passage.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::exportRoomContextFromCurrentMap()
{
    if (!p_activeMap)
        return AtlasStatus::ATLAS_STATUS_SUCCESS;

    /* Export BOTH confirmed detected rooms AND candidate/marker-based rooms.
     * Candidate rooms (prospective/provisional) may not have full wall loops
     * yet but still carry spatial identity needed for cross-restart matching.
     */
    std::vector<semantic::Room *> rooms{};
    if (p_activeMap->getAllDetectedMapRooms(rooms) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllDetectedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> candidateRooms{};
    if (p_activeMap->getAllCandidateMapRooms(candidateRooms) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllCandidateMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    rooms.insert(rooms.end(), candidateRooms.begin(), candidateRooms.end());

    if (rooms.empty())
        return AtlasStatus::ATLAS_STATUS_SUCCESS;

    std::vector<semantic::RoomContextSnapshot> snapshots;
    snapshots.reserve(rooms.size());

    unsigned long mapIdValue{};
    if (p_activeMap->getId(mapIdValue) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const long unsigned int mapId = static_cast<long unsigned int>(mapIdValue);

    for (semantic::Room *p_room : rooms)
    {
        bool roomIsBad{};
        if (!(!p_room) && p_room->isBad(roomIsBad) !=
                              semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_room || roomIsBad)
            continue;

        semantic::RoomContextSnapshot snap;
        captureRoomContext(p_room, snap);
        bool roomHasPreviouslyVisited{};
        if (p_room->hasPreviouslyVisited(roomHasPreviouslyVisited) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasPreviouslyVisited returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        snap.wasPreviouslyVisited = roomHasPreviouslyVisited;

        /* Assign persistent tag to old map rooms for merge trigger.
         * Tag format: "room_<id>".
         */
        int roomId3{};
        if (p_room->getId(roomId3) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const std::string roomTag = "room_" + std::to_string(roomId3);
        snap.roomTag              = roomTag;
        bool roomHasRoomTag{};
        if (p_room->hasRoomTag(roomHasRoomTag) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!roomHasRoomTag)
        {
            if (p_room->setRoomTag(roomTag) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setRoomTag returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        snapshots.push_back(snap);
    }

    const std::size_t exportedRoomCount = snapshots.size();
    {
        std::lock_guard<std::mutex> contextLock(roomContextMutex);
        roomContextHistory[mapId] = std::move(snapshots);
    }

    /* Record the departure room for mission-chain tracing: the room the UAV
     * was following when this map was stranded. */
    int departureRoomId{};
    if (getCurrentSemanticRoomIdentity(departureRoomId) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentSemanticRoomIdentity returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    if (departureRoomId >= 0)
    {
        for (semantic::Room *p_room : rooms)
        {
            bool roomIsBad2{};
            if ((p_room != nullptr) &&
                p_room->isBad(roomIsBad2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int roomId4{};
            if ((p_room != nullptr && !roomIsBad2) &&
                p_room->getId(roomId4) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room != nullptr && !roomIsBad2 && roomId4 == departureRoomId)
            {
                if (p_activeMap->setFinalRoom(p_room) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setFinalRoom returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                break;
            }
        }
    }

    std::cout << "[Atlas] Exported room context: " << exportedRoomCount
              << " rooms (mapId: " << mapId << ")" << std::endl;

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
