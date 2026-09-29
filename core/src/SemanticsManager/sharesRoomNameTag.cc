/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "SemanticsManager.h"

#include "private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief       Tests whether two maps observe a common tagged room name.
 *
 *              A new map is a deterministic merge candidate for the current
 *              map only when at least one non-empty room tag collected from
 *              the other map's detected and marker-based rooms also appears
 *              among the current map's tagged rooms. Room tags originate from
 *              context snapshots and are propagated by matchRoomsToContext,
 *              so they are a stable correspondences key between maps.
 *
 * @param[in]   p_firstMap_in
 *              Map whose detected and marker-based room tags are collected.
 * @param[in]   p_secondMap_in
 *              Map whose tagged rooms are tested against the collected tags.
 *
 * @return      True when both maps observe at least one shared room tag.
 */
bool sharesRoomNameTag(Map *p_firstMap_in, Map *p_secondMap_in)
{
    std::unordered_set<std::string> firstMapRoomTags;
    for (semantic::Room *p_room : p_firstMap_in->getAllDetectedMapRooms())
    {
        bool roomHasRoomTag{};
        if (p_room->hasRoomTag(roomHasRoomTag) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (roomHasRoomTag)
        {
            std::string roomTag{};
            if (p_room->getRoomTag(roomTag) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomTag returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            firstMapRoomTags.insert(roomTag);
        }
    }
    for (semantic::Room *p_room : p_firstMap_in->getAllMarkerBasedMapRooms())
    {
        bool roomHasRoomTag2{};
        if (p_room->hasRoomTag(roomHasRoomTag2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (roomHasRoomTag2)
        {
            std::string roomTag2{};
            if (p_room->getRoomTag(roomTag2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomTag returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            firstMapRoomTags.insert(roomTag2);
        }
    }

    if (firstMapRoomTags.empty())
    {
        std::cout << "[SemMgr] sharesRoomNameTag: first map (id="
                  << p_firstMap_in->getId()
                  << ") has NO tagged rooms (detected="
                  << p_firstMap_in->getAllDetectedMapRooms().size()
                  << ", marker="
                  << p_firstMap_in->getAllMarkerBasedMapRooms().size() << ")"
                  << std::endl;
        return false;
    }

    for (semantic::Room *p_room : p_secondMap_in->getAllDetectedMapRooms())
    {
        bool roomHasRoomTag3{};
        if (p_room->hasRoomTag(roomHasRoomTag3) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::string roomTag3{};
        if ((roomHasRoomTag3) && p_room->getRoomTag(roomTag3) !=
                                     semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (roomHasRoomTag3 && firstMapRoomTags.count(roomTag3) != 0U)
        {
            std::string roomTag4{};
            if (p_room->getRoomTag(roomTag4) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomTag returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] sharesRoomNameTag: MATCH found tag "
                      << roomTag4 << " between maps " << p_firstMap_in->getId()
                      << " and " << p_secondMap_in->getId() << std::endl;
            return true;
        }
    }
    for (semantic::Room *p_room : p_secondMap_in->getAllMarkerBasedMapRooms())
    {
        bool roomHasRoomTag4{};
        if (p_room->hasRoomTag(roomHasRoomTag4) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::string roomTag5{};
        if ((roomHasRoomTag4) && p_room->getRoomTag(roomTag5) !=
                                     semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (roomHasRoomTag4 && firstMapRoomTags.count(roomTag5) != 0U)
        {
            std::string roomTag6{};
            if (p_room->getRoomTag(roomTag6) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomTag returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] sharesRoomNameTag: MATCH found tag "
                      << roomTag6 << " between maps " << p_firstMap_in->getId()
                      << " and " << p_secondMap_in->getId() << std::endl;
            return true;
        }
    }

    std::cout << "[SemMgr] sharesRoomNameTag: NO match. First map (id="
              << p_firstMap_in->getId() << ") tags: " << firstMapRoomTags.size()
              << ", second map (id=" << p_secondMap_in->getId() << ") detected="
              << p_secondMap_in->getAllDetectedMapRooms().size() << " marker="
              << p_secondMap_in->getAllMarkerBasedMapRooms().size()
              << std::endl;

    return false;
}

} // namespace core
} // namespace vs_graphs
