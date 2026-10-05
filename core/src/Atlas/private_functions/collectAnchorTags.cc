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
 * @file            collectAnchorTags.cc
 *
 * @brief           Implements collectAnchorTags(), declared in
 *                  Atlas/private_functions.h.
 */

#include "Atlas.h"

#include "private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

AtlasStatus collectAnchorTags(Map                   *p_oldMap_in,
                              Map                   *p_currentMap_in,
                              std::set<std::string> &anchorTags_out)
{
    std::set<std::string> anchorTags;
    if (p_oldMap_in == nullptr || p_currentMap_in == nullptr)
    {
        anchorTags_out = anchorTags;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
    std::set<std::string>         currentTags;
    std::vector<semantic::Room *> currentMapAllRooms{};
    if (p_currentMap_in->getAllRooms(currentMapAllRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (semantic::Room *p_room : currentMapAllRooms)
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
        bool roomHasRoomTag{};
        if ((p_room != nullptr && !roomIsBad) &&
            p_room->hasRoomTag(roomHasRoomTag) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::string roomTag{};
        if ((p_room != nullptr && !roomIsBad && roomHasRoomTag) &&
            p_room->getRoomTag(roomTag) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad && roomHasRoomTag &&
            !roomTag.empty())
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
            currentTags.insert(roomTag2);
        }
    }
    std::vector<semantic::Room *> oldMapAllRooms{};
    if (p_oldMap_in->getAllRooms(oldMapAllRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (semantic::Room *p_room : oldMapAllRooms)
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
        bool roomHasRoomTag2{};
        if ((p_room != nullptr && !roomIsBad2) &&
            p_room->hasRoomTag(roomHasRoomTag2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::string roomTag3{};
        if ((p_room != nullptr && !roomIsBad2 && roomHasRoomTag2) &&
            p_room->getRoomTag(roomTag3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::string roomTag4{};
        if ((p_room != nullptr && !roomIsBad2 && roomHasRoomTag2 &&
             !roomTag3.empty()) &&
            p_room->getRoomTag(roomTag4) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad2 && roomHasRoomTag2 &&
            !roomTag3.empty() && currentTags.count(roomTag4) > 0U)
        {
            std::string roomTag5{};
            if (p_room->getRoomTag(roomTag5) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomTag returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            anchorTags.insert(roomTag5);
        }
    }
    anchorTags_out = anchorTags;
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
