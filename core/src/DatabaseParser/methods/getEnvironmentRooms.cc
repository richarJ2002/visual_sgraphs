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
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNSS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "DatabaseParser.h"

#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

DBParserStatus DBParser::getEnvironmentRooms(
    Json                           environmentData_in,
    std::vector<semantic::Room *> &environmentRooms_out)
{
    environmentRooms.clear();

    // Check if the JSON file contains rooms
    if (environmentData_in["rooms"].size() != 0)
    {
        // items() yields a library-internal proxy type with no public name
        // (docs/design/auto_policy.md, section 5), so it is deduced here.
        for (const auto &environmentDatum : environmentData_in["rooms"].items())
        {
            // Initialization
            semantic::Room *p_environmentRoom = new semantic::Room();

            // Fill the room entity
            if (p_environmentRoom->setOpId(-1) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setOpId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_environmentRoom->setOpIdG(-1) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setOpIdG returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_environmentRoom->setId(std::stoi(environmentDatum.key())) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_environmentRoom->setName(environmentDatum.value()["name"]) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setName returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_environmentRoom->setMetaMarkerId(
                    environmentDatum.value()["metaMarker"]) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMetaMarkerId returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            // Set the room variant (corridors are incomplete rooms, not a
            // distinct semantic type, so every env room is a plain ROOM)
            if (p_environmentRoom->setRoomVariant(
                    semantic::Room::RoomVariant::ROOM) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            // Fill the vector
            environmentRooms.push_back(p_environmentRoom);
        }

        // Print the loaded rooms
        std::string name2{};
        if (environmentRooms[0]->getName(name2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getName returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        VSLAM_LOG_INFO("- Fetched %d rooms from the JSON file! [e.g., '%s'].\n",
                       static_cast<int>(environmentRooms.size()),
                       name2.c_str());
    }
    else
        VSLAM_LOG_INFO("- No rooms found in the JSON file!\n");

    environmentRooms_out = environmentRooms;
    return DBParserStatus::DBPARSER_STATUS_SUCCESS;
}
} // namespace core
} // namespace vs_graphs
