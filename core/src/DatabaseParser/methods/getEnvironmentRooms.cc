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
        for (const auto &environmentDatum : environmentData_in["rooms"].items())
        {
            // Initialization
            semantic::Room *p_environmentRoom = new semantic::Room();

            // Fill the room entity
            if (p_environmentRoom->setOpId(-1) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setOpId cannot fail; continue as before.
            }
            if (p_environmentRoom->setOpIdG(-1) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setOpIdG cannot fail; continue as before.
            }
            if (p_environmentRoom->setId(stoi(environmentDatum.key())) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setId cannot fail; continue as before.
            }
            if (p_environmentRoom->setName(environmentDatum.value()["name"]) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setName cannot fail; continue as before.
            }
            if (p_environmentRoom->setMetaMarkerId(
                    environmentDatum.value()["metaMarker"]) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setMetaMarkerId cannot fail; continue as before.
            }

            // Set the room variant (corridors are incomplete rooms, not a
            // distinct semantic type, so every env room is a plain ROOM)
            if (p_environmentRoom->setRoomVariant(
                    semantic::Room::RoomVariant::ROOM) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setRoomVariant cannot fail; continue as before.
            }

            // Fill the vector
            environmentRooms.push_back(p_environmentRoom);
        }

        // Print the loaded rooms
        std::string name2{};
        if (environmentRooms[0]->getName(name2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getName cannot fail; continue as before.
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
