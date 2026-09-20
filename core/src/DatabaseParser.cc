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

/*!
 * @file         DatabaseParser.cc
 *
 * @brief        Implements DBParser declared in DatabaseParser.h.
 */

#include "DatabaseParser.h"

#include "System.h"

namespace vs_graphs
{
namespace core
{
DBParser::DBParser() {}
DBParser::~DBParser() {}

json DBParser::parseJsonFile(std::string jsonFilePath_in)
{
    try
    {
        VSLAM_LOG_INFO("- Loading JSON data from %s\n",
                       jsonFilePath_in.c_str());
        // Reading the JSON file from the given path
        ifstream jsonFile(jsonFilePath_in);
        // Parsing the JSON file to get the envrionment data
        json     environmentData = json::parse(jsonFile);
        // Return parsed data
        return environmentData;
    }
    catch (json::parse_error &ex)
    {
        VSLAM_LOG_ERROR("- Error parsing the environment JSON file: %s\n",
                        ex.what());
        VSLAM_LOG_ERROR("- Exiting ... \n\n");
        exit(1);
    }
}

std::vector<semantic::Room *>
    DBParser::getEnvironmentRooms(json environmentData_in)
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
            p_environmentRoom->setOpId(-1);
            p_environmentRoom->setOpIdG(-1);
            p_environmentRoom->setId(stoi(environmentDatum.key()));
            p_environmentRoom->setName(environmentDatum.value()["name"]);
            p_environmentRoom->setMetaMarkerId(
                environmentDatum.value()["metaMarker"]);

            // Set the room variant (corridors are incomplete rooms, not a
            // distinct semantic type, so every env room is a plain ROOM)
            p_environmentRoom->setRoomVariant(
                semantic::Room::RoomVariant::ROOM);

            // Fill the vector
            environmentRooms.push_back(p_environmentRoom);
        }

        // Print the loaded rooms
        VSLAM_LOG_INFO("- Fetched %d rooms from the JSON file! [e.g., '%s'].\n",
                       static_cast<int>(environmentRooms.size()),
                       environmentRooms[0]->getName().c_str());
    }
    else
        VSLAM_LOG_INFO("- No rooms found in the JSON file!\n");

    return environmentRooms;
}
} // namespace core
} // namespace vs_graphs
