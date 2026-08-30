/**
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

#include "DatabaseParser.h"

#include "System.h"

namespace ORB_SLAM3
{
DBParser::DBParser() {}
DBParser::~DBParser() {}

json DBParser::jsonParser(string jsonFilePath)
{
    try
    {
        VSLAM_LOG_INFO("- Loading JSON data from %s\n", jsonFilePath.c_str());
        // Reading the JSON file from the given path
        ifstream jsonFile(jsonFilePath);
        // Parsing the JSON file to get the envrionment data
        json     envData = json::parse(jsonFile);
        // Return parsed data
        return envData;
    }
    catch (json::parse_error &ex)
    {
        VSLAM_LOG_ERROR("- Error parsing the environment JSON file: %s\n",
                        ex.what());
        VSLAM_LOG_ERROR("- Exiting ... \n\n");
        exit(1);
    }
}

std::vector<Room *> DBParser::getEnvRooms(json envData)
{
    envRooms.clear();

    // Check if the JSON file contains rooms
    if (envData["rooms"].size() != 0)
    {
        for (const auto &envDatum : envData["rooms"].items())
        {
            // Initialization
            Room *envRoom = new Room();

            // Fill the room entity
            envRoom->setOpId(-1);
            envRoom->setOpIdG(-1);
            envRoom->setId(stoi(envDatum.key()));
            envRoom->setName(envDatum.value()["name"]);
            envRoom->setMetaMarkerId(envDatum.value()["metaMarker"]);

            // Set the room variant (corridors are incomplete rooms, not a
            // distinct semantic type, so every env room is a plain ROOM)
            envRoom->setRoomVariant(Room::ROOM);

            // Fill the vector
            envRooms.push_back(envRoom);
        }

        // Print the loaded rooms
        VSLAM_LOG_INFO("- Fetched %d rooms from the JSON file! [e.g., '%s'].\n",
                       static_cast<int>(envRooms.size()),
                       envRooms[0]->getName().c_str());
    }
    else
        VSLAM_LOG_INFO("- No rooms found in the JSON file!\n");

    return envRooms;
}
} // namespace ORB_SLAM3
