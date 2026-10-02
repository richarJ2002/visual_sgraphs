/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
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
 * @file            test_DatabaseParser.cpp
 *
 * @brief           DBParser::getEnvironmentRooms builds one room per entry of
 *                  the "rooms" array, with the entry's index as the room id
 *                  and its name and meta marker copied.
 */

#include "DatabaseParser.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{

TEST(DBParser, EnvironmentRoomsTakeTheirArrayIndexAsId)
{
    Json environmentData = Json::parse(
        R"({"rooms": [{"name": "Office-421", "metaMarker": 5},
                      {"name": "Lab", "metaMarker": 7}]})");
    DBParser                      parser;
    std::vector<semantic::Room *> rooms;
    ASSERT_EQ(parser.getEnvironmentRooms(environmentData, rooms),
              DBParserStatus::DBPARSER_STATUS_SUCCESS);
    ASSERT_EQ(rooms.size(), 2U);

    const std::vector<std::string> expectedNames{"Office-421", "Lab"};
    const std::vector<int>         expectedMetaMarkers{5, 7};
    for (std::size_t roomIndex = 0; roomIndex < rooms.size(); ++roomIndex)
    {
        int id{};
        ASSERT_EQ(rooms[roomIndex]->getId(id),
                  semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ(id, static_cast<int>(roomIndex));
        std::string name;
        ASSERT_EQ(rooms[roomIndex]->getName(name),
                  semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ(name, expectedNames[roomIndex]);
        int metaMarkerId{};
        ASSERT_EQ(rooms[roomIndex]->getMetaMarkerId(metaMarkerId),
                  semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ(metaMarkerId, expectedMetaMarkers[roomIndex]);
    }

    // The parser hands out non-owning views and never frees its rooms.
    for (semantic::Room *p_room : rooms)
    {
        delete p_room;
    }
}

TEST(DBParser, NoRoomsGivesAnEmptyList)
{
    Json                          environmentData = Json::parse(R"({})");
    DBParser                      parser;
    std::vector<semantic::Room *> rooms;
    ASSERT_EQ(parser.getEnvironmentRooms(environmentData, rooms),
              DBParserStatus::DBPARSER_STATUS_SUCCESS);
    EXPECT_TRUE(rooms.empty());
}

} // namespace
} // namespace core
} // namespace vs_graphs
