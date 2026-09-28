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
 * @file            test_SemanticStatus.cpp
 *
 * @brief           Failure statuses of the semantic objects: every rejected
 *                  input returns INVALID_ARGUMENT and leaves the object and
 *                  the outputs unchanged; an absent element is data, not a
 *                  failure.
 */

#include "Geometric/Plane.h"
#include "Semantic/Floor.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace semantic  = vs_graphs::core::semantic;
namespace geometric = vs_graphs::core::geometric;

using semantic::FloorStatus;
using semantic::PassageStatus;
using semantic::RoomStatus;

TEST(SemanticStatus, RoomRejectsNullOrRepeatedWallsAndPassages)
{
    semantic::Room   room;
    geometric::Plane wall;
    bool             result = true;

    EXPECT_EQ(room.removeWall(nullptr, result),
              RoomStatus::ROOM_STATUS_INVALID_ARGUMENT);
    EXPECT_TRUE(result);
    EXPECT_EQ(room.replaceWall(nullptr, &wall, result),
              RoomStatus::ROOM_STATUS_INVALID_ARGUMENT);
    EXPECT_EQ(room.replaceWall(&wall, &wall, result),
              RoomStatus::ROOM_STATUS_INVALID_ARGUMENT);
    EXPECT_EQ(room.replaceGroundPlane(nullptr, &wall, result),
              RoomStatus::ROOM_STATUS_INVALID_ARGUMENT);
    EXPECT_EQ(room.removePassageAssociation(nullptr, result),
              RoomStatus::ROOM_STATUS_INVALID_ARGUMENT);
    EXPECT_EQ(room.replacePassageAssociation(nullptr, nullptr, result),
              RoomStatus::ROOM_STATUS_INVALID_ARGUMENT);
    EXPECT_TRUE(result);

    std::vector<geometric::Plane *> walls;
    ASSERT_EQ(room.getWalls(walls), RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(walls.empty());
}

TEST(SemanticStatus, RoomReportsAnAbsentWallAsDataNotFailure)
{
    semantic::Room   room;
    geometric::Plane wall;
    bool             wasWallRemoved = true;

    EXPECT_EQ(room.removeWall(&wall, wasWallRemoved),
              RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(wasWallRemoved);
}

TEST(SemanticStatus, PassageRejectsNullRepeatedAndDegenerateInputs)
{
    semantic::Passage passage;
    geometric::Plane  plane;
    semantic::Room    room;
    bool              result = true;

    EXPECT_EQ(passage.replaceProspectiveRoom(nullptr, &room, result),
              PassageStatus::PASSAGE_STATUS_INVALID_ARGUMENT);
    EXPECT_EQ(passage.replaceProspectiveRoom(&room, &room, result),
              PassageStatus::PASSAGE_STATUS_INVALID_ARGUMENT);
    EXPECT_EQ(passage.replacePlaneAssociation(nullptr, &plane, result),
              PassageStatus::PASSAGE_STATUS_INVALID_ARGUMENT);
    EXPECT_EQ(passage.mergeFromDuplicate(nullptr, result),
              PassageStatus::PASSAGE_STATUS_INVALID_ARGUMENT);
    EXPECT_EQ(passage.mergeFromDuplicate(&passage, result),
              PassageStatus::PASSAGE_STATUS_INVALID_ARGUMENT);
    EXPECT_TRUE(result);

    EXPECT_EQ(passage.setKnownSideDirection(Eigen::Vector3d::Zero()),
              PassageStatus::PASSAGE_STATUS_INVALID_ARGUMENT);
    EXPECT_EQ(passage.setKnownSideDirection(
                  Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                                  0.0,
                                  0.0)),
              PassageStatus::PASSAGE_STATUS_INVALID_ARGUMENT);
}

TEST(SemanticStatus, FloorRejectsNullRoomsAndDegeneratePlanes)
{
    semantic::Floor floor;
    semantic::Room  room;
    bool            wasRoomReplaced = true;

    EXPECT_EQ(floor.replaceRoom(nullptr, &room, wasRoomReplaced),
              FloorStatus::FLOOR_STATUS_INVALID_ARGUMENT);
    EXPECT_EQ(floor.replaceRoom(&room, &room, wasRoomReplaced),
              FloorStatus::FLOOR_STATUS_INVALID_ARGUMENT);
    EXPECT_TRUE(wasRoomReplaced);

    EXPECT_EQ(floor.setPlaneIdentity(Eigen::Vector4d::Zero(), 10U, 3U),
              FloorStatus::FLOOR_STATUS_INVALID_ARGUMENT);
    bool hasPlaneIdentity = true;
    ASSERT_EQ(floor.hasPlaneIdentity(hasPlaneIdentity),
              FloorStatus::FLOOR_STATUS_SUCCESS);
    EXPECT_FALSE(hasPlaneIdentity);

    EXPECT_EQ(
        floor.setPlaneIdentity(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0), 10U, 3U),
        FloorStatus::FLOOR_STATUS_SUCCESS);
}
