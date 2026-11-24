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
 * @file            selectMajorityRoomId.cc
 *
 * @brief           Implements selectMajorityRoomId(), declared in
 *                  scene_graph/room/keyframes/public_functions/
 *                  public_functions.h.
 */

#include "scene_understanding/scene_graph/room/keyframes/public_functions/public_functions.h"

#include <map>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus selectMajorityRoomId(
    const std::vector<std::pair<int, long unsigned int>> &roomVotes_in,
    int                                                  &majorityRoomId_out)
{
    majorityRoomId_out = -1;

    /* Votes and latest voter id per room, in room id order. */
    std::map<int, std::pair<std::size_t, long unsigned int>> roomTallies{};
    for (const std::pair<int, long unsigned int> &roomVote : roomVotes_in)
    {
        if (roomVote.first < 0)
        {
            continue;
        }
        std::pair<std::size_t, long unsigned int> &roomTally =
            roomTallies[roomVote.first];
        roomTally.first++;
        if (roomVote.second > roomTally.second)
        {
            roomTally.second = roomVote.second;
        }
    }

    std::pair<std::size_t, long unsigned int> bestTally{0U, 0U};
    for (const std::pair<const int, std::pair<std::size_t, long unsigned int>>
             &roomTally : roomTallies)
    {
        if (roomTally.second > bestTally)
        {
            bestTally          = roomTally.second;
            majorityRoomId_out = roomTally.first;
        }
    }
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
