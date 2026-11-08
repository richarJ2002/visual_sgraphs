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
 * @file            hasLiveRoomLink.cc
 *
 * @brief           Implements hasLiveRoomLink(), declared in
 *                  scene_graph/SceneGraph/private_functions/private_functions.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/private_functions/private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

bool hasLiveRoomLink(const SceneMap    &sceneMap_in,
                     semantic::Passage *p_passage_in)
{
    semantic::Passage::KnownSideProvenance knownSideProvenance{};
    semantic::Room                        *p_prospectiveRoom = nullptr;
    if (p_passage_in->getKnownSideProvenance(knownSideProvenance) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_in->getProspectiveRoom(p_prospectiveRoom) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKnownSideProvenance/getProspectiveRoom returned "
                     "a failure status although they cannot fail; continuing "
                     "as before.",
                     __func__);
    }
    if (isRoomLive(knownSideProvenance.p_room) || isRoomLive(p_prospectiveRoom))
    {
        return true;
    }

    /* A live room that lists the passage still leads somewhere. */
    for (const std::map<int, std::unique_ptr<semantic::Room>> *p_roomsById :
         {&sceneMap_in.detectedRooms, &sceneMap_in.candidateRooms})
    {
        for (const std::pair<const int, std::unique_ptr<semantic::Room>>
                 &roomEntry : *p_roomsById)
        {
            std::vector<semantic::Passage *> roomPassages{};
            if (isRoomLive(roomEntry.second.get()) &&
                roomEntry.second->getPassages(roomPassages) ==
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS &&
                std::find(roomPassages.begin(),
                          roomPassages.end(),
                          p_passage_in) != roomPassages.end())
            {
                return true;
            }
        }
    }
    return false;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
