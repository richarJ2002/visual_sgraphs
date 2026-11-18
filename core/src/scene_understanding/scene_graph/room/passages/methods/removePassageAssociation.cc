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
 * @file            removePassageAssociation.cc
 *
 * @brief           Implements Room::removePassageAssociation(), declared in
 *                  Semantic/Room.h.
 */

#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/objects/RoomClass.h"
#include <algorithm>
#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::removePassageAssociation(
    vs_graphs::core::semantic::Passage *p_removedPassage_in,
    bool                               &wasPassageRemoved_out)
{
    if (p_removedPassage_in == nullptr)
    {
        return RoomStatus::ROOM_STATUS_INVALID_ARGUMENT;
    }

    const std::vector<Passage *>::iterator passageIt =
        std::find(passages.begin(), passages.end(), p_removedPassage_in);

    if (passageIt == passages.end())
    {
        wasPassageRemoved_out = false;
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    passages.erase(passageIt);
    wasPassageRemoved_out = true;
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
