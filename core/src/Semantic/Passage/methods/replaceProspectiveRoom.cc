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
 * @file            replaceProspectiveRoom.cc
 *
 * @brief           Implements Passage::replaceProspectiveRoom(), declared in
 *                  Semantic/Passage.h.
 */

#include "Semantic/Passage.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

PassageStatus Passage::replaceProspectiveRoom(
    vs_graphs::core::semantic::Room *p_retiredRoom_in,
    vs_graphs::core::semantic::Room *p_retainedRoom_in,
    bool                            &wasRoomReplaced_out)
{
    if (p_retiredRoom_in == nullptr || p_retainedRoom_in == nullptr ||
        p_retiredRoom_in == p_retainedRoom_in)
    {
        return PassageStatus::PASSAGE_STATUS_INVALID_ARGUMENT;
    }

    std::lock_guard<std::mutex> lock(geometryMutex);

    bool replaced = false;
    if (p_prospectiveRoom == p_retiredRoom_in)
    {
        p_prospectiveRoom = p_retainedRoom_in;
        replaced          = true;
    }

    if (knownSideProvenance.p_room == p_retiredRoom_in)
    {
        knownSideProvenance.p_room = p_retainedRoom_in;
        replaced                   = true;
    }

    wasRoomReplaced_out = replaced;
    return PassageStatus::PASSAGE_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
