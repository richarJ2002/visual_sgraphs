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

/*!
 * @file            selectBootstrapRoom.cc
 *
 * @brief           Implements selectBootstrapRoom(), declared in
 *                  bootstrap/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/bootstrap/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

semantic::Room *
    selectBootstrapRoom(const std::vector<semantic::Room *> &rooms_in,
                        int                                  currentRoomId_in,
                        int                                  recoveryRoomId_in)
{
    semantic::Room *p_bootstrapRoom =
        findLiveRoomById(rooms_in, currentRoomId_in);
    if (p_bootstrapRoom == nullptr)
    {
        p_bootstrapRoom = findLiveRoomById(rooms_in, recoveryRoomId_in);
    }

    /* With a recovery room, never fall back to an arbitrary live room: a
     * spurious free-space room created during the reset transient would
     * otherwise take the current room away from the last known hierarchy.
     * The lowest-id fallback applies to a cold start only. */
    if (p_bootstrapRoom == nullptr && recoveryRoomId_in < 0)
    {
        p_bootstrapRoom = findFirstLiveRoom(rooms_in);
    }
    return p_bootstrapRoom;
}

} /* namespace core */
} /* namespace vs_graphs */
