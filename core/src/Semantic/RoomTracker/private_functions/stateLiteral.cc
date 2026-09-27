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

#include "Semantic/RoomTracker.h"

#include <cmath>
#include <iostream>
#include <sstream>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

const char *stateLiteral(RoomTrackingState state)
{
    switch (state)
    {
    case RoomTrackingState::UNKNOWN:
        return "UNKNOWN";
    case RoomTrackingState::CONFIRMED_ROOM:
        return "CONFIRMED_ROOM";
    case RoomTrackingState::CROSSING_PASSAGE:
        return "CROSSING_PASSAGE";
    case RoomTrackingState::LOST_WITHOUT_ROOM:
        return "LOST_WITHOUT_ROOM";
    case RoomTrackingState::LOST_WITH_LAST_ROOM:
        return "LOST_WITH_LAST_ROOM";
    case RoomTrackingState::REACQUIRING_IN_NEW_MAP:
        return "REACQUIRING_IN_NEW_MAP";
    }
    return "UNKNOWN";
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
