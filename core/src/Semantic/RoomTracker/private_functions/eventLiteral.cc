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

const char *eventLiteral(RoomTrackingEvent event)
{
    switch (event)
    {
    case RoomTrackingEvent::FIRST_ROOM_CONFIRMED:
        return "FIRST_ROOM_CONFIRMED";
    case RoomTrackingEvent::PASSAGE_CROSSING_DETECTED:
        return "PASSAGE_CROSSING_DETECTED";
    case RoomTrackingEvent::TRACKING_LOST:
        return "TRACKING_LOST";
    case RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE:
        return "PASSAGE_TRAVERSAL_COMPLETE";
    case RoomTrackingEvent::ROOM_REACQUIRED:
        return "ROOM_REACQUIRED";
    case RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH:
        return "NEW_MAP_WITH_ROOM_MATCH";
    case RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM:
        return "VERIFIED_MATCH_TO_LAST_ROOM";
    case RoomTrackingEvent::LOST_TIMEOUT:
        return "LOST_TIMEOUT";
    case RoomTrackingEvent::REACQUIRE_TIMEOUT:
        return "REACQUIRE_TIMEOUT";
    }
    return "UNKNOWN_EVENT";
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
