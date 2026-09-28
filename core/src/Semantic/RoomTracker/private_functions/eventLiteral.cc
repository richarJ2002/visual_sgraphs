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

RoomTrackerStatus eventLiteral(RoomTrackingEvent event_in,
                               const char      *&p_eventLiteral_out)
{
    switch (event_in)
    {
    case RoomTrackingEvent::FIRST_ROOM_CONFIRMED:
    {
        p_eventLiteral_out = "FIRST_ROOM_CONFIRMED";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingEvent::PASSAGE_CROSSING_DETECTED:
    {
        p_eventLiteral_out = "PASSAGE_CROSSING_DETECTED";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingEvent::TRACKING_LOST:
    {
        p_eventLiteral_out = "TRACKING_LOST";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE:
    {
        p_eventLiteral_out = "PASSAGE_TRAVERSAL_COMPLETE";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingEvent::ROOM_REACQUIRED:
    {
        p_eventLiteral_out = "ROOM_REACQUIRED";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH:
    {
        p_eventLiteral_out = "NEW_MAP_WITH_ROOM_MATCH";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM:
    {
        p_eventLiteral_out = "VERIFIED_MATCH_TO_LAST_ROOM";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingEvent::LOST_TIMEOUT:
    {
        p_eventLiteral_out = "LOST_TIMEOUT";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingEvent::REACQUIRE_TIMEOUT:
    {
        p_eventLiteral_out = "REACQUIRE_TIMEOUT";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    }
    p_eventLiteral_out = "UNKNOWN_EVENT";
    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
