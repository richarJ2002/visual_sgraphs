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

RoomTrackerStatus stateLiteral(RoomTrackingState state_in,
                               const char      *&p_stateLiteral_out)
{
    switch (state_in)
    {
    case RoomTrackingState::UNKNOWN:
    {
        p_stateLiteral_out = "UNKNOWN";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingState::CONFIRMED_ROOM:
    {
        p_stateLiteral_out = "CONFIRMED_ROOM";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingState::CROSSING_PASSAGE:
    {
        p_stateLiteral_out = "CROSSING_PASSAGE";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingState::LOST_WITHOUT_ROOM:
    {
        p_stateLiteral_out = "LOST_WITHOUT_ROOM";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingState::LOST_WITH_LAST_ROOM:
    {
        p_stateLiteral_out = "LOST_WITH_LAST_ROOM";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    case RoomTrackingState::REACQUIRING_IN_NEW_MAP:
    {
        p_stateLiteral_out = "REACQUIRING_IN_NEW_MAP";
        return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
    }
    }
    p_stateLiteral_out = "UNKNOWN";
    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
