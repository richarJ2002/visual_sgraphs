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

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomTrackerStatus RoomTracker::reset(double now_s_in)
{
    trackingState = RoomTrackingState::UNKNOWN;
    eventHistory.clear();
    lastEvent                = TransitionEvent();
    lastReceivedTime_s       = 0.0;
    lastEnterStateTime_s     = 0.0;
    crossingDwellStartTime_s = -1.0;
    reacquireRetryCount      = 0U;
    reacquireLastRetryTime_s = -1.0;
    hasObservedBothSides     = false;
    wasTrackingLost          = false;
    if (now_s_in > 0.0)
    {
        lastReceivedTime_s = now_s_in;
    }

    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
