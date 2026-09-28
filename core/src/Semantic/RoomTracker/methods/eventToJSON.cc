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

RoomTrackerStatus RoomTracker::eventToJSON(const TransitionEvent &event_in,
                                           std::string           &json_out)
{
    std::ostringstream textStream;
    textStream << "{";
    std::string formattedValue{};
    if (formatDouble(event_in.timestamp_s, formattedValue) !=
        RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
    {
        // formatDouble cannot fail; continue as before.
    }
    textStream << "\"timestamp\":" << formattedValue << ",";
    const char *p_stateLiteral = nullptr;
    if (stateLiteral(event_in.sourceState, p_stateLiteral) !=
        RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
    {
        // stateLiteral cannot fail; continue as before.
    }
    textStream << "\"source\":\"" << p_stateLiteral << "\",";
    const char *p_eventLiteral = nullptr;
    if (eventLiteral(event_in.event, p_eventLiteral) !=
        RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
    {
        // eventLiteral cannot fail; continue as before.
    }
    textStream << "\"event\":\"" << p_eventLiteral << "\",";
    const char *p_stateLiteral2 = nullptr;
    if (stateLiteral(event_in.targetState, p_stateLiteral2) !=
        RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
    {
        // stateLiteral cannot fail; continue as before.
    }
    textStream << "\"target\":\"" << p_stateLiteral2 << "\",";
    textStream << "\"accepted\":" << (event_in.isAccepted ? "true" : "false")
               << ",";
    std::string formattedValue2{};
    if (formatDouble(event_in.dwell_s, formattedValue2) !=
        RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
    {
        // formatDouble cannot fail; continue as before.
    }
    textStream << "\"dwell_s\":" << formattedValue2 << ",";
    std::string formattedValue3{};
    if (formatDouble(event_in.confidence, formattedValue3) !=
        RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
    {
        // formatDouble cannot fail; continue as before.
    }
    textStream << "\"confidence\":" << formattedValue3 << ",";
    textStream << "\"verification_pass\":"
               << (event_in.hasVerificationPassed ? "true" : "false");
    textStream << "}";
    json_out = textStream.str();
    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
