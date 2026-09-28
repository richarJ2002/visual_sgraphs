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
    textStream << "\"timestamp\":" << formatDouble(event_in.timestamp_s) << ",";
    textStream << "\"source\":\"" << stateLiteral(event_in.sourceState)
               << "\",";
    textStream << "\"event\":\"" << eventLiteral(event_in.event) << "\",";
    textStream << "\"target\":\"" << stateLiteral(event_in.targetState)
               << "\",";
    textStream << "\"accepted\":" << (event_in.isAccepted ? "true" : "false")
               << ",";
    textStream << "\"dwell_s\":" << formatDouble(event_in.dwell_s) << ",";
    textStream << "\"confidence\":" << formatDouble(event_in.confidence) << ",";
    textStream << "\"verification_pass\":"
               << (event_in.hasVerificationPassed ? "true" : "false");
    textStream << "}";
    json_out = textStream.str();
    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
