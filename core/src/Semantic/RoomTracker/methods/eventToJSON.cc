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

std::string RoomTracker::eventToJSON(const TransitionEvent &event)
{
    std::ostringstream stream;
    stream << "{";
    stream << "\"timestamp\":" << formatDouble(event.timestamp_s) << ",";
    stream << "\"source\":\"" << stateLiteral(event.sourceState) << "\",";
    stream << "\"event\":\"" << eventLiteral(event.event) << "\",";
    stream << "\"target\":\"" << stateLiteral(event.targetState) << "\",";
    stream << "\"accepted\":" << (event.accepted ? "true" : "false") << ",";
    stream << "\"dwell_s\":" << formatDouble(event.dwell_s) << ",";
    stream << "\"confidence\":" << formatDouble(event.confidence) << ",";
    stream << "\"verification_pass\":"
           << (event.verificationPass ? "true" : "false");
    stream << "}";
    return stream.str();
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
