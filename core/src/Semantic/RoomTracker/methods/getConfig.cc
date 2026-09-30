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

/*!
 * @file            getConfig.cc
 *
 * @brief           Implements RoomTracker::getConfig(), declared in
 *                  Semantic/RoomTracker.h.
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

RoomTrackerStatus
    RoomTracker::getConfig(const RoomTrackerConfig *&p_config_out) const
{
    p_config_out = &(config);
    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
