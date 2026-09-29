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
#include <rclcpp/logging.hpp>
#include <sstream>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomTrackerStatus
    RoomTracker::applyEvent(RoomTrackingEvent           event_in,
                            double                      now_s_in,
                            const TraversalGuardValues &crossing_in,
                            const VerificationVerdict  &verification_in,
                            RoomTrackingState          &nextState_out)
{
    bool isAccepted{};
    if (applyRow(trackingState,
                 event_in,
                 now_s_in,
                 crossing_in,
                 verification_in,
                 isAccepted) != RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: applyRow returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    nextState_out = trackingState;
    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
