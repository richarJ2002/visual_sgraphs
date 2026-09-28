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

RoomTrackerStatus
    RoomTracker::applyRow(RoomTrackingState           source_in,
                          RoomTrackingEvent           event_in,
                          double                      now_s_in,
                          const TraversalGuardValues &crossing_in,
                          const VerificationVerdict  &verification_in,
                          bool                       &isAccepted_out)
{
    /* Transition table, guarded rows first. */
    if (source_in == RoomTrackingState::UNKNOWN &&
        event_in == RoomTrackingEvent::FIRST_ROOM_CONFIRMED)
    {
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   verification_in.isPass()) !=
            RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }
    else if (source_in == RoomTrackingState::CONFIRMED_ROOM &&
             event_in == RoomTrackingEvent::PASSAGE_CROSSING_DETECTED)
    {
        const bool guardSatisfied =
            crossing_in.isPassageDetected && crossing_in.isPassable &&
            std::isfinite(crossing_in.dwell_s) && crossing_in.dwell_s >= 0.0 &&
            std::isfinite(crossing_in.confidence) &&
            crossing_in.confidence >= 0.0 && crossing_in.confidence <= 1.0 &&
            crossing_in.dwell_s >= config.crossing_dwell_s &&
            crossing_in.confidence >= config.crossing_confidence;
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   guardSatisfied) !=
            RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }
    else if (source_in == RoomTrackingState::CROSSING_PASSAGE &&
             event_in == RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE)
    {
        const bool guardSatisfied =
            crossing_in.areBothSidesObserved &&
            std::isfinite(crossing_in.dwell_s) && crossing_in.dwell_s >= 0.0 &&
            crossing_in.dwell_s >= config.crossing_dwell_s &&
            verification_in.isPass();
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   guardSatisfied) !=
            RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }
    else if (source_in == RoomTrackingState::LOST_WITHOUT_ROOM &&
             event_in == RoomTrackingEvent::ROOM_REACQUIRED)
    {
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   verification_in.isPass()) !=
            RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }
    else if (source_in == RoomTrackingState::LOST_WITH_LAST_ROOM &&
             event_in == RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH)
    {
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   verification_in.isPass()) !=
            RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }
    else if (source_in == RoomTrackingState::REACQUIRING_IN_NEW_MAP &&
             event_in == RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM)
    {
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   verification_in.isPass()) !=
            RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }
    /* Unconditional rows: no guard checks, transition always fires. */
    else if (source_in == RoomTrackingState::CONFIRMED_ROOM &&
             event_in == RoomTrackingEvent::TRACKING_LOST)
    {
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   true) != RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }
    else if (source_in == RoomTrackingState::CROSSING_PASSAGE &&
             event_in == RoomTrackingEvent::TRACKING_LOST)
    {
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   true) != RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }
    else if (source_in == RoomTrackingState::LOST_WITH_LAST_ROOM &&
             event_in == RoomTrackingEvent::LOST_TIMEOUT)
    {
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   true) != RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }
    else if (source_in == RoomTrackingState::REACQUIRING_IN_NEW_MAP &&
             event_in == RoomTrackingEvent::REACQUIRE_TIMEOUT)
    {
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   true) != RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }
    else
    {
        /* Events with no defined row for the source state are rejected. */
        if (commit(source_in,
                   event_in,
                   now_s_in,
                   crossing_in,
                   verification_in,
                   false) != RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // commit cannot fail; continue as before.
        }
    }

    isAccepted_out = eventHistory.back().isAccepted;
    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
