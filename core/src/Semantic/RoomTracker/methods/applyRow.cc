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

bool RoomTracker::applyRow(RoomTrackingState           source,
                           RoomTrackingEvent           event,
                           double                      now_s,
                           const TraversalGuardValues &crossing,
                           const VerificationVerdict  &verification)
{
    /* Transition table, guarded rows first. */
    if (source == RoomTrackingState::UNKNOWN &&
        event == RoomTrackingEvent::FIRST_ROOM_CONFIRMED)
    {
        commit(source,
               event,
               now_s,
               crossing,
               verification,
               verification.isPass());
    }
    else if (source == RoomTrackingState::CONFIRMED_ROOM &&
             event == RoomTrackingEvent::PASSAGE_CROSSING_DETECTED)
    {
        const bool guard =
            crossing.passageDetected && crossing.passable &&
            std::isfinite(crossing.dwell_s) && crossing.dwell_s >= 0.0 &&
            std::isfinite(crossing.confidence) && crossing.confidence >= 0.0 &&
            crossing.confidence <= 1.0 &&
            crossing.dwell_s >= config_.crossing_dwell_s &&
            crossing.confidence >= config_.crossing_confidence;
        commit(source, event, now_s, crossing, verification, guard);
    }
    else if (source == RoomTrackingState::CROSSING_PASSAGE &&
             event == RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE)
    {
        const bool guard = crossing.bothSidesObserved &&
                           std::isfinite(crossing.dwell_s) &&
                           crossing.dwell_s >= 0.0 &&
                           crossing.dwell_s >= config_.crossing_dwell_s &&
                           verification.isPass();
        commit(source, event, now_s, crossing, verification, guard);
    }
    else if (source == RoomTrackingState::LOST_WITHOUT_ROOM &&
             event == RoomTrackingEvent::ROOM_REACQUIRED)
    {
        commit(source,
               event,
               now_s,
               crossing,
               verification,
               verification.isPass());
    }
    else if (source == RoomTrackingState::LOST_WITH_LAST_ROOM &&
             event == RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH)
    {
        commit(source,
               event,
               now_s,
               crossing,
               verification,
               verification.isPass());
    }
    else if (source == RoomTrackingState::REACQUIRING_IN_NEW_MAP &&
             event == RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM)
    {
        commit(source,
               event,
               now_s,
               crossing,
               verification,
               verification.isPass());
    }
    /* Unconditional rows: no guard checks, transition always fires. */
    else if (source == RoomTrackingState::CONFIRMED_ROOM &&
             event == RoomTrackingEvent::TRACKING_LOST)
    {
        commit(source, event, now_s, crossing, verification, true);
    }
    else if (source == RoomTrackingState::CROSSING_PASSAGE &&
             event == RoomTrackingEvent::TRACKING_LOST)
    {
        commit(source, event, now_s, crossing, verification, true);
    }
    else if (source == RoomTrackingState::LOST_WITH_LAST_ROOM &&
             event == RoomTrackingEvent::LOST_TIMEOUT)
    {
        commit(source, event, now_s, crossing, verification, true);
    }
    else if (source == RoomTrackingState::REACQUIRING_IN_NEW_MAP &&
             event == RoomTrackingEvent::REACQUIRE_TIMEOUT)
    {
        commit(source, event, now_s, crossing, verification, true);
    }
    else
    {
        /* Events with no defined row for the source state are rejected. */
        commit(source, event, now_s, crossing, verification, false);
    }

    return eventHistory_.back().accepted;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
