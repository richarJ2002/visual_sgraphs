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

void RoomTracker::commit(RoomTrackingState           source,
                         RoomTrackingEvent           event,
                         double                      now_s,
                         const TraversalGuardValues &crossing,
                         const VerificationVerdict  &verification,
                         bool                        accepted)
{
    TransitionEvent record;
    record.timestamp_s      = now_s;
    record.sourceState      = source;
    record.event            = event;
    record.dwell_s          = crossing.dwell_s;
    record.confidence       = crossing.confidence;
    record.verificationPass = verification.isPass();
    record.targetState      = accepted ? state_ : source;

    /* Resolve the target state for accepted transitions. */
    if (accepted)
    {
        if (event == RoomTrackingEvent::FIRST_ROOM_CONFIRMED ||
            event == RoomTrackingEvent::ROOM_REACQUIRED ||
            event == RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM ||
            event == RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE)
        {
            record.targetState = RoomTrackingState::CONFIRMED_ROOM;
        }
        else if (event == RoomTrackingEvent::PASSAGE_CROSSING_DETECTED)
        {
            record.targetState = RoomTrackingState::CROSSING_PASSAGE;
        }
        else if (event == RoomTrackingEvent::TRACKING_LOST)
        {
            record.targetState = RoomTrackingState::LOST_WITH_LAST_ROOM;
        }
        else if (event == RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH)
        {
            record.targetState = RoomTrackingState::REACQUIRING_IN_NEW_MAP;
        }
        else if (event == RoomTrackingEvent::LOST_TIMEOUT ||
                 event == RoomTrackingEvent::REACQUIRE_TIMEOUT)
        {
            record.targetState = RoomTrackingState::LOST_WITHOUT_ROOM;
        }
        else
        {
            record.targetState = source;
        }
    }
    record.accepted = accepted;

    eventHistory_.push_back(record);
    lastEvent_ = record;

    if (accepted)
    {
        std::cout << "[RoomTracker] transition: " << eventToJSON(record)
                  << std::endl;

        state_                    = record.targetState;
        lastEnterStateTime_s_     = now_s;
        crossingDwellStartTime_s_ = -1.0;
        hasObservedBothSides_     = false;
        if (record.targetState == RoomTrackingState::REACQUIRING_IN_NEW_MAP)
        {
            reacquireRetryCount_      = 0U;
            reacquireLastRetryTime_s_ = -1.0;
        }
    }
    else
    {
        std::cout << "[RoomTracker] WARN rejected transition: "
                  << eventToJSON(record) << std::endl;
    }
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
