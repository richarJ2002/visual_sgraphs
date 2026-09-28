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
    RoomTracker::commit(RoomTrackingState           source_in,
                        RoomTrackingEvent           event_in,
                        double                      now_s_in,
                        const TraversalGuardValues &crossing_in,
                        const VerificationVerdict  &verification_in,
                        bool                        accepted_in)
{
    TransitionEvent transitionRecord;
    transitionRecord.timestamp_s           = now_s_in;
    transitionRecord.sourceState           = source_in;
    transitionRecord.event                 = event_in;
    transitionRecord.dwell_s               = crossing_in.dwell_s;
    transitionRecord.confidence            = crossing_in.confidence;
    transitionRecord.hasVerificationPassed = verification_in.isPass();
    transitionRecord.targetState = accepted_in ? trackingState : source_in;

    /* Resolve the target state for accepted transitions. */
    if (accepted_in)
    {
        if (event_in == RoomTrackingEvent::FIRST_ROOM_CONFIRMED ||
            event_in == RoomTrackingEvent::ROOM_REACQUIRED ||
            event_in == RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM ||
            event_in == RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE)
        {
            transitionRecord.targetState = RoomTrackingState::CONFIRMED_ROOM;
        }
        else if (event_in == RoomTrackingEvent::PASSAGE_CROSSING_DETECTED)
        {
            transitionRecord.targetState = RoomTrackingState::CROSSING_PASSAGE;
        }
        else if (event_in == RoomTrackingEvent::TRACKING_LOST)
        {
            transitionRecord.targetState =
                RoomTrackingState::LOST_WITH_LAST_ROOM;
        }
        else if (event_in == RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH)
        {
            transitionRecord.targetState =
                RoomTrackingState::REACQUIRING_IN_NEW_MAP;
        }
        else if (event_in == RoomTrackingEvent::LOST_TIMEOUT ||
                 event_in == RoomTrackingEvent::REACQUIRE_TIMEOUT)
        {
            transitionRecord.targetState = RoomTrackingState::LOST_WITHOUT_ROOM;
        }
        else
        {
            transitionRecord.targetState = source_in;
        }
    }
    transitionRecord.isAccepted = accepted_in;

    eventHistory.push_back(transitionRecord);
    lastEvent = transitionRecord;

    if (accepted_in)
    {
        std::string json{};
        if (eventToJSON(transitionRecord, json) !=
            RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // eventToJSON cannot fail; continue as before.
        }
        std::cout << "[RoomTracker] transition: " << json << std::endl;

        trackingState            = transitionRecord.targetState;
        lastEnterStateTime_s     = now_s_in;
        crossingDwellStartTime_s = -1.0;
        hasObservedBothSides     = false;
        if (transitionRecord.targetState ==
            RoomTrackingState::REACQUIRING_IN_NEW_MAP)
        {
            reacquireRetryCount      = 0U;
            reacquireLastRetryTime_s = -1.0;
        }
    }
    else
    {
        std::string json2{};
        if (eventToJSON(transitionRecord, json2) !=
            RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // eventToJSON cannot fail; continue as before.
        }
        std::cout << "[RoomTracker] WARN rejected transition: " << json2
                  << std::endl;
    }

    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
