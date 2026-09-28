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

RoomTrackerStatus RoomTracker::step(double                      now_s_in,
                                    const TraversalGuardValues &crossing_in,
                                    const VerificationVerdict  &verification_in,
                                    const TrackingStatusInput  &tracking_in,
                                    RoomTrackingState          &nextState_out)
{
    const double effectiveNow =
        !std::isfinite(now_s_in)
            ? lastReceivedTime_s
            : (now_s_in < lastReceivedTime_s ? lastReceivedTime_s : now_s_in);

    const bool newlyTrackingLost = tracking_in.isLost && !wasTrackingLost;

    if (newlyTrackingLost)
    {
        /* Transition-table rows 3 and 5 (unconditional); events with no row
         * for the source state (e.g. unrecognised loss from UNKNOWN) are
         * rejected by the oracle. */
        RoomTrackingState nextState{};
        if (applyEvent(RoomTrackingEvent::TRACKING_LOST,
                       effectiveNow,
                       crossing_in,
                       verification_in,
                       nextState) !=
            RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
        {
            // applyEvent cannot fail; continue as before.
        }
    }
    else
    {
        switch (trackingState)
        {
        case RoomTrackingState::LOST_WITH_LAST_ROOM:
        {
            if (tracking_in.isNewMapCreated && verification_in.isPass())
            {
                /* Transition-table row 7 (guarded). */
                RoomTrackingState nextState2{};
                if (applyEvent(RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                               effectiveNow,
                               crossing_in,
                               verification_in,
                               nextState2) !=
                    RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
                {
                    // applyEvent cannot fail; continue as before.
                }
            }
            else if (effectiveNow - lastEnterStateTime_s >=
                     config.lost_timeout_s)
            {
                /* Transition-table row 8 (unconditional). */
                RoomTrackingState nextState3{};
                if (applyEvent(RoomTrackingEvent::LOST_TIMEOUT,
                               effectiveNow,
                               crossing_in,
                               verification_in,
                               nextState3) !=
                    RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
                {
                    // applyEvent cannot fail; continue as before.
                }
            }
            break;
        }
        case RoomTrackingState::REACQUIRING_IN_NEW_MAP:
        {
            if (verification_in.isPass())
            {
                /* Transition-table row 9 (guarded). */
                RoomTrackingState nextState4{};
                if (applyEvent(RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
                               effectiveNow,
                               crossing_in,
                               verification_in,
                               nextState4) !=
                    RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
                {
                    // applyEvent cannot fail; continue as before.
                }
            }
            else if (effectiveNow - lastEnterStateTime_s >=
                     config.reacquire_timeout_s)
            {
                /* Transition-table row 10 (unconditional). */
                RoomTrackingState nextState5{};
                if (applyEvent(RoomTrackingEvent::REACQUIRE_TIMEOUT,
                               effectiveNow,
                               crossing_in,
                               verification_in,
                               nextState5) !=
                    RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
                {
                    // applyEvent cannot fail; continue as before.
                }
            }
            else if (reacquireLastRetryTime_s < 0.0 ||
                     effectiveNow - reacquireLastRetryTime_s >=
                         config.reacquire_retry_interval_s)
            {
                /* Retry backoff: after the configured number of
                 * failed attempts the reacquire is retired. */
                ++reacquireRetryCount;
                reacquireLastRetryTime_s = effectiveNow;
                if (reacquireRetryCount > config.reacquire_max_retries)
                {
                    RoomTrackingState nextState6{};
                    if (applyEvent(RoomTrackingEvent::REACQUIRE_TIMEOUT,
                                   effectiveNow,
                                   crossing_in,
                                   verification_in,
                                   nextState6) !=
                        RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
                    {
                        // applyEvent cannot fail; continue as before.
                    }
                }
            }
            break;
        }
        case RoomTrackingState::UNKNOWN:
        {
            /* Transition-table row 1 (guarded). */
            if (verification_in.isPass())
            {
                RoomTrackingState nextState7{};
                if (applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                               effectiveNow,
                               crossing_in,
                               verification_in,
                               nextState7) !=
                    RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
                {
                    // applyEvent cannot fail; continue as before.
                }
            }
            break;
        }
        case RoomTrackingState::CONFIRMED_ROOM:
        {
            /* Transition-table row 2 (guarded). The dwell timer
             * starts on first guard satisfaction and resets on any failure. */
            const bool guardSatisfied =
                crossing_in.isPassageDetected && crossing_in.isPassable &&
                std::isfinite(crossing_in.confidence) &&
                crossing_in.confidence >= 0.0 &&
                crossing_in.confidence <= 1.0 &&
                crossing_in.confidence >= config.crossing_confidence;
            TraversalGuardValues updatedGuardValues = crossing_in;
            double               accumulatedDwell{};
            if (accumulateDwell(trackingState,
                                effectiveNow,
                                guardSatisfied,
                                accumulatedDwell) !=
                RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
            {
                // accumulateDwell cannot fail; continue as before.
            }
            updatedGuardValues.dwell_s = accumulatedDwell;
            if (updatedGuardValues.dwell_s >= config.crossing_dwell_s)
            {
                RoomTrackingState nextState8{};
                if (applyEvent(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                               effectiveNow,
                               updatedGuardValues,
                               verification_in,
                               nextState8) !=
                    RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
                {
                    // applyEvent cannot fail; continue as before.
                }
            }
            break;
        }
        case RoomTrackingState::CROSSING_PASSAGE:
        {
            /* Transition-table row 4 (guarded). Both-sides evidence is
             * cumulative; the dwell timer runs once the traversal facts and
             * the verification verdict both hold. */
            hasObservedBothSides =
                hasObservedBothSides || crossing_in.areBothSidesObserved;
            const bool guardSatisfied =
                hasObservedBothSides && verification_in.isPass();
            TraversalGuardValues updatedGuardValues = crossing_in;
            double               accumulatedDwell2{};
            if (accumulateDwell(trackingState,
                                effectiveNow,
                                guardSatisfied,
                                accumulatedDwell2) !=
                RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
            {
                // accumulateDwell cannot fail; continue as before.
            }
            updatedGuardValues.dwell_s              = accumulatedDwell2;
            updatedGuardValues.areBothSidesObserved = hasObservedBothSides;
            if (updatedGuardValues.dwell_s >= config.crossing_dwell_s)
            {
                RoomTrackingState nextState9{};
                if (applyEvent(RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                               effectiveNow,
                               updatedGuardValues,
                               verification_in,
                               nextState9) !=
                    RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
                {
                    // applyEvent cannot fail; continue as before.
                }
            }
            break;
        }
        case RoomTrackingState::LOST_WITHOUT_ROOM:
        {
            /* Transition-table row 6 (guarded). */
            if (verification_in.isPass())
            {
                RoomTrackingState nextState10{};
                if (applyEvent(RoomTrackingEvent::ROOM_REACQUIRED,
                               effectiveNow,
                               crossing_in,
                               verification_in,
                               nextState10) !=
                    RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
                {
                    // applyEvent cannot fail; continue as before.
                }
            }
            break;
        }
        }
    }

    wasTrackingLost    = tracking_in.isLost;
    lastReceivedTime_s = effectiveNow;
    nextState_out      = trackingState;
    return RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
