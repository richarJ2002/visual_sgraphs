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

RoomTrackingState RoomTracker::step(double                      now_s,
                                    const TraversalGuardValues &crossing,
                                    const VerificationVerdict  &verification,
                                    const TrackingStatusInput  &tracking)
{
    const double effectiveNow =
        !std::isfinite(now_s)
            ? lastReceivedTime_s_
            : (now_s < lastReceivedTime_s_ ? lastReceivedTime_s_ : now_s);

    const bool newlyTrackingLost = tracking.lost && !wasTrackingLost_;

    if (newlyTrackingLost)
    {
        /* Transition-table rows 3 and 5 (unconditional); events with no row
         * for the source state (e.g. unrecognised loss from UNKNOWN) are
         * rejected by the oracle. */
        applyEvent(RoomTrackingEvent::TRACKING_LOST,
                   effectiveNow,
                   crossing,
                   verification);
    }
    else
    {
        switch (state_)
        {
        case RoomTrackingState::LOST_WITH_LAST_ROOM:
        {
            if (tracking.newMapCreated && verification.isPass())
            {
                /* Transition-table row 7 (guarded). */
                applyEvent(RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                           effectiveNow,
                           crossing,
                           verification);
            }
            else if (effectiveNow - lastEnterStateTime_s_ >=
                     config_.lost_timeout_s)
            {
                /* Transition-table row 8 (unconditional). */
                applyEvent(RoomTrackingEvent::LOST_TIMEOUT,
                           effectiveNow,
                           crossing,
                           verification);
            }
            break;
        }
        case RoomTrackingState::REACQUIRING_IN_NEW_MAP:
        {
            if (verification.isPass())
            {
                /* Transition-table row 9 (guarded). */
                applyEvent(RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
                           effectiveNow,
                           crossing,
                           verification);
            }
            else if (effectiveNow - lastEnterStateTime_s_ >=
                     config_.reacquire_timeout_s)
            {
                /* Transition-table row 10 (unconditional). */
                applyEvent(RoomTrackingEvent::REACQUIRE_TIMEOUT,
                           effectiveNow,
                           crossing,
                           verification);
            }
            else if (reacquireLastRetryTime_s_ < 0.0 ||
                     effectiveNow - reacquireLastRetryTime_s_ >=
                         config_.reacquire_retry_interval_s)
            {
                /* Retry backoff: after the configured number of
                 * failed attempts the reacquire is retired. */
                ++reacquireRetryCount_;
                reacquireLastRetryTime_s_ = effectiveNow;
                if (reacquireRetryCount_ > config_.reacquire_max_retries)
                {
                    applyEvent(RoomTrackingEvent::REACQUIRE_TIMEOUT,
                               effectiveNow,
                               crossing,
                               verification);
                }
            }
            break;
        }
        case RoomTrackingState::UNKNOWN:
        {
            /* Transition-table row 1 (guarded). */
            if (verification.isPass())
            {
                applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           effectiveNow,
                           crossing,
                           verification);
            }
            break;
        }
        case RoomTrackingState::CONFIRMED_ROOM:
        {
            /* Transition-table row 2 (guarded). The dwell timer
             * starts on first guard satisfaction and resets on any failure. */
            const bool guardSatisfied =
                crossing.passageDetected && crossing.passable &&
                std::isfinite(crossing.confidence) &&
                crossing.confidence >= 0.0 && crossing.confidence <= 1.0 &&
                crossing.confidence >= config_.crossing_confidence;
            TraversalGuardValues guard = crossing;
            guard.dwell_s =
                accumulateDwell(state_, effectiveNow, guardSatisfied);
            if (guard.dwell_s >= config_.crossing_dwell_s)
            {
                applyEvent(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                           effectiveNow,
                           guard,
                           verification);
            }
            break;
        }
        case RoomTrackingState::CROSSING_PASSAGE:
        {
            /* Transition-table row 4 (guarded). Both-sides evidence is
             * cumulative; the dwell timer runs once the traversal facts and
             * the verification verdict both hold. */
            hasObservedBothSides_ =
                hasObservedBothSides_ || crossing.bothSidesObserved;
            const bool guardSatisfied =
                hasObservedBothSides_ && verification.isPass();
            TraversalGuardValues guard = crossing;
            guard.dwell_s =
                accumulateDwell(state_, effectiveNow, guardSatisfied);
            guard.bothSidesObserved = hasObservedBothSides_;
            if (guard.dwell_s >= config_.crossing_dwell_s)
            {
                applyEvent(RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                           effectiveNow,
                           guard,
                           verification);
            }
            break;
        }
        case RoomTrackingState::LOST_WITHOUT_ROOM:
        {
            /* Transition-table row 6 (guarded). */
            if (verification.isPass())
            {
                applyEvent(RoomTrackingEvent::ROOM_REACQUIRED,
                           effectiveNow,
                           crossing,
                           verification);
            }
            break;
        }
        }
    }

    wasTrackingLost_    = tracking.lost;
    lastReceivedTime_s_ = effectiveNow;
    return state_;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
