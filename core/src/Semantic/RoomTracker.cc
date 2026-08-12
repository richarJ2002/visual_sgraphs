/**
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

namespace ORB_SLAM3
{
namespace
{
/*! @brief Formats a double for JSON output without trailing exponent noise. */
std::string formatDouble(double value)
{
    std::ostringstream stream;
    stream << value;
    return stream.str();
}

const char *stateLiteral(RoomTrackingState state)
{
    switch (state)
    {
    case RoomTrackingState::UNKNOWN:
        return "UNKNOWN";
    case RoomTrackingState::CONFIRMED_ROOM:
        return "CONFIRMED_ROOM";
    case RoomTrackingState::CROSSING_PASSAGE:
        return "CROSSING_PASSAGE";
    case RoomTrackingState::LOST_WITHOUT_ROOM:
        return "LOST_WITHOUT_ROOM";
    case RoomTrackingState::LOST_WITH_LAST_ROOM:
        return "LOST_WITH_LAST_ROOM";
    case RoomTrackingState::REACQUIRING_IN_NEW_MAP:
        return "REACQUIRING_IN_NEW_MAP";
    }
    return "UNKNOWN";
}

const char *eventLiteral(RoomTrackingEvent event)
{
    switch (event)
    {
    case RoomTrackingEvent::FIRST_ROOM_CONFIRMED:
        return "FIRST_ROOM_CONFIRMED";
    case RoomTrackingEvent::PASSAGE_CROSSING_DETECTED:
        return "PASSAGE_CROSSING_DETECTED";
    case RoomTrackingEvent::TRACKING_LOST:
        return "TRACKING_LOST";
    case RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE:
        return "PASSAGE_TRAVERSAL_COMPLETE";
    case RoomTrackingEvent::ROOM_REACQUIRED:
        return "ROOM_REACQUIRED";
    case RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH:
        return "NEW_MAP_WITH_ROOM_MATCH";
    case RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM:
        return "VERIFIED_MATCH_TO_LAST_ROOM";
    case RoomTrackingEvent::LOST_TIMEOUT:
        return "LOST_TIMEOUT";
    case RoomTrackingEvent::REACQUIRE_TIMEOUT:
        return "REACQUIRE_TIMEOUT";
    }
    return "UNKNOWN_EVENT";
}

} // namespace

RoomTracker::RoomTracker(const RoomTrackerConfig &config_in)
    : config_(config_in)
{
}

void RoomTracker::reset(double now_s)
{
    state_                       = RoomTrackingState::UNKNOWN;
    eventHistory_.clear();
    lastEvent_                   = TransitionEvent();
    lastReceivedTime_s_          = 0.0;
    lastEnterStateTime_s_        = 0.0;
    crossingDwellStartTime_s_    = -1.0;
    reacquireRetryCount_         = 0U;
    reacquireLastRetryTime_s_    = -1.0;
    hasObservedBothSides_        = false;
    wasTrackingLost_             = false;
    if (now_s > 0.0)
    {
        lastReceivedTime_s_ = now_s;
    }
}

RoomTrackingState RoomTracker::getState() const
{
    return state_;
}

const std::vector<TransitionEvent> &RoomTracker::getEventHistory() const
{
    return eventHistory_;
}

const TransitionEvent &RoomTracker::getLastEvent() const
{
    return lastEvent_;
}

const RoomTrackerConfig &RoomTracker::getConfig() const
{
    return config_;
}

std::string RoomTracker::stateToString(RoomTrackingState state)
{
    return stateLiteral(state);
}

std::string RoomTracker::eventToString(RoomTrackingEvent event)
{
    return eventLiteral(event);
}

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

double RoomTracker::computeConfidence(double inlier_ratio,
                                      double normalised_condition_number,
                                      double angular_residual_rad,
                                      double sigma_theta_rad)
{
    if (!std::isfinite(inlier_ratio) ||
        !std::isfinite(normalised_condition_number) ||
        !std::isfinite(angular_residual_rad))
    {
        return 0.0;
    }
    if (inlier_ratio < 0.0)
    {
        inlier_ratio = 0.0;
    }
    else if (inlier_ratio > 1.0)
    {
        inlier_ratio = 1.0;
    }
    if (normalised_condition_number < 0.0)
    {
        normalised_condition_number = 0.0;
    }
    else if (normalised_condition_number > 1.0)
    {
        normalised_condition_number = 1.0;
    }

    double residualDecay = 1.0;
    if (std::isfinite(sigma_theta_rad) && sigma_theta_rad > 1e-12)
    {
        residualDecay = std::exp(-angular_residual_rad / sigma_theta_rad);
    }
    else
    {
        residualDecay = std::abs(angular_residual_rad) < 1e-12 ? 1.0 : 0.0;
    }

    return inlier_ratio * (1.0 - normalised_condition_number) * residualDecay;
}

double RoomTracker::accumulateDwell(RoomTrackingState state,
                                    double            now_s,
                                    bool              guardSatisfied)
{
    (void)state;
    if (!guardSatisfied)
    {
        crossingDwellStartTime_s_ = -1.0;
        return 0.0;
    }
    if (crossingDwellStartTime_s_ < 0.0)
    {
        crossingDwellStartTime_s_ = now_s;
    }
    return now_s - crossingDwellStartTime_s_;
}

RoomTrackingState RoomTracker::step(double                        now_s,
                                    const TraversalGuardValues   &crossing,
                                    const VerificationVerdict    &verification,
                                    const TrackingStatusInput    &tracking)
{
    const double effectiveNow =
        now_s < lastReceivedTime_s_ ? lastReceivedTime_s_ : now_s;

    const bool newlyTrackingLost = tracking.lost && !wasTrackingLost_;

    if (newlyTrackingLost)
    {
        /* Section 18.2 rows 3 and 5 (unconditional); events with no row for
         * the source state (e.g. unrecognised loss from UNKNOWN) are rejected
         * by the oracle. */
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
            if (tracking.newMapCreated && verification.pass)
            {
                /* Section 18.2 row 7 (guarded). */
                applyEvent(RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                           effectiveNow,
                           crossing,
                           verification);
            }
            else if (effectiveNow - lastEnterStateTime_s_ >=
                     config_.lost_timeout_s)
            {
                /* Section 18.2 row 8 (unconditional). */
                applyEvent(RoomTrackingEvent::LOST_TIMEOUT,
                           effectiveNow,
                           crossing,
                           verification);
            }
            break;
        }
        case RoomTrackingState::REACQUIRING_IN_NEW_MAP:
        {
            if (verification.pass)
            {
                /* Section 18.2 row 9 (guarded). */
                applyEvent(RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
                           effectiveNow,
                           crossing,
                           verification);
            }
            else if (effectiveNow - lastEnterStateTime_s_ >=
                     config_.reacquire_timeout_s)
            {
                /* Section 18.2 row 10 (unconditional). */
                applyEvent(RoomTrackingEvent::REACQUIRE_TIMEOUT,
                           effectiveNow,
                           crossing,
                           verification);
            }
            else if (reacquireLastRetryTime_s_ < 0.0 ||
                     effectiveNow - reacquireLastRetryTime_s_ >=
                         config_.reacquire_retry_interval_s)
            {
                /* Section 18.5 retry backoff: after the configured number of
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
            /* Section 18.2 row 1 (guarded). */
            if (verification.pass)
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
            /* Section 18.2 row 2 (guarded). The Section 18.3 dwell timer
             * starts on first guard satisfaction and resets on any failure. */
            const bool guardSatisfied =
                crossing.passageDetected && crossing.passable &&
                crossing.confidence >= config_.crossing_confidence;
            TraversalGuardValues guard = crossing;
            guard.dwell_s = accumulateDwell(state_,
                                            effectiveNow,
                                            guardSatisfied);
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
            /* Section 18.2 row 4 (guarded). Both-sides evidence is
             * cumulative; the dwell timer runs once the traversal facts and
             * the verification verdict both hold. */
            hasObservedBothSides_ =
                hasObservedBothSides_ || crossing.bothSidesObserved;
            const bool guardSatisfied =
                hasObservedBothSides_ && verification.pass;
            TraversalGuardValues guard = crossing;
            guard.dwell_s = accumulateDwell(state_,
                                            effectiveNow,
                                            guardSatisfied);
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
            /* Section 18.2 row 6 (guarded). */
            if (verification.pass)
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

RoomTrackingState
    RoomTracker::applyEvent(RoomTrackingEvent         event,
                            double                    now_s,
                            const TraversalGuardValues &crossing,
                            const VerificationVerdict  &verification)
{
    applyRow(state_, event, now_s, crossing, verification);
    return state_;
}

bool RoomTracker::applyRow(RoomTrackingState          source,
                           RoomTrackingEvent          event,
                           double                     now_s,
                           const TraversalGuardValues &crossing,
                           const VerificationVerdict  &verification)
{
    /* Section 18.2 transition table, guarded rows first. */
    if (source == RoomTrackingState::UNKNOWN &&
        event == RoomTrackingEvent::FIRST_ROOM_CONFIRMED)
    {
        commit(source,
               event,
               now_s,
               crossing,
               verification,
               verification.pass);
    }
    else if (source == RoomTrackingState::CONFIRMED_ROOM &&
             event == RoomTrackingEvent::PASSAGE_CROSSING_DETECTED)
    {
        const bool guard =
            crossing.passageDetected && crossing.passable &&
            crossing.dwell_s >= config_.crossing_dwell_s &&
            crossing.confidence >= config_.crossing_confidence;
        commit(source, event, now_s, crossing, verification, guard);
    }
    else if (source == RoomTrackingState::CROSSING_PASSAGE &&
             event == RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE)
    {
        const bool guard = crossing.bothSidesObserved &&
                           crossing.dwell_s >= config_.crossing_dwell_s &&
                           verification.pass;
        commit(source, event, now_s, crossing, verification, guard);
    }
    else if (source == RoomTrackingState::LOST_WITHOUT_ROOM &&
             event == RoomTrackingEvent::ROOM_REACQUIRED)
    {
        commit(source, event, now_s, crossing, verification, verification.pass);
    }
    else if (source == RoomTrackingState::LOST_WITH_LAST_ROOM &&
             event == RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH)
    {
        commit(source, event, now_s, crossing, verification, verification.pass);
    }
    else if (source == RoomTrackingState::REACQUIRING_IN_NEW_MAP &&
             event == RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM)
    {
        commit(source, event, now_s, crossing, verification, verification.pass);
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

void RoomTracker::commit(RoomTrackingState          source,
                         RoomTrackingEvent          event,
                         double                     now_s,
                         const TraversalGuardValues &crossing,
                         const VerificationVerdict  &verification,
                         bool                       accepted)
{
    TransitionEvent record;
    record.timestamp_s      = now_s;
    record.sourceState      = source;
    record.event            = event;
    record.dwell_s          = crossing.dwell_s;
    record.confidence       = crossing.confidence;
    record.verificationPass = verification.pass;
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

        state_                 = record.targetState;
        lastEnterStateTime_s_  = now_s;
        crossingDwellStartTime_s_ = -1.0;
        hasObservedBothSides_  = false;
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

} // namespace ORB_SLAM3