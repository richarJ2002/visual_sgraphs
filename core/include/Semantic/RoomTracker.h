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
 * @file            RoomTracker.h
 *
 * @brief           Declares the room tracker: which room the camera is in, the
 *                  events that move it between rooms and their verification
 *                  verdicts.
 */

#ifndef ROOMTRACKER_H
#define ROOMTRACKER_H
#include "Semantic/RoomTrackerStatus.h"
#include "Semantic/VerificationVerdictStatus.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief        Lifecycle states of the camera relative to the
 *               mapped rooms.
 *
 *               The state machine is self-contained: it carries
 *               only state names, event labels, timestamps and
 *               scalar guard values, and is therefore
 *               frame-agnostic.
 */
enum class RoomTrackingState
{
    UNKNOWN,
    CONFIRMED_ROOM,
    CROSSING_PASSAGE,
    LOST_WITHOUT_ROOM,
    LOST_WITH_LAST_ROOM,
    REACQUIRING_IN_NEW_MAP
};

/*!
 * @brief        Events consumed by the transition engine.
 *
 *               The set of (source, event) rows is exactly the
 *               transition table. An event with no defined row for
 *               the current source state is rejected: the state
 *               does not change and a rejection record is logged.
 */
enum class RoomTrackingEvent
{
    FIRST_ROOM_CONFIRMED,
    PASSAGE_CROSSING_DETECTED,
    TRACKING_LOST,
    PASSAGE_TRAVERSAL_COMPLETE,
    ROOM_REACQUIRED,
    NEW_MAP_WITH_ROOM_MATCH,
    VERIFIED_MATCH_TO_LAST_ROOM,
    LOST_TIMEOUT,
    REACQUIRE_TIMEOUT
};

/*!
 * @brief        Guard values describing one transition attempt.
 */
struct TraversalGuardValues
{
    /*!
     * @brief    Continuous time the guard has been satisfied. The caller
     *           (RoomTracker::step in production, the tests in unit tests)
     *           supplies it; RoomTracker::step accumulates it across the
     *           dwell timers.
     *
     * @units    seconds
     */
    double dwell_s = 0.0;

    /*!
     * @brief    Traversal/verification confidence in [0, 1].
     */
    double confidence = 0.0;

    /*!
     * @brief    A trajectory segment crossed a passable passage aperture
     *           (segmentCrossesPassageOpening()).
     */
    bool isPassageDetected = false;

    /*!
     * @brief    The crossed passage is passable (Passage::isPassable()).
     */
    bool isPassable = false;

    /*!
     * @brief    Both sides of the passage have been observed.
     */
    bool areBothSidesObserved = false;
};

/*!
 * @brief        Abstract verification-result event.
 *
 *               Production remains UNAVAILABLE until a future typed
 *               geometric-verifier producer supplies a result. Tests
 *               may inject deterministic values. Only the verdict
 *               and confidence are consumed by the transition
 *               engine.
 */
enum class VerificationStatus
{
    UNAVAILABLE,
    PASS,
    REJECTED
};

/*!
 * @brief        Result of a geometric room-match verification: whether it
 *               passed and the numbers behind that verdict.
 */
struct VerificationVerdict
{
    /*!
     * @brief        Whether a verifier produced a result at all, and if so
     *               PASS or REJECTED.
     */
    VerificationStatus status = VerificationStatus::UNAVAILABLE;
    /*!
     * @brief        Verifier's own pass flag; isPass() also needs status ==
     *               PASS and every metric in range.
     */
    bool               hasPassed = false;
    /*!
     * @brief        Number of inliers the verifier counted.
     */
    unsigned int       inlierCount = 0U;
    /*!
     * @brief        Fraction of inliers among all matches, in [0, 1].
     */
    double             inlierRatio = 0.0;
    /*!
     * @brief        Condition number of the verification normalised to
     *               [0, 1]; confidence falls as it rises.
     */
    double             normalisedConditionNumber = 0.0;
    /*!
     * @brief        Angular residual of the verification, radians, at least
     *               0.
     */
    double             angularResidual_rad = 0.0;
    /*!
     * @brief        Overall confidence of the verdict, in [0, 1].
     */
    double             confidence = 0.0;

    /*! Returns true only for a finite, internally consistent PASS. */
    [[nodiscard]] VerificationVerdictStatus isPass(bool &isPass_out) const
    {
        isPass_out =
            status == VerificationStatus::PASS && hasPassed &&
            std::isfinite(inlierRatio) && inlierRatio >= 0.0 &&
            inlierRatio <= 1.0 && std::isfinite(normalisedConditionNumber) &&
            normalisedConditionNumber >= 0.0 &&
            normalisedConditionNumber <= 1.0 &&
            std::isfinite(angularResidual_rad) && angularResidual_rad >= 0.0 &&
            std::isfinite(confidence) && confidence >= 0.0 && confidence <= 1.0;
        return VerificationVerdictStatus::VERIFICATION_VERDICT_STATUS_SUCCESS;
    }
};

/*!
 * @brief           Tracking and map-lifecycle signals fed to RoomTracker::step.
 */
struct TrackingStatusInput
{
    /*!
     * @brief       Tracking was declared lost for the current cycle.
     */
    bool isLost = false;

    /*!
     * @brief       A new map was created while tracking was lost.
     */
    bool isNewMapCreated = false;
};

/*!
 * @brief           Per-transition record.
 *
 *                  Records both accepted and rejected transitions so tests and
 *                  diagnostics can verify that a rejected event changed no
 *                  state. Serialised as a JSON line by eventToJSON().
 */
struct TransitionEvent
{
    /*!
     * @brief        Monotonic time when the transition was attempted, seconds.
     */
    double            timestamp_s = 0.0;
    /*!
     * @brief        State the tracker was in before the attempt.
     */
    RoomTrackingState sourceState = RoomTrackingState::UNKNOWN;
    /*!
     * @brief        State after the attempt; equals sourceState when the
     *               transition was rejected.
     */
    RoomTrackingState targetState = RoomTrackingState::UNKNOWN;
    /*!
     * @brief        Event that triggered the attempt.
     */
    RoomTrackingEvent event = RoomTrackingEvent::FIRST_ROOM_CONFIRMED;
    /*!
     * @brief        Continuous guard dwell time supplied with the attempt,
     *               seconds.
     */
    double            dwell_s = 0.0;
    /*!
     * @brief        Traversal confidence supplied with the attempt, in [0, 1].
     */
    double            confidence = 0.0;
    /*!
     * @brief        True when the verification verdict supplied with the
     *               attempt passed isPass().
     */
    bool              hasVerificationPassed = false;
    /*!
     * @brief        True when the transition was committed, false when it was
     *               rejected and the state stayed unchanged.
     */
    bool              isAccepted = false;
};

/*!
 * @brief        RoomTracker tuning parameters.
 */
struct RoomTrackerConfig
{
    /*! Minimum continuous crossed-passage dwell before committing the
     *  CONFIRMED_ROOM <-> CROSSING_PASSAGE transitions (seconds). */
    double       crossing_dwell_s = 2.0;
    /*! Minimum traversal confidence (0..1) for a crossing to count. */
    double       crossing_confidence = 0.7;
    /*! Maximum time in LOST_WITH_LAST_ROOM before decay to
     *  LOST_WITHOUT_ROOM (seconds). */
    double       lost_timeout_s = 30.0;
    /*! Maximum time in REACQUIRING_IN_NEW_MAP before decay to
     *  LOST_WITHOUT_ROOM (seconds). */
    double       reacquire_timeout_s = 60.0;
    /*! Retry backoff between failed reacquire attempts (seconds). */
    double       reacquire_retry_interval_s = 5.0;
    /*! Maximum failed reacquire attempts before timeout applies. */
    unsigned int reacquire_max_retries = 3U;
    /*!
     * @brief        Minimum planes required to attempt a reacquire.
     *               Consumed by the verification stub; acceptance
     *               still requires the full verification gates.
     */
    unsigned int reacquire_min_planes = 3U;
};

/*!
 * @brief        Room-state machine implementing the transition
 *               table.
 *
 *               Standalone and pure (no ROS, Eigen, PCL or Atlas
 *               types) so it can be driven directly by
 *               deterministic unit tests and by
 *               SemanticsManager::Run.
 */
class RoomTracker
{
  public:
    /*!
     * @brief       Constructs a tracker with the given configuration.
     */
    explicit RoomTracker(
        const RoomTrackerConfig &configuration_in = RoomTrackerConfig()) :
        config(configuration_in)
    {}

    /*!
     * @brief       Resets state, timers, retry counters and event history.
     */
    [[nodiscard]] RoomTrackerStatus reset(double now_s_in);

    /*!
     * @brief        Per-cycle integration entry point.
     *
     *               1. Advances time (never backwards).
     *               2. Fires unconditional timeout transitions first.
     *               3. Fires the tracking-lost transitions of the
     *               transition table.
     *               4. Evaluates the guarded transitions using the
     *               supplied crossing evidence and verification
     *               verdict, applying the dwell timers internally.
     *
     *               At most one transition is committed per cycle.
     *
     * @param[in]    now_s_in
     *               Monotonic seconds since an arbitrary epoch.
     *
     * @param[in]    crossing_in
     *               Passage crossing evidence from
     *               updateTraversalEvidence().
     *
     * @param[in]    verification_in
     *               Abstract verification verdict (verification
     *               stub).
     *
     * @param[in]    tracking_in
     *               Tracking-loss and new-map lifecycle signals.
     *
     * @param[out] nextState_out The state after the cycle.
     * @return ROOM_TRACKER_STATUS_SUCCESS.
     */
    [[nodiscard]] RoomTrackerStatus
        step(double                      now_s_in,
             const TraversalGuardValues &crossing_in,
             const VerificationVerdict  &verification_in,
             const TrackingStatusInput  &tracking_in,
             RoomTrackingState          &nextState_out);

    /*!
     * @brief        Discrete transition oracle: applies exactly one
     *               transition-table row and returns the resulting
     *               state.
     *
     *               Events with no row defined for the current source
     *               state are rejected: the state is unchanged, a
     *               rejected TransitionEvent is recorded and a WARN is
     *               logged.
     *
     * @param[in]    event_in
     *               The event to apply.
     *
     * @param[in]    now_s_in
     *               Monotonic seconds used as the record timestamp.
     *
     * @param[in]    crossing_in
     *               Explicit guard values for the guarded rows.
     *
     * @param[in]    verification_in
     *               Explicit verification verdict for the guarded
     *               rows.
     *
     * @param[out] nextState_out The state after applying the row.
     * @return ROOM_TRACKER_STATUS_SUCCESS.
     */
    [[nodiscard]] RoomTrackerStatus
        applyEvent(RoomTrackingEvent           event_in,
                   double                      now_s_in,
                   const TraversalGuardValues &crossing_in,
                   const VerificationVerdict  &verification_in,
                   RoomTrackingState          &nextState_out);

    /*!
     * @brief       Returns the current state.
     */
    [[nodiscard]] RoomTrackerStatus
        getState(RoomTrackingState &state_out) const;

    /*!
     * @brief       Returns every recorded TransitionEvent (accepted or
     *              rejected), oldest first.
     */
    [[nodiscard]] RoomTrackerStatus getEventHistory(
        const std::vector<TransitionEvent> *&p_eventHistory_out) const;

    /*!
     * @brief       Returns the most recent TransitionEvent record.
     */
    [[nodiscard]] RoomTrackerStatus
        getLastEvent(const TransitionEvent *&p_lastEvent_out) const;

    /*!
     * @brief       Returns the tracker configuration.
     */
    [[nodiscard]] RoomTrackerStatus
        getConfig(const RoomTrackerConfig *&p_config_out) const;

    /*!
     * @brief       Renders a state as a stable literal name.
     */
    [[nodiscard]] static RoomTrackerStatus
        stateToString(RoomTrackingState state_in, std::string &text_out);

    /*!
     * @brief       Renders an event as a stable literal name.
     */
    [[nodiscard]] static RoomTrackerStatus
        eventToString(RoomTrackingEvent event_in, std::string &text_out);

    /*!
     * @brief       Serialises a TransitionEvent as one JSON object line.
     */
    [[nodiscard]] static RoomTrackerStatus
        eventToJSON(const TransitionEvent &event_in, std::string &json_out);

    /*!
     * @brief        Confidence formula:
     *
     *               confidence = inlier_ratio * (1 -
     *               normalised_condition_number)
     *               * exp(-angular_residual / sigma_theta_rad)
     *
     *               Non-finite or out-of-range inputs are clamped; a
     *               non-positive sigma results in 1.0 for a zero
     *               residual and 0.0 otherwise.
     */
    [[nodiscard]] static RoomTrackerStatus
        computeConfidence(double  inlierRatio_in,
                          double  normalizedConditionNumber_in,
                          double  angularResidual_rad_in,
                          double  sigmaTheta_rad_in,
                          double &confidence_out);

  private:
    /*!
     * @brief       Central row engine: applies the source row for (state,
     *              event). Returns true when the transition was committed.
     */
    [[nodiscard]] RoomTrackerStatus
        applyRow(RoomTrackingState           source_in,
                 RoomTrackingEvent           event_in,
                 double                      now_s_in,
                 const TraversalGuardValues &crossing_in,
                 const VerificationVerdict  &verification_in,
                 bool                       &isAccepted_out);

    /*!
     * @brief       Commits target as the new state and records the event.
     */
    [[nodiscard]] RoomTrackerStatus
        commit(RoomTrackingState           source_in,
               RoomTrackingEvent           event_in,
               double                      now_s_in,
               const TraversalGuardValues &crossing_in,
               const VerificationVerdict  &verification_in,
               bool                        accepted_in);

    /*!
     * @brief       Accumulates the crossing/dwell timer for the given state.
     *
     * @param[in]   state_in
     *              Current tracking state (not used by the timer).
     *
     * @param[in]   now_s_in
     *              Current time, in seconds.
     *
     * @param[in]   guardSatisfied_in
     *              Whether the dwell guard holds now; false restarts the
     *              timer.
     *
     * @param[out]  accumulatedDwell_out
     *              Seconds elapsed since the guard became satisfied, or 0
     *              when it is not.
     *
     * @return      ROOM_TRACKER_STATUS_SUCCESS.
     */
    [[nodiscard]] RoomTrackerStatus
        accumulateDwell(RoomTrackingState state_in,
                        double            now_s_in,
                        bool              guardSatisfied_in,
                        double           &accumulatedDwell_out);

  private:
    /*!
     * @brief        Tuning parameters this tracker was constructed with.
     */
    RoomTrackerConfig            config;
    /*!
     * @brief        Current state of the tracker.
     */
    RoomTrackingState            trackingState = RoomTrackingState::UNKNOWN;
    /*!
     * @brief        Every transition attempt, accepted or rejected, oldest
     *               first; cleared by reset().
     */
    std::vector<TransitionEvent> eventHistory;
    /*!
     * @brief        Most recent transition record; default-constructed until
     *               the first attempt.
     */
    TransitionEvent              lastEvent;

    /*!
     * @brief        Latest time step() has seen, seconds; step() never lets
     *               time run backwards.
     */
    double lastReceivedTime_s = 0.0;
    /*!
     * @brief        Time the current state was entered, seconds; drives the
     *               lost and reacquire timeouts.
     */
    double lastEnterStateTime_s = 0.0;
    /*!
     * @brief        Time the crossing guard first held, seconds; -1 when the
     *               dwell timer is not running.
     */
    double crossingDwellStartTime_s = -1.0;

    /*!
     * @brief        Failed reacquire attempts since entering
     *               REACQUIRING_IN_NEW_MAP.
     */
    unsigned int reacquireRetryCount = 0U;
    /*!
     * @brief        Time of the latest reacquire attempt, seconds; -1 when
     *               none has been made yet.
     */
    double       reacquireLastRetryTime_s = -1.0;

    /*!
     * @brief        True once both sides of the crossed passage have been
     *               observed during the current crossing.
     */
    bool hasObservedBothSides = false;
    /*!
     * @brief        Tracking-lost flag from the previous step() cycle, used
     *               to detect the moment tracking is newly lost.
     */
    bool wasTrackingLost = false;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // ROOMTRACKER_H
