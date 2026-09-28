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

#ifndef ROOMTRACKER_H
#define ROOMTRACKER_H
#include "Semantic/RoomTrackerStatus.h"

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
 *
 * @param[in]    dwell_s
 *               Continuous seconds the guard has been satisfied.
 *               The caller (RoomTracker::step in production, tests
 *               in unit test runs) supplies this;
 *               RoomTracker::step accumulates it across the dwell
 *               timers.
 *
 * @param[in]    confidence
 *               Traversal/verification confidence in [0, 1].
 *
 * @param[in]    isPassageDetected
 *               A trajectory segment crossed a passable passage
 *               aperture (segmentCrossesPassageOpening()).
 *
 * @param[in]    passable
 *               The crossed passage is passable
 *               (Passage::isPassable()).
 *
 * @param[in]    areBothSidesObserved
 *               Both sides of the passage have been observed.
 */
struct TraversalGuardValues
{
    double dwell_s              = 0.0;
    double confidence           = 0.0;
    bool   isPassageDetected    = false;
    bool   isPassable           = false;
    bool   areBothSidesObserved = false;
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

struct VerificationVerdict
{
    VerificationStatus status      = VerificationStatus::UNAVAILABLE;
    bool               hasPassed   = false;
    unsigned int       inlierCount = 0U;
    double             inlierRatio = 0.0;
    double             normalisedConditionNumber = 0.0;
    double             angularResidual_rad       = 0.0;
    double             confidence                = 0.0;

    /*! Returns true only for a finite, internally consistent PASS. */
    bool isPass() const
    {
        return status == VerificationStatus::PASS && hasPassed &&
               std::isfinite(inlierRatio) && inlierRatio >= 0.0 &&
               inlierRatio <= 1.0 && std::isfinite(normalisedConditionNumber) &&
               normalisedConditionNumber >= 0.0 &&
               normalisedConditionNumber <= 1.0 &&
               std::isfinite(angularResidual_rad) &&
               angularResidual_rad >= 0.0 && std::isfinite(confidence) &&
               confidence >= 0.0 && confidence <= 1.0;
    }
};

/*!
 * @brief           Tracking and map-lifecycle signals fed to RoomTracker::step.
 *
 * @param lost
 *                  Tracking was declared lost for the current cycle.
 * @param isNewMapCreated
 *                  A new map was created while tracking was lost.
 */
struct TrackingStatusInput
{
    bool isLost          = false;
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
    double            timestamp_s = 0.0;
    RoomTrackingState sourceState = RoomTrackingState::UNKNOWN;
    RoomTrackingState targetState = RoomTrackingState::UNKNOWN;
    RoomTrackingEvent event       = RoomTrackingEvent::FIRST_ROOM_CONFIRMED;
    double            dwell_s     = 0.0;
    double            confidence  = 0.0;
    bool              hasVerificationPassed = false;
    bool              isAccepted            = false;
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
     * @param[out] accumulatedDwell_out Dwell seconds elapsed so far
     * (accumulated countdown).
     * @return ROOM_TRACKER_STATUS_SUCCESS.
     */
    [[nodiscard]] RoomTrackerStatus
        accumulateDwell(RoomTrackingState state_in,
                        double            now_s_in,
                        bool              guardSatisfied_in,
                        double           &accumulatedDwell_out);

  private:
    RoomTrackerConfig            config;
    RoomTrackingState            trackingState = RoomTrackingState::UNKNOWN;
    std::vector<TransitionEvent> eventHistory;
    TransitionEvent              lastEvent;

    double lastReceivedTime_s       = 0.0;
    double lastEnterStateTime_s     = 0.0;
    double crossingDwellStartTime_s = -1.0;

    unsigned int reacquireRetryCount      = 0U;
    double       reacquireLastRetryTime_s = -1.0;

    bool hasObservedBothSides = false;
    bool wasTrackingLost      = false;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // ROOMTRACKER_H
