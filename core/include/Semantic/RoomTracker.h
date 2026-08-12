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

#ifndef ROOMTRACKER_H
#define ROOMTRACKER_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ORB_SLAM3
{
/*!
 * @brief           Lifecycle states of the camera relative to the mapped rooms.
 *
 *                  Mirrors Section 18.1 of WP13. The state machine is
 *                  self-contained: it carries only state names, event labels,
 *                  timestamps and scalar guard values, and is therefore
 *                  frame-agnostic.
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
 * @brief           Events consumed by the transition engine.
 *
 *                  The set of (source, event) rows is exactly the Section 18.2
 *                  table. An event with no defined row for the current source
 *                  state is rejected: the state does not change and a
 *                  rejection record is logged.
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
 * @brief           Guard values describing one transition attempt.
 *
 * @param dwell_s
 *                  Continuous seconds the guard has been satisfied. The caller
 *                  (RoomTracker::step in production, tests in unit test runs)
 *                  supplies this; RoomTracker::step accumulates it across the
 *                  Section 18.3 dwell timers.
 * @param confidence
 *                  Traversal/verification confidence in [0, 1] (Section 18.3).
 * @param passageDetected
 *                  A trajectory segment crossed a passable passage aperture
 *                  (segmentCrossesPassageOpening()).
 * @param passable
 *                  The crossed passage is passable (Passage::isPassable()).
 * @param bothSidesObserved
 *                  Both sides of the passage have been observed.
 */
struct TraversalGuardValues
{
    double dwell_s = 0.0;
    double confidence = 0.0;
    bool   passageDetected = false;
    bool   passable = false;
    bool   bothSidesObserved = false;
};

/*!
 * @brief           Abstract verification-result event.
 *
 *                  Phase 4 supplies the real plane-gated geometric verifier;
 *                  until then callers populate this with synthetic pass/fail
 *                  values (Section 19.2). Only the verdict and confidence are
 *                  consumed by the transition engine in Phase 1.
 */
struct VerificationVerdict
{
    bool          pass = false;
    unsigned int  inlierCount = 0U;
    double        inlierRatio = 0.0;
    double        normalisedConditionNumber = 0.0;
    double        angularResidual_rad = 0.0;
    double        confidence = 0.0;
};

/*!
 * @brief           Tracking and map-lifecycle signals fed to RoomTracker::step.
 *
 * @param lost
 *                  Tracking was declared lost for the current cycle.
 * @param newMapCreated
 *                  A new map was created while tracking was lost.
 */
struct TrackingStatusInput
{
    bool lost = false;
    bool newMapCreated = false;
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
    RoomTrackingEvent event = RoomTrackingEvent::FIRST_ROOM_CONFIRMED;
    double            dwell_s = 0.0;
    double            confidence = 0.0;
    bool              verificationPass = false;
    bool              accepted = false;
};

/*!
 * @brief           RoomTracker tuning (Section 18.4).
 */
struct RoomTrackerConfig
{
    /*! Minimum continuous crossed-passage dwell before committing the
     *  CONFIRMED_ROOM <-> CROSSING_PASSAGE transitions (seconds). */
    double crossing_dwell_s = 2.0;
    /*! Minimum traversal confidence (0..1) for a crossing to count. */
    double crossing_confidence = 0.7;
    /*! Maximum time in LOST_WITH_LAST_ROOM before decay to
     *  LOST_WITHOUT_ROOM (seconds). */
    double lost_timeout_s = 30.0;
    /*! Maximum time in REACQUIRING_IN_NEW_MAP before decay to
     *  LOST_WITHOUT_ROOM (seconds). */
    double reacquire_timeout_s = 60.0;
    /*! Retry backoff between failed reacquire attempts (seconds). */
    double reacquire_retry_interval_s = 5.0;
    /*! Maximum failed reacquire attempts before timeout applies. */
    unsigned int reacquire_max_retries = 3U;
    /*! Minimum planes required to attempt a reacquire. Consumed by the
     *  Phase 4 verification stub; acceptance still requires the full
     *  verification gates. */
    unsigned int reacquire_min_planes = 3U;
};

/*!
 * @brief           Room-state machine implementing the Section 18.2 table.
 *
 *                  Standalone and pure (no ROS, Eigen, PCL or Atlas types) so
 *                  it can be driven directly by deterministic unit tests and
 *                  by SemanticsManager::Run.
 */
class RoomTracker
{
  public:
    /*!
     * @brief       Constructs a tracker with the given configuration.
     */
    explicit RoomTracker(const RoomTrackerConfig &config_in = RoomTrackerConfig());

    /*!
     * @brief       Resets state, timers, retry counters and event history.
     */
    void reset(double now_s);

    /*!
     * @brief       Per-cycle integration entry point.
     *
     *               1. Advances time (never backwards).
     *               2. Fires unconditional timeout transitions first.
     *               3. Fires the tracking-lost transitions of Section 18.2.
     *               4. Evaluates the guarded transitions using the supplied
     *                  crossing evidence and verification verdict, applying
     *                  the Section 18.3 dwell timers internally.
     *
     *                At most one transition is committed per cycle.
     *
     * @param[in]   now_s
     *              Monotonic seconds since an arbitrary epoch.
     * @param[in]   crossing
     *              Passage crossing evidence from updateTraversalEvidence().
     * @param[in]   verification
     *              Abstract verification verdict (Phase 4 stub in Phase 1).
     * @param[in]   tracking
     *              Tracking-loss and new-map lifecycle signals.
     *
     * @return      The state after the cycle.
     */
    RoomTrackingState step(double                        now_s,
                           const TraversalGuardValues   &crossing,
                           const VerificationVerdict    &verification,
                           const TrackingStatusInput    &tracking);

    /*!
     * @brief       Discrete transition oracle: applies exactly one Section 18.2
     *              row and returns the resulting state.
     *
     *              Events with no row defined for the current source state are
     *              rejected: the state is unchanged, a rejected TransitionEvent
     *              is recorded and a WARN is logged.
     *
     * @param[in]   event
     *              The event to apply.
     * @param[in]   now_s
     *              Monotonic seconds used as the record timestamp.
     * @param[in]   crossing
     *              Explicit guard values for the guarded rows.
     * @param[in]   verification
     *              Explicit verification verdict for the guarded rows.
     *
     * @return      The state after applying the row.
     */
    RoomTrackingState applyEvent(RoomTrackingEvent         event,
                                 double                    now_s,
                                 const TraversalGuardValues &crossing,
                                 const VerificationVerdict  &verification);

    /*!
     * @brief       Returns the current state.
     */
    RoomTrackingState getState() const;

    /*!
     * @brief       Returns every recorded TransitionEvent (accepted or
     *              rejected), oldest first.
     */
    const std::vector<TransitionEvent> &getEventHistory() const;

    /*!
     * @brief       Returns the most recent TransitionEvent record.
     */
    const TransitionEvent &getLastEvent() const;

    /*!
     * @brief       Returns the tracker configuration.
     */
    const RoomTrackerConfig &getConfig() const;

    /*!
     * @brief       Renders a state as a stable literal name.
     */
    static std::string stateToString(RoomTrackingState state);

    /*!
     * @brief       Renders an event as a stable literal name.
     */
    static std::string eventToString(RoomTrackingEvent event);

    /*!
     * @brief       Serialises a TransitionEvent as one JSON object line.
     */
    static std::string eventToJSON(const TransitionEvent &event);

    /*!
     * @brief       Section 18.3 confidence formula:
     *
     *              confidence = inlier_ratio * (1 - normalised_condition_number)
     *                           * exp(-angular_residual / sigma_theta_rad)
     *
     *              Non-finite or out-of-range inputs are clamped; a
     *              non-positive sigma results in 1.0 for a zero residual and
     *              0.0 otherwise.
     */
    static double computeConfidence(double inlier_ratio,
                                    double normalised_condition_number,
                                    double angular_residual_rad,
                                    double sigma_theta_rad);

  private:
    /*!
     * @brief       Central row engine: applies the source row for (state,
     *              event). Returns true when the transition was committed.
     */
    bool applyRow(RoomTrackingState         source,
                  RoomTrackingEvent         event,
                  double                    now_s,
                  const TraversalGuardValues &crossing,
                  const VerificationVerdict  &verification);

    /*!
     * @brief       Commits target as the new state and records the event.
     */
    void commit(RoomTrackingState         source,
                RoomTrackingEvent         event,
                double                    now_s,
                const TraversalGuardValues &crossing,
                const VerificationVerdict  &verification,
                bool                      accepted);

    /*!
     * @brief       Accumulates the crossing/dwell timer for the given state.
     *
     * @return      Dwell seconds elapsed so far (accumulated countdown).
     */
    double accumulateDwell(RoomTrackingState state,
                           double            now_s,
                           bool              guardSatisfied);

  private:
    RoomTrackerConfig             config_;
    RoomTrackingState             state_ = RoomTrackingState::UNKNOWN;
    std::vector<TransitionEvent>  eventHistory_;
    TransitionEvent               lastEvent_;

    double lastReceivedTime_s_ = 0.0;
    double lastEnterStateTime_s_ = 0.0;
    double crossingDwellStartTime_s_ = -1.0;

    unsigned int reacquireRetryCount_ = 0U;
    double       reacquireLastRetryTime_s_ = -1.0;

    bool  hasObservedBothSides_ = false;
    bool  wasTrackingLost_ = false;
};

} // namespace ORB_SLAM3

#endif // ROOMTRACKER_H