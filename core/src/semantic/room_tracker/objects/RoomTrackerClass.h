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
 * @file            RoomTrackerClass.h
 *
 * @brief           Declares the room tracker: which room the camera is in, the
 *                  events that move it between rooms and their verification
 *                  verdicts.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKER_CLASS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKER_CLASS_H
#include "semantic/room_tracker/objects/RoomTrackerConfigStruct.h"
#include "semantic/room_tracker/objects/RoomTrackerStatusEnum.h"
#include "semantic/room_tracker/objects/RoomTrackingEventEnum.h"
#include "semantic/room_tracker/objects/RoomTrackingStateEnum.h"
#include "semantic/room_tracker/objects/TrackingStatusInputStruct.h"
#include "semantic/room_tracker/objects/TransitionEventStruct.h"
#include "semantic/room_tracker/objects/TraversalGuardValuesStruct.h"
#include "semantic/room_tracker/objects/VerificationStatusEnum.h"
#include "semantic/room_tracker/objects/VerificationVerdictStatusEnum.h"
#include "semantic/room_tracker/objects/VerificationVerdictStruct.h"

#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Room-state machine implementing the transition
 *                  table.
 *
 *                  Standalone and pure (no ROS, Eigen, PCL or Atlas
 *                  types) so it can be driven directly by
 *                  deterministic unit tests and by
 *                  SemanticsManager::Run.
 */
class RoomTracker
{
  public:
    /*!
     * @brief           Constructs a tracker with the given configuration.
     */
    explicit RoomTracker(
        const RoomTrackerConfig &configuration_in = RoomTrackerConfig()) :
        config(configuration_in)
    {}

    /*!
     * @brief           Resets state, timers, retry counters and event history.
     */
    [[nodiscard]] RoomTrackerStatus reset(double now_s_in);

    /*!
     * @brief           Per-cycle integration entry point.
     *
     *                  1. Advances time (never backwards).
     *                  2. Fires unconditional timeout transitions first.
     *                  3. Fires the tracking-lost transitions of the
     *                     transition table.
     *                  4. Evaluates the guarded transitions using the
     *                     supplied crossing evidence and verification
     *                     verdict, applying the dwell timers internally.
     *
     *                  At most one transition is committed per cycle.
     *
     * @param[in]       now_s_in
     *                  Monotonic seconds since an arbitrary epoch.
     *
     * @param[in]       crossing_in
     *                  Passage crossing evidence from
     *                  updateTraversalEvidence().
     *
     * @param[in]       verification_in
     *                  Abstract verification verdict (verification
     *                  stub).
     *
     * @param[in]       tracking_in
     *                  Tracking-loss and new-map lifecycle signals.
     *
     * @param[out]      nextState_out
     *                  The state after the cycle.
     *
     * @return          ROOM_TRACKER_STATUS_SUCCESS.
     */
    [[nodiscard]] RoomTrackerStatus
        step(double                      now_s_in,
             const TraversalGuardValues &crossing_in,
             const VerificationVerdict  &verification_in,
             const TrackingStatusInput  &tracking_in,
             RoomTrackingState          &nextState_out);

    /*!
     * @brief           Discrete transition oracle: applies exactly one
     *                  transition-table row and returns the resulting
     *                  state.
     *
     *                  Events with no row defined for the current source
     *                  state are rejected: the state is unchanged, a
     *                  rejected TransitionEvent is recorded and a WARN is
     *                  logged.
     *
     * @param[in]       event_in
     *                  The event to apply.
     *
     * @param[in]       now_s_in
     *                  Monotonic seconds used as the record timestamp.
     *
     * @param[in]       crossing_in
     *                  Explicit guard values for the guarded rows.
     *
     * @param[in]       verification_in
     *                  Explicit verification verdict for the guarded
     *                  rows.
     *
     * @param[out]      nextState_out
     *                  The state after applying the row.
     *
     * @return          ROOM_TRACKER_STATUS_SUCCESS.
     */
    [[nodiscard]] RoomTrackerStatus
        applyEvent(RoomTrackingEvent           event_in,
                   double                      now_s_in,
                   const TraversalGuardValues &crossing_in,
                   const VerificationVerdict  &verification_in,
                   RoomTrackingState          &nextState_out);

    /*!
     * @brief           Returns the current state.
     */
    [[nodiscard]] RoomTrackerStatus
        getState(RoomTrackingState &state_out) const;

    /*!
     * @brief           Returns every recorded TransitionEvent (accepted or
     *                  rejected), oldest first.
     */
    [[nodiscard]] RoomTrackerStatus getEventHistory(
        const std::vector<TransitionEvent> *&p_eventHistory_out) const;

    /*!
     * @brief           Returns the most recent TransitionEvent record.
     */
    [[nodiscard]] RoomTrackerStatus
        getLastEvent(const TransitionEvent *&p_lastEvent_out) const;

    /*!
     * @brief           Returns the tracker configuration.
     */
    [[nodiscard]] RoomTrackerStatus
        getConfig(const RoomTrackerConfig *&p_config_out) const;

    /*!
     * @brief           Renders a state as a stable literal name.
     */
    [[nodiscard]] static RoomTrackerStatus
        stateToString(RoomTrackingState state_in, std::string &text_out);

    /*!
     * @brief           Renders an event as a stable literal name.
     */
    [[nodiscard]] static RoomTrackerStatus
        eventToString(RoomTrackingEvent event_in, std::string &text_out);

    /*!
     * @brief           Serialises a TransitionEvent as one JSON object line.
     */
    [[nodiscard]] static RoomTrackerStatus
        eventToJSON(const TransitionEvent &event_in, std::string &json_out);

    /*!
     * @brief           Confidence formula:
     *
     *                  confidence = inlier_ratio * (1 -
     *                  normalised_condition_number)
     *                  * exp(-angular_residual / sigma_theta_rad)
     *
     *                  Non-finite or out-of-range inputs are clamped; a
     *                  non-positive sigma results in 1.0 for a zero
     *                  residual and 0.0 otherwise.
     */
    [[nodiscard]] static RoomTrackerStatus
        computeConfidence(double  inlierRatio_in,
                          double  normalizedConditionNumber_in,
                          double  angularResidual_rad_in,
                          double  sigmaTheta_rad_in,
                          double &confidence_out);

  private:
    /*!
     * @brief           Central row engine: applies the source row for (state,
     *                  event). Returns true when the transition was committed.
     */
    [[nodiscard]] RoomTrackerStatus
        applyRow(RoomTrackingState           source_in,
                 RoomTrackingEvent           event_in,
                 double                      now_s_in,
                 const TraversalGuardValues &crossing_in,
                 const VerificationVerdict  &verification_in,
                 bool                       &isAccepted_out);

    /*!
     * @brief           Commits target as the new state and records the event.
     */
    [[nodiscard]] RoomTrackerStatus
        commit(RoomTrackingState           source_in,
               RoomTrackingEvent           event_in,
               double                      now_s_in,
               const TraversalGuardValues &crossing_in,
               const VerificationVerdict  &verification_in,
               bool                        accepted_in);

    /*!
     * @brief           Accumulates the crossing/dwell timer for the given
     *                  state.
     *
     * @param[in]       state_in
     *                  Current tracking state (not used by the timer).
     *
     * @param[in]       now_s_in
     *                  Current time, in seconds.
     *
     * @param[in]       guardSatisfied_in
     *                  Whether the dwell guard holds now; false restarts the
     *                  timer.
     *
     * @param[out]      accumulatedDwell_out
     *                  Seconds elapsed since the guard became satisfied, or 0
     *                  when it is not.
     *
     * @return          ROOM_TRACKER_STATUS_SUCCESS.
     */
    [[nodiscard]] RoomTrackerStatus
        accumulateDwell(RoomTrackingState state_in,
                        double            now_s_in,
                        bool              guardSatisfied_in,
                        double           &accumulatedDwell_out);

  private:
    /*!
     * @brief           Tuning parameters this tracker was constructed with.
     */
    RoomTrackerConfig            config;
    /*!
     * @brief           Current state of the tracker.
     */
    RoomTrackingState            trackingState = RoomTrackingState::UNKNOWN;
    /*!
     * @brief           Every transition attempt, accepted or rejected, oldest
     *                  first; cleared by reset().
     */
    std::vector<TransitionEvent> eventHistory;
    /*!
     * @brief           Most recent transition record; default-constructed until
     *                  the first attempt.
     */
    TransitionEvent              lastEvent;

    /*!
     * @brief           Latest time step() has seen, seconds; step() never lets
     *                  time run backwards.
     */
    double lastReceivedTime_s = 0.0;
    /*!
     * @brief           Time the current state was entered, seconds; drives the
     *                  lost and reacquire timeouts.
     */
    double lastEnterStateTime_s = 0.0;
    /*!
     * @brief           Time the crossing guard first held, seconds; -1 when the
     *                  dwell timer is not running.
     */
    double crossingDwellStartTime_s = -1.0;

    /*!
     * @brief           Failed reacquire attempts since entering
     *                  REACQUIRING_IN_NEW_MAP.
     */
    unsigned int reacquireRetryCount = 0U;
    /*!
     * @brief           Time of the latest reacquire attempt, seconds; -1 when
     *                  none has been made yet.
     */
    double       reacquireLastRetryTime_s = -1.0;

    /*!
     * @brief           True once both sides of the crossed passage have been
     *                  observed during the current crossing.
     */
    bool hasObservedBothSides = false;
    /*!
     * @brief           Tracking-lost flag from the previous step() cycle, used
     *                  to detect the moment tracking is newly lost.
     */
    bool wasTrackingLost = false;
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_ROOM_TRACKER_CLASS_H \
        */
