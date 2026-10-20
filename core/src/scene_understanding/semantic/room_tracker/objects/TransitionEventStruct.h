/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            TransitionEventStruct.h
 *
 * @brief           Declares TransitionEvent, the per-transition record.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_TRANSITION_EVENT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_TRANSITION_EVENT_STRUCT_H

#include "scene_understanding/semantic/room_tracker/objects/RoomTrackingEventEnum.h"
#include "scene_understanding/semantic/room_tracker/objects/RoomTrackingStateEnum.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

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
     * @brief           Monotonic time when the transition was attempted,
     *                  seconds.
     */
    double            timestamp_s = 0.0;
    /*!
     * @brief           State the tracker was in before the attempt.
     */
    RoomTrackingState sourceState = RoomTrackingState::UNKNOWN;
    /*!
     * @brief           State after the attempt; equals sourceState when the
     *                  transition was rejected.
     */
    RoomTrackingState targetState = RoomTrackingState::UNKNOWN;
    /*!
     * @brief           Event that triggered the attempt.
     */
    RoomTrackingEvent event = RoomTrackingEvent::FIRST_ROOM_CONFIRMED;
    /*!
     * @brief           Continuous guard dwell time supplied with the attempt,
     *                  seconds.
     */
    double            dwell_s = 0.0;
    /*!
     * @brief           Traversal confidence supplied with the attempt, in [0,
     *                  1].
     */
    double            confidence = 0.0;
    /*!
     * @brief           True when the verification verdict supplied with the
     *                  attempt passed isPass().
     */
    bool              hasVerificationPassed = false;
    /*!
     * @brief           True when the transition was committed, false when it
     *                  was rejected and the state stayed unchanged.
     */
    bool              isAccepted = false;
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_TRACKER_OBJECTS_TRANSITION_EVENT_STRUCT_H \
        */
