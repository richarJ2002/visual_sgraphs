/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
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

#include "SemanticsManager.h"

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{

void SemanticsManager::updateRoomTrackerState(double now_s)
{
    /* Consume the per-cycle signals. trackingLostPending_ is set on another
     * thread (System::TrackRGBD's real per-frame tracking state, and also
     * reachable via the on-demand System::GetMissionHealthSnapshot RPC), so
     * it is read and cleared under mMutexCurrentRoom. */
    bool                          crossingPending     = false;
    bool                          bothSidesPending    = false;
    bool                          trackingLostPending = false;
    semantic::VerificationVerdict verification;
    {
        std::lock_guard<std::mutex> currentRoomLock(mMutexCurrentRoom);
        crossingPending           = crossingEventPending_;
        bothSidesPending          = crossingBothSidesPending_;
        trackingLostPending       = trackingLostPending_;
        crossingEventPending_     = false;
        crossingBothSidesPending_ = false;
        trackingLostPending_      = false;
        if (verificationVerdictPending_)
        {
            verification                = verificationVerdict_;
            verificationVerdict_        = semantic::VerificationVerdict();
            verificationVerdictPending_ = false;
        }
    }

    /* Passage crossing evidence. segmentCrossesPassageOpening() already
     * required a passable passage; a detected crossing is therefore direct
     * geometric evidence and carries full traversal confidence until the
     * geometric verifier supplies a calibrated value. */
    semantic::TraversalGuardValues crossing;
    crossing.passageDetected   = crossingPending;
    crossing.passable          = crossingPending;
    crossing.confidence        = crossingPending ? 1.0 : 0.0;
    crossing.bothSidesObserved = bothSidesPending;

    semantic::TrackingStatusInput tracking;
    tracking.lost = trackingLostPending;
    pendingNewMapCreated_ =
        pendingNewMapCreated_ || p_atlas->consumeNewMapCreatedEvent();
    tracking.newMapCreated = pendingNewMapCreated_;
    if (tracking.lost && tracking.newMapCreated)
    {
        /* RoomTracker commits at most one row per cycle. Preserve the map event
         * for the following cycle instead of losing it behind TRACKING_LOST. */
        newMapCreatedDeferred_ = true;
        tracking.newMapCreated = false;
    }
    else
    {
        newMapCreatedDeferred_ = false;
    }

    roomTracker_.step(now_s, crossing, verification, tracking);
    const semantic::TransitionEvent &lastEvent = roomTracker_.getLastEvent();
    if (lastEvent.accepted &&
        (lastEvent.event ==
             semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH ||
         lastEvent.event == semantic::RoomTrackingEvent::LOST_TIMEOUT ||
         lastEvent.event == semantic::RoomTrackingEvent::REACQUIRE_TIMEOUT))
    {
        pendingNewMapCreated_ = false;
    }
}

} // namespace core
} // namespace vs_graphs
