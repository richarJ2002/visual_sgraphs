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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::updateRoomTrackerState(double now_s_in)
{
    /* Consume the per-cycle signals. isTrackingLostPending is set on another
     * thread (System::TrackRGBD's real per-frame tracking state, and also
     * reachable via the on-demand System::GetMissionHealthSnapshot RPC), so
     * it is read and cleared under currentRoomMutex. */
    bool                          crossingPending     = false;
    bool                          bothSidesPending    = false;
    bool                          trackingLostPending = false;
    semantic::VerificationVerdict verification;
    {
        std::lock_guard<std::mutex> currentRoomLock(currentRoomMutex);
        crossingPending            = isCrossingEventPending;
        bothSidesPending           = isCrossingBothSidesPending;
        trackingLostPending        = isTrackingLostPending;
        isCrossingEventPending     = false;
        isCrossingBothSidesPending = false;
        isTrackingLostPending      = false;
        if (isVerificationVerdictPending)
        {
            verification                 = verificationVerdict;
            verificationVerdict          = semantic::VerificationVerdict();
            isVerificationVerdictPending = false;
        }
    }

    /* Passage crossing evidence. segmentCrossesPassageOpening() already
     * required a passable passage; a detected crossing is therefore direct
     * geometric evidence and carries full traversal confidence until the
     * geometric verifier supplies a calibrated value. */
    semantic::TraversalGuardValues crossing;
    crossing.isPassageDetected    = crossingPending;
    crossing.isPassable           = crossingPending;
    crossing.confidence           = crossingPending ? 1.0 : 0.0;
    crossing.areBothSidesObserved = bothSidesPending;

    semantic::TrackingStatusInput tracking;
    tracking.isLost = trackingLostPending;
    bool atlasWasEventPending{};
    if (!(isNewMapCreatedPending) &&
        p_atlas->consumeNewMapCreatedEvent(atlasWasEventPending) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: consumeNewMapCreatedEvent returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    isNewMapCreatedPending   = isNewMapCreatedPending || atlasWasEventPending;
    tracking.isNewMapCreated = isNewMapCreatedPending;
    if (tracking.isLost && tracking.isNewMapCreated)
    {
        /* RoomTracker commits at most one row per cycle. Preserve the map event
         * for the following cycle instead of losing it behind TRACKING_LOST. */
        isNewMapCreatedDeferred  = true;
        tracking.isNewMapCreated = false;
    }
    else
    {
        isNewMapCreatedDeferred = false;
    }

    semantic::RoomTrackingState roomTrackerNextState{};
    if (roomTracker.step(now_s_in,
                         crossing,
                         verification,
                         tracking,
                         roomTrackerNextState) !=
        semantic::RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: step returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const semantic::TransitionEvent *p_lastEventRef = nullptr;
    if (roomTracker.getLastEvent(p_lastEventRef) !=
        semantic::RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLastEvent returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const semantic::TransitionEvent &lastEvent = *p_lastEventRef;
    if (lastEvent.isAccepted &&
        (lastEvent.event ==
             semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH ||
         lastEvent.event == semantic::RoomTrackingEvent::LOST_TIMEOUT ||
         lastEvent.event == semantic::RoomTrackingEvent::REACQUIRE_TIMEOUT))
    {
        isNewMapCreatedPending = false;
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
