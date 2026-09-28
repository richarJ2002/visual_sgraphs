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
 *
 * Confirmed: semantic::RoomTracker implements the transition table of
 * exactly 10 transition rows (6 guarded, 4 unconditional). This suite covers:
 *   (a) each guarded row with a guard-satisfied and a guard-rejected case;
 *   (b) each unconditional row as an event-triggered target-state test;
 *   (c) every source state rejecting events with no defined row.
 * Each trajectory is replayed several times with jittered timestamps.
 */

#include "Semantic/RoomTracker.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{

semantic::TraversalGuardValues nominalCrossing()
{
    semantic::TraversalGuardValues crossing;
    crossing.isPassageDetected    = true;
    crossing.isPassable           = true;
    crossing.confidence           = 1.0;
    crossing.areBothSidesObserved = false;
    return crossing;
}

semantic::VerificationVerdict passVerdict(unsigned int inliers = 5U)
{
    semantic::VerificationVerdict verdict;
    verdict.status      = semantic::VerificationStatus::PASS;
    verdict.hasPassed   = true;
    verdict.inlierCount = inliers;
    verdict.inlierRatio = 1.0;
    verdict.confidence  = 1.0;
    return verdict;
}

semantic::VerificationVerdict failVerdict()
{
    semantic::VerificationVerdict verdict;
    verdict.hasPassed   = false;
    verdict.inlierCount = 0U;
    verdict.confidence  = 0.0;
    return verdict;
}

semantic::TrackingStatusInput nominalTracking()
{
    return semantic::TrackingStatusInput();
}

} // namespace

/* ------------------------------------------------------------------------ *
 * Unconditional rows (4): event triggers the target state with no guard.
 * ------------------------------------------------------------------------ */

TEST(RoomTrackerTransitions, UnconditionalTrackingLostFromConfirmedRoom)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker                        tracker;
        const double                                 now = 100.0 + run * 1.0;
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      nextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState, semantic::RoomTrackingState::CONFIRMED_ROOM);
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ(
            (tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                now + run * 0.1,
                                semantic::TraversalGuardValues(),
                                failVerdict(),
                                nextState2)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2, semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);
    }
}

TEST(RoomTrackerTransitions, UnconditionalTrackingLostFromCrossingPassage)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker                        tracker;
        const double                                 now = 200.0 + run;
        vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      trackerNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        semantic::TraversalGuardValues crossing = nominalCrossing();
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        crossing.dwell_s = (*p_trackerConfig).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                      now,
                      crossing,
                      passVerdict(),
                      nextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState, semantic::RoomTrackingState::CROSSING_PASSAGE);
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ(
            (tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                now + run * 0.1,
                                semantic::TraversalGuardValues(),
                                failVerdict(),
                                nextState2)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2, semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);
    }
}

TEST(RoomTrackerTransitions, UnconditionalLostTimeout)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker                        tracker;
        const double                                 now = 300.0 + run;
        vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      trackerNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ(
            (tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                now,
                                semantic::TraversalGuardValues(),
                                failVerdict(),
                                nextState)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState, semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ((tracker.applyEvent(semantic::RoomTrackingEvent::LOST_TIMEOUT,
                                      now + run * 0.1,
                                      semantic::TraversalGuardValues(),
                                      failVerdict(),
                                      nextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2, semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);
    }
}

TEST(RoomTrackerTransitions, UnconditionalReacquireTimeout)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker                        tracker;
        const double                                 now = 400.0 + run;
        vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      trackerNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ(
            (tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                now,
                                semantic::TraversalGuardValues(),
                                failVerdict(),
                                nextState)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState, semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      nextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2,
                  semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        vs_graphs::core::semantic::RoomTrackingState nextState3{};
        ASSERT_EQ(
            (tracker.applyEvent(semantic::RoomTrackingEvent::REACQUIRE_TIMEOUT,
                                now + run * 0.1,
                                semantic::TraversalGuardValues(),
                                failVerdict(),
                                nextState3)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState3, semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);
    }
}

/* ------------------------------------------------------------------------ *
 * Guarded rows (6): guard-satisfied fires, guard-rejected stays in place.
 * ------------------------------------------------------------------------ */

TEST(RoomTrackerTransitions, GuardedFirstRoomConfirmed)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker                        tracker;
        const double                                 now = 10.0 + run;
        /* Guard satisfied: verification verdict PASS. */
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(2U),
                      nextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState, semantic::RoomTrackingState::CONFIRMED_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);

        /* Guard rejected: reset, verification verdict FAIL. */
        semantic::RoomTracker                        second;
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now + run * 0.1,
                      semantic::TraversalGuardValues(),
                      failVerdict(),
                      nextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2, semantic::RoomTrackingState::UNKNOWN);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent2 =
            nullptr;
        ASSERT_EQ((second.getLastEvent(p_lastEvent2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_FALSE((*p_lastEvent2).isAccepted);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent3 =
            nullptr;
        ASSERT_EQ((second.getLastEvent(p_lastEvent3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ((*p_lastEvent3).targetState,
                  semantic::RoomTrackingState::UNKNOWN);
    }
}

TEST(RoomTrackerTransitions, GuardedPassageCrossingDetected)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker                        tracker;
        const double                                 now = 20.0 + run;
        vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      trackerNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);

        /* Guard satisfied: passable crossing with dwell and confidence above
         * the configured thresholds. */
        semantic::TraversalGuardValues crossing = nominalCrossing();
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        crossing.dwell_s = (*p_trackerConfig).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                      now,
                      crossing,
                      passVerdict(),
                      nextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState, semantic::RoomTrackingState::CROSSING_PASSAGE);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);

        /* Guard rejected below dwell threshold. */
        semantic::RoomTracker                        second;
        vs_graphs::core::semantic::RoomTrackingState secondNextState{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      secondNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        semantic::TraversalGuardValues shortDwell = nominalCrossing();
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig2 =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        shortDwell.dwell_s = (*p_trackerConfig2).crossing_dwell_s / 2.0;
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                      now,
                      shortDwell,
                      passVerdict(),
                      nextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2, semantic::RoomTrackingState::CONFIRMED_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent2 =
            nullptr;
        ASSERT_EQ((second.getLastEvent(p_lastEvent2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_FALSE((*p_lastEvent2).isAccepted);

        /* Guard rejected below confidence threshold. */
        semantic::RoomTracker                        third;
        vs_graphs::core::semantic::RoomTrackingState thirdNextState{};
        ASSERT_EQ(
            (third.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                              now,
                              semantic::TraversalGuardValues(),
                              passVerdict(),
                              thirdNextState)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        semantic::TraversalGuardValues lowConfidence = nominalCrossing();
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig3 =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        lowConfidence.dwell_s = (*p_trackerConfig3).crossing_dwell_s;
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig4 =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig4)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        lowConfidence.confidence =
            (*p_trackerConfig4).crossing_confidence - 0.1;
        vs_graphs::core::semantic::RoomTrackingState nextState3{};
        ASSERT_EQ((third.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                      now,
                      lowConfidence,
                      passVerdict(),
                      nextState3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState3, semantic::RoomTrackingState::CONFIRMED_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent3 =
            nullptr;
        ASSERT_EQ((third.getLastEvent(p_lastEvent3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_FALSE((*p_lastEvent3).isAccepted);

        /* Guard rejected: crossing segment not passable. */
        semantic::RoomTracker                        fourth;
        vs_graphs::core::semantic::RoomTrackingState fourthNextState{};
        ASSERT_EQ((fourth.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      fourthNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        semantic::TraversalGuardValues blocked = nominalCrossing();
        blocked.isPassable                     = false;
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig5 =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig5)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        blocked.dwell_s = (*p_trackerConfig5).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState nextState4{};
        ASSERT_EQ((fourth.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                      now,
                      blocked,
                      passVerdict(),
                      nextState4)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState4, semantic::RoomTrackingState::CONFIRMED_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent4 =
            nullptr;
        ASSERT_EQ((fourth.getLastEvent(p_lastEvent4)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_FALSE((*p_lastEvent4).isAccepted);
    }
}

TEST(RoomTrackerTransitions, GuardedPassageTraversalComplete)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker                        tracker;
        const double                                 now = 30.0 + run;
        vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      trackerNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        semantic::TraversalGuardValues crossed = nominalCrossing();
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        crossed.dwell_s = (*p_trackerConfig).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                      now,
                      crossed,
                      passVerdict(),
                      nextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState, semantic::RoomTrackingState::CROSSING_PASSAGE);

        /* Guard satisfied: both sides observed, dwell elapsed, verdict PASS. */
        semantic::TraversalGuardValues complete = nominalCrossing();
        complete.areBothSidesObserved           = true;
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig2 =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        complete.dwell_s = (*p_trackerConfig2).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                      now,
                      complete,
                      passVerdict(),
                      nextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2, semantic::RoomTrackingState::CONFIRMED_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);

        /* Guard rejected: traversal complete verdict FAILS verification. */
        semantic::RoomTracker                        second;
        vs_graphs::core::semantic::RoomTrackingState secondNextState{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      secondNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        const vs_graphs::core::semantic::RoomTrackerConfig *p_secondConfig =
            nullptr;
        ASSERT_EQ((second.getConfig(p_secondConfig)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        crossed.dwell_s = (*p_secondConfig).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState secondNextState2{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                      now,
                      crossed,
                      passVerdict(),
                      secondNextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState nextState3{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                      now,
                      complete,
                      failVerdict(),
                      nextState3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState3, semantic::RoomTrackingState::CROSSING_PASSAGE);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent2 =
            nullptr;
        ASSERT_EQ((second.getLastEvent(p_lastEvent2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_FALSE((*p_lastEvent2).isAccepted);

        /* Guard rejected: neither side observed. */
        semantic::RoomTracker                        third;
        vs_graphs::core::semantic::RoomTrackingState thirdNextState{};
        ASSERT_EQ(
            (third.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                              now,
                              semantic::TraversalGuardValues(),
                              passVerdict(),
                              thirdNextState)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        const vs_graphs::core::semantic::RoomTrackerConfig *p_thirdConfig =
            nullptr;
        ASSERT_EQ((third.getConfig(p_thirdConfig)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        crossed.dwell_s = (*p_thirdConfig).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState thirdNextState2{};
        ASSERT_EQ((third.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                      now,
                      crossed,
                      passVerdict(),
                      thirdNextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        semantic::TraversalGuardValues oneSided = nominalCrossing();
        oneSided.areBothSidesObserved           = false;
        const vs_graphs::core::semantic::RoomTrackerConfig *p_thirdConfig2 =
            nullptr;
        ASSERT_EQ((third.getConfig(p_thirdConfig2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        oneSided.dwell_s = (*p_thirdConfig2).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState nextState4{};
        ASSERT_EQ((third.applyEvent(
                      semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                      now,
                      oneSided,
                      passVerdict(),
                      nextState4)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState4, semantic::RoomTrackingState::CROSSING_PASSAGE);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent3 =
            nullptr;
        ASSERT_EQ((third.getLastEvent(p_lastEvent3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_FALSE((*p_lastEvent3).isAccepted);
    }
}

TEST(RoomTrackerTransitions, GuardedRoomReacquired)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker                        tracker;
        const double                                 now = 40.0 + run;
        vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      trackerNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState trackerNextState2{};
        ASSERT_EQ(
            (tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                now,
                                semantic::TraversalGuardValues(),
                                failVerdict(),
                                trackerNextState2)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ((tracker.applyEvent(semantic::RoomTrackingEvent::LOST_TIMEOUT,
                                      now,
                                      semantic::TraversalGuardValues(),
                                      failVerdict(),
                                      nextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState, semantic::RoomTrackingState::LOST_WITHOUT_ROOM);

        /* Guard satisfied: verification PASS reacquires the room. */
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ(
            (tracker.applyEvent(semantic::RoomTrackingEvent::ROOM_REACQUIRED,
                                now,
                                semantic::TraversalGuardValues(),
                                passVerdict(4U),
                                nextState2)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2, semantic::RoomTrackingState::CONFIRMED_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);

        /* Guard rejected: verification FAIL keeps the lost state. */
        semantic::RoomTracker                        second;
        vs_graphs::core::semantic::RoomTrackingState secondNextState{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      secondNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState secondNextState2{};
        ASSERT_EQ((second.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     failVerdict(),
                                     secondNextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState nextState3{};
        ASSERT_EQ((second.applyEvent(semantic::RoomTrackingEvent::LOST_TIMEOUT,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     failVerdict(),
                                     nextState3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState3, semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
        vs_graphs::core::semantic::RoomTrackingState nextState4{};
        ASSERT_EQ(
            (second.applyEvent(semantic::RoomTrackingEvent::ROOM_REACQUIRED,
                               now + run * 0.1,
                               semantic::TraversalGuardValues(),
                               failVerdict(),
                               nextState4)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState4, semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent2 =
            nullptr;
        ASSERT_EQ((second.getLastEvent(p_lastEvent2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_FALSE((*p_lastEvent2).isAccepted);
    }
}

TEST(RoomTrackerTransitions, GuardedNewMapWithRoomMatch)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker                        tracker;
        const double                                 now = 50.0 + run;
        vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      trackerNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ(
            (tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                now,
                                semantic::TraversalGuardValues(),
                                failVerdict(),
                                nextState)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState, semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);

        /* Guard satisfied: verification PASS on a new-map match. */
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(3U),
                      nextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2,
                  semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);

        /* Guard rejected: verification FAIL stays lost. */
        semantic::RoomTracker                        second;
        vs_graphs::core::semantic::RoomTrackingState secondNextState{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      secondNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState secondNextState2{};
        ASSERT_EQ((second.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     failVerdict(),
                                     secondNextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState nextState3{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                      now + run * 0.1,
                      semantic::TraversalGuardValues(),
                      failVerdict(),
                      nextState3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState3, semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent2 =
            nullptr;
        ASSERT_EQ((second.getLastEvent(p_lastEvent2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_FALSE((*p_lastEvent2).isAccepted);
    }
}

TEST(RoomTrackerTransitions, GuardedVerifiedMatchToLastRoom)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker                        tracker;
        const double                                 now = 60.0 + run;
        vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      trackerNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState trackerNextState2{};
        ASSERT_EQ(
            (tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                now,
                                semantic::TraversalGuardValues(),
                                failVerdict(),
                                trackerNextState2)),
            vs_graphs::core::semantic::RoomTrackerStatus::
                ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      nextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState,
                  semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);

        /* Guard satisfied: full verification gates PASS. */
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ((tracker.applyEvent(
                      semantic::RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(6U),
                      nextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2, semantic::RoomTrackingState::CONFIRMED_ROOM);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);

        /* Guard rejected: verification FAIL stays in reacquire. */
        semantic::RoomTracker                        second;
        vs_graphs::core::semantic::RoomTrackingState secondNextState{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      secondNextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState secondNextState2{};
        ASSERT_EQ((second.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     failVerdict(),
                                     secondNextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState secondNextState3{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                      now,
                      semantic::TraversalGuardValues(),
                      passVerdict(),
                      secondNextState3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        vs_graphs::core::semantic::RoomTrackingState nextState3{};
        ASSERT_EQ((second.applyEvent(
                      semantic::RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
                      now + run * 0.1,
                      semantic::TraversalGuardValues(),
                      failVerdict(),
                      nextState3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState3,
                  semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent2 =
            nullptr;
        ASSERT_EQ((second.getLastEvent(p_lastEvent2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_FALSE((*p_lastEvent2).isAccepted);
    }
}

/* ------------------------------------------------------------------------ *
 * Undefined events: no row for the source state -> rejected, no state change.
 * ------------------------------------------------------------------------ */

TEST(RoomTrackerTransitions, UndefinedEventsRejectedEverywhere)
{
    const std::vector<semantic::RoomTrackingEvent> allEvents = {
        semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
        semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
        semantic::RoomTrackingEvent::TRACKING_LOST,
        semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
        semantic::RoomTrackingEvent::ROOM_REACQUIRED,
        semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
        semantic::RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
        semantic::RoomTrackingEvent::LOST_TIMEOUT,
        semantic::RoomTrackingEvent::REACQUIRE_TIMEOUT,
    };

    const std::vector<
        std::pair<semantic::RoomTrackingState, std::vector<std::string>>>
        definedRows = {
            {semantic::RoomTrackingState::UNKNOWN, {"FIRST_ROOM_CONFIRMED"}},
            {semantic::RoomTrackingState::CONFIRMED_ROOM,
             {"PASSAGE_CROSSING_DETECTED", "TRACKING_LOST"}},
            {semantic::RoomTrackingState::CROSSING_PASSAGE,
             {"PASSAGE_TRAVERSAL_COMPLETE", "TRACKING_LOST"}},
            {semantic::RoomTrackingState::LOST_WITHOUT_ROOM,
             {"ROOM_REACQUIRED"}},
            {semantic::RoomTrackingState::LOST_WITH_LAST_ROOM,
             {"NEW_MAP_WITH_ROOM_MATCH", "LOST_TIMEOUT"}},
            {semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP,
             {"VERIFIED_MATCH_TO_LAST_ROOM", "REACQUIRE_TIMEOUT"}},
        };

    for (const auto &entry : definedRows)
    {
        const semantic::RoomTrackingState sourceState = entry.first;
        for (semantic::RoomTrackingEvent event : allEvents)
        {
            std::string text{};
            ASSERT_EQ((semantic::RoomTracker::eventToString(event, text)),
                      vs_graphs::core::semantic::RoomTrackerStatus::
                          ROOM_TRACKER_STATUS_SUCCESS);
            if (std::find(entry.second.begin(), entry.second.end(), text) !=
                entry.second.end())
            {
                continue; /* Defined for this source state. */
            }

            semantic::RoomTracker tracker;
            const double          now = 700.0;

            if (sourceState != semantic::RoomTrackingState::UNKNOWN)
            {
                /* Drive to the source state along a valid path. */
                vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
                ASSERT_EQ((tracker.applyEvent(
                              semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                              now,
                              semantic::TraversalGuardValues(),
                              passVerdict(),
                              trackerNextState)),
                          vs_graphs::core::semantic::RoomTrackerStatus::
                              ROOM_TRACKER_STATUS_SUCCESS);
                if (sourceState == semantic::RoomTrackingState::CONFIRMED_ROOM)
                {
                    /* Already reached. */
                }
                else if (sourceState ==
                         semantic::RoomTrackingState::CROSSING_PASSAGE)
                {
                    semantic::TraversalGuardValues crossed = nominalCrossing();
                    const vs_graphs::core::semantic::RoomTrackerConfig
                        *p_trackerConfig = nullptr;
                    ASSERT_EQ((tracker.getConfig(p_trackerConfig)),
                              vs_graphs::core::semantic::RoomTrackerStatus::
                                  ROOM_TRACKER_STATUS_SUCCESS);
                    crossed.dwell_s = (*p_trackerConfig).crossing_dwell_s;
                    vs_graphs::core::semantic::RoomTrackingState
                        trackerNextState2{};
                    ASSERT_EQ((tracker.applyEvent(semantic::RoomTrackingEvent::
                                                      PASSAGE_CROSSING_DETECTED,
                                                  now,
                                                  crossed,
                                                  passVerdict(),
                                                  trackerNextState2)),
                              vs_graphs::core::semantic::RoomTrackerStatus::
                                  ROOM_TRACKER_STATUS_SUCCESS);
                }
                else if (sourceState ==
                         semantic::RoomTrackingState::LOST_WITH_LAST_ROOM)
                {
                    vs_graphs::core::semantic::RoomTrackingState
                        trackerNextState3{};
                    ASSERT_EQ((tracker.applyEvent(
                                  semantic::RoomTrackingEvent::TRACKING_LOST,
                                  now,
                                  semantic::TraversalGuardValues(),
                                  failVerdict(),
                                  trackerNextState3)),
                              vs_graphs::core::semantic::RoomTrackerStatus::
                                  ROOM_TRACKER_STATUS_SUCCESS);
                }
                else if (sourceState ==
                         semantic::RoomTrackingState::LOST_WITHOUT_ROOM)
                {
                    vs_graphs::core::semantic::RoomTrackingState
                        trackerNextState4{};
                    ASSERT_EQ((tracker.applyEvent(
                                  semantic::RoomTrackingEvent::TRACKING_LOST,
                                  now,
                                  semantic::TraversalGuardValues(),
                                  failVerdict(),
                                  trackerNextState4)),
                              vs_graphs::core::semantic::RoomTrackerStatus::
                                  ROOM_TRACKER_STATUS_SUCCESS);
                    vs_graphs::core::semantic::RoomTrackingState
                        trackerNextState5{};
                    ASSERT_EQ((tracker.applyEvent(
                                  semantic::RoomTrackingEvent::LOST_TIMEOUT,
                                  now,
                                  semantic::TraversalGuardValues(),
                                  failVerdict(),
                                  trackerNextState5)),
                              vs_graphs::core::semantic::RoomTrackerStatus::
                                  ROOM_TRACKER_STATUS_SUCCESS);
                }
                else if (sourceState ==
                         semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP)
                {
                    vs_graphs::core::semantic::RoomTrackingState
                        trackerNextState6{};
                    ASSERT_EQ((tracker.applyEvent(
                                  semantic::RoomTrackingEvent::TRACKING_LOST,
                                  now,
                                  semantic::TraversalGuardValues(),
                                  failVerdict(),
                                  trackerNextState6)),
                              vs_graphs::core::semantic::RoomTrackerStatus::
                                  ROOM_TRACKER_STATUS_SUCCESS);
                    vs_graphs::core::semantic::RoomTrackingState
                        trackerNextState7{};
                    ASSERT_EQ(
                        (tracker.applyEvent(semantic::RoomTrackingEvent::
                                                NEW_MAP_WITH_ROOM_MATCH,
                                            now,
                                            semantic::TraversalGuardValues(),
                                            passVerdict(),
                                            trackerNextState7)),
                        vs_graphs::core::semantic::RoomTrackerStatus::
                            ROOM_TRACKER_STATUS_SUCCESS);
                }
            }

            vs_graphs::core::semantic::RoomTrackingState state2{};
            ASSERT_EQ((tracker.getState(state2)),
                      vs_graphs::core::semantic::RoomTrackerStatus::
                          ROOM_TRACKER_STATUS_SUCCESS);
            std::string text2{};
            ASSERT_EQ((semantic::RoomTracker::eventToString(event, text2)),
                      vs_graphs::core::semantic::RoomTrackerStatus::
                          ROOM_TRACKER_STATUS_SUCCESS);
            EXPECT_EQ(state2, sourceState)
                << "setup mismatch before injecting " << text2;
            vs_graphs::core::semantic::RoomTrackingState nextState{};
            ASSERT_EQ((tracker.applyEvent(event,
                                          now + 1.0,
                                          nominalCrossing(),
                                          passVerdict(),
                                          nextState)),
                      vs_graphs::core::semantic::RoomTrackerStatus::
                          ROOM_TRACKER_STATUS_SUCCESS);
            std::string text3{};
            ASSERT_EQ((semantic::RoomTracker::eventToString(event, text3)),
                      vs_graphs::core::semantic::RoomTrackerStatus::
                          ROOM_TRACKER_STATUS_SUCCESS);
            std::string text4{};
            ASSERT_EQ(
                (semantic::RoomTracker::stateToString(sourceState, text4)),
                vs_graphs::core::semantic::RoomTrackerStatus::
                    ROOM_TRACKER_STATUS_SUCCESS);
            EXPECT_EQ(nextState, sourceState)
                << "undefined event " << text3 << " must not change state "
                << text4;
            const vs_graphs::core::semantic::TransitionEvent *p_lastEvent =
                nullptr;
            ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                      vs_graphs::core::semantic::RoomTrackerStatus::
                          ROOM_TRACKER_STATUS_SUCCESS);
            EXPECT_FALSE((*p_lastEvent).isAccepted);
            vs_graphs::core::semantic::RoomTrackingState state3{};
            ASSERT_EQ((tracker.getState(state3)),
                      vs_graphs::core::semantic::RoomTrackerStatus::
                          ROOM_TRACKER_STATUS_SUCCESS);
            EXPECT_EQ(state3, sourceState);
        }
    }
}

/* ------------------------------------------------------------------------ *
 * semantic::RoomTracker::step() trajectories (dwell timers, timeouts, retries).
 * ------------------------------------------------------------------------ */

TEST(RoomTrackerStep, DwellCrossingTrajectoryWithJitter)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker tracker;
        const double          t0 = 1000.0 + run * 100.0;

        /* UNKNOWN -> CONFIRMED_ROOM once a room is confirmed. */
        vs_graphs::core::semantic::RoomTrackingState nextState{};
        ASSERT_EQ((tracker.step(t0,
                                semantic::TraversalGuardValues(),
                                passVerdict(),
                                nominalTracking(),
                                nextState)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState, semantic::RoomTrackingState::CONFIRMED_ROOM);

        /* Crossing guard holds; the dwell has not yet elapsed. */
        semantic::TraversalGuardValues crossing = nominalCrossing();
        const double                   t1 = t0 + 1.0; /* < crossing_dwell_s */
        vs_graphs::core::semantic::RoomTrackingState nextState2{};
        ASSERT_EQ((tracker.step(t1,
                                crossing,
                                passVerdict(),
                                nominalTracking(),
                                nextState2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState2, semantic::RoomTrackingState::CONFIRMED_ROOM);

        /* Now the dwell has elapsed: the crossing commits. */
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        const double t2 = t1 + (*p_trackerConfig).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState nextState3{};
        ASSERT_EQ((tracker.step(t2,
                                crossing,
                                passVerdict(),
                                nominalTracking(),
                                nextState3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState3, semantic::RoomTrackingState::CROSSING_PASSAGE);

        /* Traversal completion needs both sides + dwell + verdict PASS. */
        semantic::TraversalGuardValues complete = nominalCrossing();
        complete.areBothSidesObserved           = true;
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig2 =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        const double t3 = t2 + (*p_trackerConfig2).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState nextState4{};
        ASSERT_EQ((tracker.step(t3,
                                complete,
                                passVerdict(),
                                nominalTracking(),
                                nextState4)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState4, semantic::RoomTrackingState::CROSSING_PASSAGE);
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig3 =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig3)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        const double t4 = t3 + (*p_trackerConfig3).crossing_dwell_s;
        vs_graphs::core::semantic::RoomTrackingState nextState5{};
        ASSERT_EQ((tracker.step(t4,
                                complete,
                                passVerdict(),
                                nominalTracking(),
                                nextState5)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_EQ(nextState5, semantic::RoomTrackingState::CONFIRMED_ROOM);

        const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
        ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_TRUE((*p_lastEvent).isAccepted);
        const std::vector<vs_graphs::core::semantic::TransitionEvent>
            *p_eventHistory = nullptr;
        ASSERT_EQ((tracker.getEventHistory(p_eventHistory)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        EXPECT_GE((*p_eventHistory).size(), 3U);
    }
}

TEST(RoomTrackerStep, HysteresisResetsDwellOnGuardFailure)
{
    semantic::RoomTracker                        tracker;
    const double                                 t0 = 0.0;
    vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
    ASSERT_EQ((tracker.step(t0,
                            semantic::TraversalGuardValues(),
                            passVerdict(),
                            nominalTracking(),
                            trackerNextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);

    semantic::TraversalGuardValues               crossing = nominalCrossing();
    const double                                 t1       = t0 + 1.0;
    vs_graphs::core::semantic::RoomTrackingState nextState{};
    ASSERT_EQ(
        (tracker
             .step(t1, crossing, passVerdict(), nominalTracking(), nextState)),
        vs_graphs::core::semantic::RoomTrackerStatus::
            ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState, semantic::RoomTrackingState::CONFIRMED_ROOM);

    /* Guard fails (confidence drops below threshold); dwell must reset. */
    semantic::TraversalGuardValues weak             = crossing;
    weak.confidence                                 = 0.0;
    const double                                 t2 = t1 + 1.0;
    vs_graphs::core::semantic::RoomTrackingState nextState2{};
    ASSERT_EQ(
        (tracker.step(t2, weak, passVerdict(), nominalTracking(), nextState2)),
        vs_graphs::core::semantic::RoomTrackerStatus::
            ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState2, semantic::RoomTrackingState::CONFIRMED_ROOM);

    /* Re-satisfying the guard restarts the full dwell window. */
    const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig =
        nullptr;
    ASSERT_EQ((tracker.getConfig(p_trackerConfig)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    const double t3 = t2 + (*p_trackerConfig).crossing_dwell_s - 0.5;
    vs_graphs::core::semantic::RoomTrackingState nextState3{};
    ASSERT_EQ(
        (tracker
             .step(t3, crossing, passVerdict(), nominalTracking(), nextState3)),
        vs_graphs::core::semantic::RoomTrackerStatus::
            ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState3, semantic::RoomTrackingState::CONFIRMED_ROOM);
    const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig2 =
        nullptr;
    ASSERT_EQ((tracker.getConfig(p_trackerConfig2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    const double t4 = t3 + (*p_trackerConfig2).crossing_dwell_s;
    vs_graphs::core::semantic::RoomTrackingState nextState4{};
    ASSERT_EQ(
        (tracker
             .step(t4, crossing, passVerdict(), nominalTracking(), nextState4)),
        vs_graphs::core::semantic::RoomTrackerStatus::
            ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState4, semantic::RoomTrackingState::CROSSING_PASSAGE);
}

TEST(RoomTrackerStep, InvalidCrossingGuardsFailClosed)
{
    semantic::RoomTracker                        tracker;
    vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
    ASSERT_EQ((tracker.step(10.0,
                            semantic::TraversalGuardValues(),
                            passVerdict(),
                            nominalTracking(),
                            trackerNextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);

    semantic::TraversalGuardValues invalid = nominalCrossing();
    invalid.confidence = std::numeric_limits<double>::quiet_NaN();
    vs_graphs::core::semantic::RoomTrackingState nextState{};
    ASSERT_EQ(
        (tracker
             .step(11.0, invalid, passVerdict(), nominalTracking(), nextState)),
        vs_graphs::core::semantic::RoomTrackerStatus::
            ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState, semantic::RoomTrackingState::CONFIRMED_ROOM);

    invalid            = nominalCrossing();
    invalid.confidence = 1.1;
    vs_graphs::core::semantic::RoomTrackingState nextState2{};
    ASSERT_EQ((tracker.step(12.0,
                            invalid,
                            passVerdict(),
                            nominalTracking(),
                            nextState2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState2, semantic::RoomTrackingState::CONFIRMED_ROOM);
}

TEST(RoomTrackerStep, BothSidesAndDwellAccumulateAcrossCycles)
{
    semantic::RoomTracker                        tracker;
    vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
    ASSERT_EQ((tracker.step(100.0,
                            semantic::TraversalGuardValues(),
                            passVerdict(),
                            nominalTracking(),
                            trackerNextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);

    semantic::TraversalGuardValues               crossing = nominalCrossing();
    vs_graphs::core::semantic::RoomTrackingState nextState{};
    ASSERT_EQ((tracker.step(101.0,
                            crossing,
                            passVerdict(),
                            nominalTracking(),
                            nextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState, semantic::RoomTrackingState::CONFIRMED_ROOM);
    vs_graphs::core::semantic::RoomTrackingState nextState2{};
    ASSERT_EQ((tracker.step(103.0,
                            crossing,
                            passVerdict(),
                            nominalTracking(),
                            nextState2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState2, semantic::RoomTrackingState::CROSSING_PASSAGE);

    semantic::TraversalGuardValues oneSide = nominalCrossing();
    oneSide.areBothSidesObserved           = true;
    vs_graphs::core::semantic::RoomTrackingState nextState3{};
    ASSERT_EQ((tracker.step(104.0,
                            oneSide,
                            passVerdict(),
                            nominalTracking(),
                            nextState3)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState3, semantic::RoomTrackingState::CROSSING_PASSAGE);
    vs_graphs::core::semantic::RoomTrackingState nextState4{};
    ASSERT_EQ((tracker.step(105.0,
                            nominalCrossing(),
                            passVerdict(),
                            nominalTracking(),
                            nextState4)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState4, semantic::RoomTrackingState::CROSSING_PASSAGE);
    vs_graphs::core::semantic::RoomTrackingState nextState5{};
    ASSERT_EQ((tracker.step(106.0,
                            nominalCrossing(),
                            passVerdict(),
                            nominalTracking(),
                            nextState5)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState5, semantic::RoomTrackingState::CONFIRMED_ROOM);
}

TEST(RoomTrackerStep, SteadyDomainInputIgnoresTimestampDiscontinuity)
{
    semantic::RoomTracker                        tracker;
    vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
    ASSERT_EQ((tracker.step(1000.0,
                            semantic::TraversalGuardValues(),
                            passVerdict(),
                            nominalTracking(),
                            trackerNextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);

    semantic::TraversalGuardValues               crossing = nominalCrossing();
    vs_graphs::core::semantic::RoomTrackingState nextState{};
    ASSERT_EQ((tracker.step(1001.0,
                            crossing,
                            passVerdict(),
                            nominalTracking(),
                            nextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState, semantic::RoomTrackingState::CONFIRMED_ROOM);
    /* A SLAM timestamp reset must not make the injected steady-domain clock
     * move backwards or complete the dwell early. */
    vs_graphs::core::semantic::RoomTrackingState nextState2{};
    ASSERT_EQ((tracker.step(2.0,
                            crossing,
                            passVerdict(),
                            nominalTracking(),
                            nextState2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState2, semantic::RoomTrackingState::CONFIRMED_ROOM);
    vs_graphs::core::semantic::RoomTrackingState nextState3{};
    ASSERT_EQ((tracker.step(1003.0,
                            crossing,
                            passVerdict(),
                            nominalTracking(),
                            nextState3)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState3, semantic::RoomTrackingState::CROSSING_PASSAGE);
}

TEST(RoomTrackerStep, TrackingLossIsConsumedOncePerEpisode)
{
    semantic::RoomTracker                        tracker;
    vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
    ASSERT_EQ((tracker.step(1.0,
                            semantic::TraversalGuardValues(),
                            passVerdict(),
                            nominalTracking(),
                            trackerNextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);

    semantic::TrackingStatusInput lost;
    lost.isLost = true;
    vs_graphs::core::semantic::RoomTrackingState trackerNextState2{};
    ASSERT_EQ((tracker.step(2.0,
                            semantic::TraversalGuardValues(),
                            failVerdict(),
                            lost,
                            trackerNextState2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    const std::vector<vs_graphs::core::semantic::TransitionEvent>
        *p_trackerEventHistory = nullptr;
    ASSERT_EQ((tracker.getEventHistory(p_trackerEventHistory)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    const std::size_t eventCountAfterLoss = (*p_trackerEventHistory).size();
    vs_graphs::core::semantic::RoomTrackingState trackerNextState3{};
    ASSERT_EQ((tracker.step(3.0,
                            semantic::TraversalGuardValues(),
                            failVerdict(),
                            lost,
                            trackerNextState3)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    const std::vector<vs_graphs::core::semantic::TransitionEvent>
        *p_eventHistory = nullptr;
    ASSERT_EQ((tracker.getEventHistory(p_eventHistory)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ((*p_eventHistory).size(), eventCountAfterLoss);
}

TEST(RoomTrackerStep, TimeoutDecaysToLostWithoutRoom)
{
    semantic::RoomTracker                        tracker;
    const double                                 t0 = 2000.0;
    vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
    ASSERT_EQ((tracker.step(t0,
                            semantic::TraversalGuardValues(),
                            passVerdict(),
                            nominalTracking(),
                            trackerNextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);

    /* Tracking is lost once: CONFIRMED_ROOM -> LOST_WITH_LAST_ROOM. */
    semantic::TrackingStatusInput lost;
    lost.isLost = true;
    vs_graphs::core::semantic::RoomTrackingState nextState{};
    ASSERT_EQ((tracker.step(t0 + 1.0,
                            semantic::TraversalGuardValues(),
                            failVerdict(),
                            lost,
                            nextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState, semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);

    /* Loss persists; the lost_timeout decays to LOST_WITHOUT_ROOM. */
    const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig =
        nullptr;
    ASSERT_EQ((tracker.getConfig(p_trackerConfig)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    const double tExpire = t0 + 1.0 + (*p_trackerConfig).lost_timeout_s;
    vs_graphs::core::semantic::RoomTrackingState nextState2{};
    ASSERT_EQ((tracker.step(tExpire,
                            semantic::TraversalGuardValues(),
                            failVerdict(),
                            lost,
                            nextState2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState2, semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
    const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
    ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_TRUE((*p_lastEvent).isAccepted);
}

TEST(RoomTrackerStep, UndefinedTrackingLossFromUnknownIsRejected)
{
    semantic::RoomTracker         tracker;
    semantic::TrackingStatusInput lost;
    lost.isLost = true;
    vs_graphs::core::semantic::RoomTrackingState nextState{};
    ASSERT_EQ((tracker.step(0.0,
                            semantic::TraversalGuardValues(),
                            failVerdict(),
                            lost,
                            nextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState, semantic::RoomTrackingState::UNKNOWN);
    const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
    ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_FALSE((*p_lastEvent).isAccepted);
    const vs_graphs::core::semantic::TransitionEvent *p_lastEvent2 = nullptr;
    ASSERT_EQ((tracker.getLastEvent(p_lastEvent2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ((*p_lastEvent2).event,
              semantic::RoomTrackingEvent::TRACKING_LOST);
}

TEST(RoomTrackerStep, UnavailableVerificationCannotConfirmOrMutateState)
{
    semantic::RoomTracker         tracker;
    semantic::VerificationVerdict unavailable;
    unavailable.hasPassed = true;
    semantic::TraversalGuardValues crossing;
    semantic::TrackingStatusInput  tracking;

    vs_graphs::core::semantic::RoomTrackingState nextState{};
    ASSERT_EQ((tracker.step(10.0, crossing, unavailable, tracking, nextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState, semantic::RoomTrackingState::UNKNOWN);
    const std::vector<vs_graphs::core::semantic::TransitionEvent>
        *p_eventHistory = nullptr;
    ASSERT_EQ((tracker.getEventHistory(p_eventHistory)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ((*p_eventHistory).size(), 0U);
}

TEST(RoomTrackerStep, MalformedPassVerdictFailsClosed)
{
    semantic::VerificationVerdict malformed = passVerdict();
    malformed.inlierRatio = std::numeric_limits<double>::infinity();
    bool isPass2{};
    ASSERT_EQ((malformed.isPass(isPass2)),
              semantic::VerificationVerdictStatus::
                  VERIFICATION_VERDICT_STATUS_SUCCESS);
    EXPECT_FALSE(isPass2);

    malformed            = passVerdict();
    malformed.confidence = -0.1;
    bool isPass3{};
    ASSERT_EQ((malformed.isPass(isPass3)),
              semantic::VerificationVerdictStatus::
                  VERIFICATION_VERDICT_STATUS_SUCCESS);
    EXPECT_FALSE(isPass3);

    malformed           = passVerdict();
    malformed.hasPassed = false;
    bool isPass4{};
    ASSERT_EQ((malformed.isPass(isPass4)),
              semantic::VerificationVerdictStatus::
                  VERIFICATION_VERDICT_STATUS_SUCCESS);
    EXPECT_FALSE(isPass4);

    malformed        = passVerdict();
    malformed.status = semantic::VerificationStatus::REJECTED;
    bool isPass5{};
    ASSERT_EQ((malformed.isPass(isPass5)),
              semantic::VerificationVerdictStatus::
                  VERIFICATION_VERDICT_STATUS_SUCCESS);
    EXPECT_FALSE(isPass5);
}

TEST(RoomTrackerStep, ReacquireRetriesThenTimeouts)
{
    semantic::RoomTracker                        tracker;
    const double                                 t0 = 3000.0;
    vs_graphs::core::semantic::RoomTrackingState trackerNextState{};
    ASSERT_EQ((tracker.step(t0,
                            semantic::TraversalGuardValues(),
                            passVerdict(),
                            nominalTracking(),
                            trackerNextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);

    semantic::TrackingStatusInput lost;
    lost.isLost = true;
    vs_graphs::core::semantic::RoomTrackingState nextState{};
    ASSERT_EQ((tracker.step(t0 + 1.0,
                            semantic::TraversalGuardValues(),
                            failVerdict(),
                            lost,
                            nextState)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState, semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);

    /* New map (still lost) with a verified match -> REACQUIRING. */
    semantic::TrackingStatusInput newMap;
    newMap.isLost          = true;
    newMap.isNewMapCreated = true;
    vs_graphs::core::semantic::RoomTrackingState nextState2{};
    ASSERT_EQ((tracker.step(t0 + 2.0,
                            semantic::TraversalGuardValues(),
                            passVerdict(),
                            newMap,
                            nextState2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(nextState2, semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);

    /* Failed verification retries at the configured interval; after the
     * configured max retries the reacquire is retired. */
    const double                                        tRetry = t0 + 2.0;
    double                                              t      = tRetry;
    const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig =
        nullptr;
    ASSERT_EQ((tracker.getConfig(p_trackerConfig)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    const unsigned int expectedRetries =
        (*p_trackerConfig).reacquire_max_retries;
    for (unsigned int attempt = 0U; attempt < expectedRetries + 1U; ++attempt)
    {
        const vs_graphs::core::semantic::RoomTrackerConfig *p_trackerConfig2 =
            nullptr;
        ASSERT_EQ((tracker.getConfig(p_trackerConfig2)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        t += (*p_trackerConfig2).reacquire_retry_interval_s;
        semantic::RoomTrackingState state{};
        ASSERT_EQ((tracker.step(t,
                                semantic::TraversalGuardValues(),
                                failVerdict(),
                                newMap,
                                state)),
                  vs_graphs::core::semantic::RoomTrackerStatus::
                      ROOM_TRACKER_STATUS_SUCCESS);
        if (attempt < expectedRetries)
        {
            EXPECT_EQ(state,
                      semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        }
    }
    vs_graphs::core::semantic::RoomTrackingState state2{};
    ASSERT_EQ((tracker.getState(state2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(state2, semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
    const vs_graphs::core::semantic::TransitionEvent *p_lastEvent = nullptr;
    ASSERT_EQ((tracker.getLastEvent(p_lastEvent)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_TRUE((*p_lastEvent).isAccepted);
    const vs_graphs::core::semantic::TransitionEvent *p_lastEvent2 = nullptr;
    ASSERT_EQ((tracker.getLastEvent(p_lastEvent2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ((*p_lastEvent2).event,
              semantic::RoomTrackingEvent::REACQUIRE_TIMEOUT);
}

} // namespace core
} // namespace vs_graphs
