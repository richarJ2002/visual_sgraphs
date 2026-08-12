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
 *
 * WP13 Phase 1 confirmed: RoomTracker implements the Section 18.2 table of
 * exactly 10 transition rows (6 guarded, 4 unconditional). This suite covers:
 *   (a) each guarded row with a guard-satisfied and a guard-rejected case;
 *   (b) each unconditional row as an event-triggered target-state test;
 *   (c) every source state rejecting events with no defined row.
 * Each trajectory is replayed several times with jittered timestamps.
 */

#include "Semantic/RoomTracker.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace ORB_SLAM3
{
namespace
{

TraversalGuardValues nominalCrossing()
{
    TraversalGuardValues crossing;
    crossing.passageDetected = true;
    crossing.passable        = true;
    crossing.confidence      = 1.0;
    crossing.bothSidesObserved = false;
    return crossing;
}

VerificationVerdict passVerdict(unsigned int inliers = 5U)
{
    VerificationVerdict verdict;
    verdict.pass        = true;
    verdict.inlierCount = inliers;
    verdict.inlierRatio = 1.0;
    verdict.confidence  = 1.0;
    return verdict;
}

VerificationVerdict failVerdict()
{
    VerificationVerdict verdict;
    verdict.pass    = false;
    verdict.inlierCount = 0U;
    verdict.confidence = 0.0;
    return verdict;
}

TrackingStatusInput nominalTracking()
{
    return TrackingStatusInput();
}

} // namespace

/* ------------------------------------------------------------------------ *
 * Unconditional rows (4): event triggers the target state with no guard.
 * ------------------------------------------------------------------------ */

TEST(RoomTrackerTransitions, UnconditionalTrackingLostFromConfirmedRoom)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double now = 100.0 + run * 1.0;
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                                     now,
                                     TraversalGuardValues(),
                                     passVerdict()),
                  RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                                     now + run * 0.1,
                                     TraversalGuardValues(),
                                     failVerdict()),
                  RoomTrackingState::LOST_WITH_LAST_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, UnconditionalTrackingLostFromCrossingPassage)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double now = 200.0 + run;
        tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           TraversalGuardValues(),
                           passVerdict());
        TraversalGuardValues crossing = nominalCrossing();
        crossing.dwell_s              = tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                                     now,
                                     crossing,
                                     passVerdict()),
                  RoomTrackingState::CROSSING_PASSAGE);
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                                     now + run * 0.1,
                                     TraversalGuardValues(),
                                     failVerdict()),
                  RoomTrackingState::LOST_WITH_LAST_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, UnconditionalLostTimeout)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double now = 300.0 + run;
        tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           TraversalGuardValues(),
                           passVerdict());
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                                     now,
                                     TraversalGuardValues(),
                                     failVerdict()),
                  RoomTrackingState::LOST_WITH_LAST_ROOM);
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::LOST_TIMEOUT,
                                     now + run * 0.1,
                                     TraversalGuardValues(),
                                     failVerdict()),
                  RoomTrackingState::LOST_WITHOUT_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, UnconditionalReacquireTimeout)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double now = 400.0 + run;
        tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           TraversalGuardValues(),
                           passVerdict());
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                                     now,
                                     TraversalGuardValues(),
                                     failVerdict()),
                  RoomTrackingState::LOST_WITH_LAST_ROOM);
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                                     now,
                                     TraversalGuardValues(),
                                     passVerdict()),
                  RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::REACQUIRE_TIMEOUT,
                                     now + run * 0.1,
                                     TraversalGuardValues(),
                                     failVerdict()),
                  RoomTrackingState::LOST_WITHOUT_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);
    }
}

/* ------------------------------------------------------------------------ *
 * Guarded rows (6): guard-satisfied fires, guard-rejected stays in place.
 * ------------------------------------------------------------------------ */

TEST(RoomTrackerTransitions, GuardedFirstRoomConfirmed)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double now = 10.0 + run;
        /* Guard satisfied: verification verdict PASS. */
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                                     now,
                                     TraversalGuardValues(),
                                     passVerdict(2U)),
                  RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected: reset, verification verdict FAIL. */
        RoomTracker second;
        EXPECT_EQ(second.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                                    now + run * 0.1,
                                    TraversalGuardValues(),
                                    failVerdict()),
                  RoomTrackingState::UNKNOWN);
        EXPECT_FALSE(second.getLastEvent().accepted);
        EXPECT_EQ(second.getLastEvent().targetState,
                  RoomTrackingState::UNKNOWN);
    }
}

TEST(RoomTrackerTransitions, GuardedPassageCrossingDetected)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double now = 20.0 + run;
        tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           TraversalGuardValues(),
                           passVerdict());

        /* Guard satisfied: passable crossing with dwell and confidence above
         * the configured thresholds. */
        TraversalGuardValues crossing = nominalCrossing();
        crossing.dwell_s = tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                                     now,
                                     crossing,
                                     passVerdict()),
                  RoomTrackingState::CROSSING_PASSAGE);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected below dwell threshold. */
        RoomTracker second;
        second.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          TraversalGuardValues(),
                          passVerdict());
        TraversalGuardValues shortDwell = nominalCrossing();
        shortDwell.dwell_s = tracker.getConfig().crossing_dwell_s / 2.0;
        EXPECT_EQ(second.applyEvent(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                                    now,
                                    shortDwell,
                                    passVerdict()),
                  RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_FALSE(second.getLastEvent().accepted);

        /* Guard rejected below confidence threshold. */
        RoomTracker third;
        third.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                         now,
                         TraversalGuardValues(),
                         passVerdict());
        TraversalGuardValues lowConfidence = nominalCrossing();
        lowConfidence.dwell_s = tracker.getConfig().crossing_dwell_s;
        lowConfidence.confidence =
            tracker.getConfig().crossing_confidence - 0.1;
        EXPECT_EQ(third.applyEvent(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                                   now,
                                   lowConfidence,
                                   passVerdict()),
                  RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_FALSE(third.getLastEvent().accepted);

        /* Guard rejected: crossing segment not passable. */
        RoomTracker fourth;
        fourth.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          TraversalGuardValues(),
                          passVerdict());
        TraversalGuardValues blocked = nominalCrossing();
        blocked.passable = false;
        blocked.dwell_s  = tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(fourth.applyEvent(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                                    now,
                                    blocked,
                                    passVerdict()),
                  RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_FALSE(fourth.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, GuardedPassageTraversalComplete)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double now = 30.0 + run;
        tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           TraversalGuardValues(),
                           passVerdict());
        TraversalGuardValues crossed = nominalCrossing();
        crossed.dwell_s = tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                                     now,
                                     crossed,
                                     passVerdict()),
                  RoomTrackingState::CROSSING_PASSAGE);

        /* Guard satisfied: both sides observed, dwell elapsed, verdict PASS. */
        TraversalGuardValues complete = nominalCrossing();
        complete.bothSidesObserved = true;
        complete.dwell_s = tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                                     now,
                                     complete,
                                     passVerdict()),
                  RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected: traversal complete verdict FAILS verification. */
        RoomTracker second;
        second.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          TraversalGuardValues(),
                          passVerdict());
        crossed.dwell_s = second.getConfig().crossing_dwell_s;
        second.applyEvent(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                          now,
                          crossed,
                          passVerdict());
        EXPECT_EQ(second.applyEvent(RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                                    now,
                                    complete,
                                    failVerdict()),
                  RoomTrackingState::CROSSING_PASSAGE);
        EXPECT_FALSE(second.getLastEvent().accepted);

        /* Guard rejected: neither side observed. */
        RoomTracker third;
        third.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                         now,
                         TraversalGuardValues(),
                         passVerdict());
        crossed.dwell_s = third.getConfig().crossing_dwell_s;
        third.applyEvent(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                         now,
                         crossed,
                         passVerdict());
        TraversalGuardValues oneSided = nominalCrossing();
        oneSided.bothSidesObserved = false;
        oneSided.dwell_s = third.getConfig().crossing_dwell_s;
        EXPECT_EQ(third.applyEvent(RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                                   now,
                                   oneSided,
                                   passVerdict()),
                  RoomTrackingState::CROSSING_PASSAGE);
        EXPECT_FALSE(third.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, GuardedRoomReacquired)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double now = 40.0 + run;
        tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           TraversalGuardValues(),
                           passVerdict());
        tracker.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                           now,
                           TraversalGuardValues(),
                           failVerdict());
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::LOST_TIMEOUT,
                                     now,
                                     TraversalGuardValues(),
                                     failVerdict()),
                  RoomTrackingState::LOST_WITHOUT_ROOM);

        /* Guard satisfied: verification PASS reacquires the room. */
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::ROOM_REACQUIRED,
                                     now,
                                     TraversalGuardValues(),
                                     passVerdict(4U)),
                  RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected: verification FAIL keeps the lost state. */
        RoomTracker second;
        second.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          TraversalGuardValues(),
                          passVerdict());
        second.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                          now,
                          TraversalGuardValues(),
                          failVerdict());
        EXPECT_EQ(second.applyEvent(RoomTrackingEvent::LOST_TIMEOUT,
                                    now,
                                    TraversalGuardValues(),
                                    failVerdict()),
                  RoomTrackingState::LOST_WITHOUT_ROOM);
        EXPECT_EQ(second.applyEvent(RoomTrackingEvent::ROOM_REACQUIRED,
                                    now + run * 0.1,
                                    TraversalGuardValues(),
                                    failVerdict()),
                  RoomTrackingState::LOST_WITHOUT_ROOM);
        EXPECT_FALSE(second.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, GuardedNewMapWithRoomMatch)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double now = 50.0 + run;
        tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           TraversalGuardValues(),
                           passVerdict());
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                                     now,
                                     TraversalGuardValues(),
                                     failVerdict()),
                  RoomTrackingState::LOST_WITH_LAST_ROOM);

        /* Guard satisfied: verification PASS on a new-map match. */
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                                     now,
                                     TraversalGuardValues(),
                                     passVerdict(3U)),
                  RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected: verification FAIL stays lost. */
        RoomTracker second;
        second.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          TraversalGuardValues(),
                          passVerdict());
        second.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                          now,
                          TraversalGuardValues(),
                          failVerdict());
        EXPECT_EQ(second.applyEvent(RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                                    now + run * 0.1,
                                    TraversalGuardValues(),
                                    failVerdict()),
                  RoomTrackingState::LOST_WITH_LAST_ROOM);
        EXPECT_FALSE(second.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, GuardedVerifiedMatchToLastRoom)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double now = 60.0 + run;
        tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           TraversalGuardValues(),
                           passVerdict());
        tracker.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                           now,
                           TraversalGuardValues(),
                           failVerdict());
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                                     now,
                                     TraversalGuardValues(),
                                     passVerdict()),
                  RoomTrackingState::REACQUIRING_IN_NEW_MAP);

        /* Guard satisfied: full verification gates PASS. */
        EXPECT_EQ(tracker.applyEvent(RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
                                     now,
                                     TraversalGuardValues(),
                                     passVerdict(6U)),
                  RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected: verification FAIL stays in reacquire. */
        RoomTracker second;
        second.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          TraversalGuardValues(),
                          passVerdict());
        second.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                          now,
                          TraversalGuardValues(),
                          failVerdict());
        second.applyEvent(RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                          now,
                          TraversalGuardValues(),
                          passVerdict());
        EXPECT_EQ(second.applyEvent(RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
                                    now + run * 0.1,
                                    TraversalGuardValues(),
                                    failVerdict()),
                  RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        EXPECT_FALSE(second.getLastEvent().accepted);
    }
}

/* ------------------------------------------------------------------------ *
 * Undefined events: no row for the source state -> rejected, no state change.
 * ------------------------------------------------------------------------ */

TEST(RoomTrackerTransitions, UndefinedEventsRejectedEverywhere)
{
    const std::vector<RoomTrackingEvent> allEvents = {
        RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
        RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
        RoomTrackingEvent::TRACKING_LOST,
        RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
        RoomTrackingEvent::ROOM_REACQUIRED,
        RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
        RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
        RoomTrackingEvent::LOST_TIMEOUT,
        RoomTrackingEvent::REACQUIRE_TIMEOUT,
    };

    const std::vector<std::pair<RoomTrackingState,
                                std::vector<std::string>>>
        definedRows = {
            {RoomTrackingState::UNKNOWN,
             {"FIRST_ROOM_CONFIRMED"}},
            {RoomTrackingState::CONFIRMED_ROOM,
             {"PASSAGE_CROSSING_DETECTED", "TRACKING_LOST"}},
            {RoomTrackingState::CROSSING_PASSAGE,
             {"PASSAGE_TRAVERSAL_COMPLETE", "TRACKING_LOST"}},
            {RoomTrackingState::LOST_WITHOUT_ROOM, {"ROOM_REACQUIRED"}},
            {RoomTrackingState::LOST_WITH_LAST_ROOM,
             {"NEW_MAP_WITH_ROOM_MATCH", "LOST_TIMEOUT"}},
            {RoomTrackingState::REACQUIRING_IN_NEW_MAP,
             {"VERIFIED_MATCH_TO_LAST_ROOM", "REACQUIRE_TIMEOUT"}},
        };

    for (const auto &entry : definedRows)
    {
        const RoomTrackingState sourceState = entry.first;
        for (RoomTrackingEvent event : allEvents)
        {
            if (std::find(entry.second.begin(), entry.second.end(),
                          RoomTracker::eventToString(event)) !=
                entry.second.end())
            {
                continue; /* Defined for this source state. */
            }

            RoomTracker tracker;
            const double now = 700.0;

            if (sourceState != RoomTrackingState::UNKNOWN)
            {
                /* Drive to the source state along a valid path. */
                tracker.applyEvent(RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                                   now,
                                   TraversalGuardValues(),
                                   passVerdict());
                if (sourceState == RoomTrackingState::CONFIRMED_ROOM)
                {
                    /* Already reached. */
                }
                else if (sourceState ==
                         RoomTrackingState::CROSSING_PASSAGE)
                {
                    TraversalGuardValues crossed = nominalCrossing();
                    crossed.dwell_s =
                        tracker.getConfig().crossing_dwell_s;
                    tracker.applyEvent(
                        RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                        now,
                        crossed,
                        passVerdict());
                }
                else if (sourceState ==
                         RoomTrackingState::LOST_WITH_LAST_ROOM)
                {
                    tracker.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                                       now,
                                       TraversalGuardValues(),
                                       failVerdict());
                }
                else if (sourceState ==
                         RoomTrackingState::LOST_WITHOUT_ROOM)
                {
                    tracker.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                                       now,
                                       TraversalGuardValues(),
                                       failVerdict());
                    tracker.applyEvent(RoomTrackingEvent::LOST_TIMEOUT,
                                       now,
                                       TraversalGuardValues(),
                                       failVerdict());
                }
                else if (sourceState ==
                         RoomTrackingState::REACQUIRING_IN_NEW_MAP)
                {
                    tracker.applyEvent(RoomTrackingEvent::TRACKING_LOST,
                                       now,
                                       TraversalGuardValues(),
                                       failVerdict());
                    tracker.applyEvent(RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                                       now,
                                       TraversalGuardValues(),
                                       passVerdict());
                }
            }

            EXPECT_EQ(tracker.getState(), sourceState)
                << "setup mismatch before injecting " << RoomTracker::eventToString(event);
            EXPECT_EQ(tracker.applyEvent(event,
                                         now + 1.0,
                                         nominalCrossing(),
                                         passVerdict()),
                      sourceState)
                << "undefined event " << RoomTracker::eventToString(event)
                << " must not change state " << RoomTracker::stateToString(sourceState);
            EXPECT_FALSE(tracker.getLastEvent().accepted);
            EXPECT_EQ(tracker.getState(), sourceState);
        }
    }
}

/* ------------------------------------------------------------------------ *
 * RoomTracker::step() trajectories (dwell timers, timeouts, retries).
 * ------------------------------------------------------------------------ */

TEST(RoomTrackerStep, DwellCrossingTrajectoryWithJitter)
{
    for (int run = 0; run < 5; ++run)
    {
        RoomTracker tracker;
        const double t0 = 1000.0 + run * 100.0;

        /* UNKNOWN -> CONFIRMED_ROOM once a room is confirmed. */
        EXPECT_EQ(tracker.step(t0,
                               TraversalGuardValues(),
                               passVerdict(),
                               nominalTracking()),
                  RoomTrackingState::CONFIRMED_ROOM);

        /* Crossing guard holds; the dwell has not yet elapsed. */
        TraversalGuardValues crossing = nominalCrossing();
        const double t1 = t0 + 1.0; /* < crossing_dwell_s */
        EXPECT_EQ(tracker.step(t1, crossing, passVerdict(), nominalTracking()),
                  RoomTrackingState::CONFIRMED_ROOM);

        /* Now the dwell has elapsed: the crossing commits. */
        const double t2 = t1 + tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(tracker.step(t2, crossing, passVerdict(), nominalTracking()),
                  RoomTrackingState::CROSSING_PASSAGE);

        /* Traversal completion needs both sides + dwell + verdict PASS. */
        TraversalGuardValues complete = nominalCrossing();
        complete.bothSidesObserved = true;
        const double t3 = t2 + tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(tracker.step(t3, complete, passVerdict(), nominalTracking()),
                  RoomTrackingState::CROSSING_PASSAGE);
        const double t4 = t3 + tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(tracker.step(t4, complete, passVerdict(), nominalTracking()),
                  RoomTrackingState::CONFIRMED_ROOM);

        EXPECT_TRUE(tracker.getLastEvent().accepted);
        EXPECT_GE(tracker.getEventHistory().size(), 3U);
    }
}

TEST(RoomTrackerStep, HysteresisResetsDwellOnGuardFailure)
{
    RoomTracker tracker;
    const double t0 = 0.0;
    tracker.step(t0, TraversalGuardValues(), passVerdict(), nominalTracking());

    TraversalGuardValues crossing = nominalCrossing();
    const double t1 = t0 + 1.0;
    EXPECT_EQ(tracker.step(t1, crossing, passVerdict(), nominalTracking()),
              RoomTrackingState::CONFIRMED_ROOM);

    /* Guard fails (confidence drops below threshold); dwell must reset. */
    TraversalGuardValues weak = crossing;
    weak.confidence = 0.0;
    const double t2 = t1 + 1.0;
    EXPECT_EQ(tracker.step(t2, weak, passVerdict(), nominalTracking()),
              RoomTrackingState::CONFIRMED_ROOM);

    /* Re-satisfying the guard restarts the full dwell window. */
    const double t3 = t2 + tracker.getConfig().crossing_dwell_s - 0.5;
    EXPECT_EQ(tracker.step(t3, crossing, passVerdict(), nominalTracking()),
              RoomTrackingState::CONFIRMED_ROOM);
    const double t4 = t3 + tracker.getConfig().crossing_dwell_s;
    EXPECT_EQ(tracker.step(t4, crossing, passVerdict(), nominalTracking()),
              RoomTrackingState::CROSSING_PASSAGE);
}

TEST(RoomTrackerStep, TimeoutDecaysToLostWithoutRoom)
{
    RoomTracker tracker;
    const double t0 = 2000.0;
    tracker.step(t0, TraversalGuardValues(), passVerdict(), nominalTracking());

    /* Tracking is lost once: CONFIRMED_ROOM -> LOST_WITH_LAST_ROOM. */
    TrackingStatusInput lost;
    lost.lost = true;
    EXPECT_EQ(tracker.step(t0 + 1.0,
                           TraversalGuardValues(),
                           failVerdict(),
                           lost),
              RoomTrackingState::LOST_WITH_LAST_ROOM);

    /* Loss persists; the lost_timeout decays to LOST_WITHOUT_ROOM. */
    const double tExpire = t0 + 1.0 + tracker.getConfig().lost_timeout_s;
    EXPECT_EQ(tracker.step(tExpire,
                           TraversalGuardValues(),
                           failVerdict(),
                           lost),
              RoomTrackingState::LOST_WITHOUT_ROOM);
    EXPECT_TRUE(tracker.getLastEvent().accepted);
}

TEST(RoomTrackerStep, UndefinedTrackingLossFromUnknownIsRejected)
{
    RoomTracker tracker;
    TrackingStatusInput lost;
    lost.lost = true;
    EXPECT_EQ(tracker.step(0.0, TraversalGuardValues(), failVerdict(), lost),
              RoomTrackingState::UNKNOWN);
    EXPECT_FALSE(tracker.getLastEvent().accepted);
    EXPECT_EQ(tracker.getLastEvent().event, RoomTrackingEvent::TRACKING_LOST);
}

TEST(RoomTrackerStep, ReacquireRetriesThenTimeouts)
{
    RoomTracker tracker;
    const double t0 = 3000.0;
    tracker.step(t0, TraversalGuardValues(), passVerdict(), nominalTracking());

    TrackingStatusInput lost;
    lost.lost = true;
    EXPECT_EQ(tracker.step(t0 + 1.0,
                           TraversalGuardValues(),
                           failVerdict(),
                           lost),
              RoomTrackingState::LOST_WITH_LAST_ROOM);

    /* New map (still lost) with a verified match -> REACQUIRING. */
    TrackingStatusInput newMap;
    newMap.lost = true;
    newMap.newMapCreated = true;
    EXPECT_EQ(tracker.step(t0 + 2.0,
                           TraversalGuardValues(),
                           passVerdict(),
                           newMap),
              RoomTrackingState::REACQUIRING_IN_NEW_MAP);

    /* Failed verification retries at the configured interval; after the
     * configured max retries the reacquire is retired. */
    const double tRetry = t0 + 2.0;
    double t = tRetry;
    const unsigned int expectedRetries =
        tracker.getConfig().reacquire_max_retries;
    for (unsigned int attempt = 0U; attempt < expectedRetries + 1U; ++attempt)
    {
        t += tracker.getConfig().reacquire_retry_interval_s;
        const RoomTrackingState state =
            tracker.step(t, TraversalGuardValues(), failVerdict(), newMap);
        if (attempt < expectedRetries)
        {
            EXPECT_EQ(state, RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        }
    }
    EXPECT_EQ(tracker.getState(), RoomTrackingState::LOST_WITHOUT_ROOM);
    EXPECT_TRUE(tracker.getLastEvent().accepted);
    EXPECT_EQ(tracker.getLastEvent().event,
              RoomTrackingEvent::REACQUIRE_TIMEOUT);
}

} // namespace ORB_SLAM3