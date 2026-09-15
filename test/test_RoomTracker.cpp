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
 * WP13 Phase 1 confirmed: semantic::RoomTracker implements the Section 18.2 table of
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
    crossing.passageDetected   = true;
    crossing.passable          = true;
    crossing.confidence        = 1.0;
    crossing.bothSidesObserved = false;
    return crossing;
}

semantic::VerificationVerdict passVerdict(unsigned int inliers = 5U)
{
    semantic::VerificationVerdict verdict;
    verdict.status      = semantic::VerificationStatus::PASS;
    verdict.pass        = true;
    verdict.inlierCount = inliers;
    verdict.inlierRatio = 1.0;
    verdict.confidence  = 1.0;
    return verdict;
}

semantic::VerificationVerdict failVerdict()
{
    semantic::VerificationVerdict verdict;
    verdict.pass        = false;
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
        semantic::RoomTracker  tracker;
        const double now = 100.0 + run * 1.0;
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     passVerdict()),
                  semantic::RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                     now + run * 0.1,
                                     semantic::TraversalGuardValues(),
                                     failVerdict()),
                  semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, UnconditionalTrackingLostFromCrossingPassage)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker  tracker;
        const double now = 200.0 + run;
        tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           semantic::TraversalGuardValues(),
                           passVerdict());
        semantic::TraversalGuardValues crossing = nominalCrossing();
        crossing.dwell_s              = tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(
            tracker.applyEvent(semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                               now,
                               crossing,
                               passVerdict()),
            semantic::RoomTrackingState::CROSSING_PASSAGE);
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                     now + run * 0.1,
                                     semantic::TraversalGuardValues(),
                                     failVerdict()),
                  semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, UnconditionalLostTimeout)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker  tracker;
        const double now = 300.0 + run;
        tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           semantic::TraversalGuardValues(),
                           passVerdict());
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     failVerdict()),
                  semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::LOST_TIMEOUT,
                                     now + run * 0.1,
                                     semantic::TraversalGuardValues(),
                                     failVerdict()),
                  semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, UnconditionalReacquireTimeout)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker  tracker;
        const double now = 400.0 + run;
        tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           semantic::TraversalGuardValues(),
                           passVerdict());
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     failVerdict()),
                  semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     passVerdict()),
                  semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::REACQUIRE_TIMEOUT,
                                     now + run * 0.1,
                                     semantic::TraversalGuardValues(),
                                     failVerdict()),
                  semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
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
        semantic::RoomTracker  tracker;
        const double now = 10.0 + run;
        /* Guard satisfied: verification verdict PASS. */
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     passVerdict(2U)),
                  semantic::RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected: reset, verification verdict FAIL. */
        semantic::RoomTracker second;
        EXPECT_EQ(second.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                                    now + run * 0.1,
                                    semantic::TraversalGuardValues(),
                                    failVerdict()),
                  semantic::RoomTrackingState::UNKNOWN);
        EXPECT_FALSE(second.getLastEvent().accepted);
        EXPECT_EQ(second.getLastEvent().targetState,
                  semantic::RoomTrackingState::UNKNOWN);
    }
}

TEST(RoomTrackerTransitions, GuardedPassageCrossingDetected)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker  tracker;
        const double now = 20.0 + run;
        tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           semantic::TraversalGuardValues(),
                           passVerdict());

        /* Guard satisfied: passable crossing with dwell and confidence above
         * the configured thresholds. */
        semantic::TraversalGuardValues crossing = nominalCrossing();
        crossing.dwell_s              = tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(
            tracker.applyEvent(semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                               now,
                               crossing,
                               passVerdict()),
            semantic::RoomTrackingState::CROSSING_PASSAGE);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected below dwell threshold. */
        semantic::RoomTracker second;
        second.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          semantic::TraversalGuardValues(),
                          passVerdict());
        semantic::TraversalGuardValues shortDwell = nominalCrossing();
        shortDwell.dwell_s = tracker.getConfig().crossing_dwell_s / 2.0;
        EXPECT_EQ(
            second.applyEvent(semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                              now,
                              shortDwell,
                              passVerdict()),
            semantic::RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_FALSE(second.getLastEvent().accepted);

        /* Guard rejected below confidence threshold. */
        semantic::RoomTracker third;
        third.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                         now,
                         semantic::TraversalGuardValues(),
                         passVerdict());
        semantic::TraversalGuardValues lowConfidence = nominalCrossing();
        lowConfidence.dwell_s = tracker.getConfig().crossing_dwell_s;
        lowConfidence.confidence =
            tracker.getConfig().crossing_confidence - 0.1;
        EXPECT_EQ(third.applyEvent(semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                                   now,
                                   lowConfidence,
                                   passVerdict()),
                  semantic::RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_FALSE(third.getLastEvent().accepted);

        /* Guard rejected: crossing segment not passable. */
        semantic::RoomTracker fourth;
        fourth.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          semantic::TraversalGuardValues(),
                          passVerdict());
        semantic::TraversalGuardValues blocked = nominalCrossing();
        blocked.passable             = false;
        blocked.dwell_s              = tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(
            fourth.applyEvent(semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                              now,
                              blocked,
                              passVerdict()),
            semantic::RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_FALSE(fourth.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, GuardedPassageTraversalComplete)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker  tracker;
        const double now = 30.0 + run;
        tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           semantic::TraversalGuardValues(),
                           passVerdict());
        semantic::TraversalGuardValues crossed = nominalCrossing();
        crossed.dwell_s              = tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(
            tracker.applyEvent(semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                               now,
                               crossed,
                               passVerdict()),
            semantic::RoomTrackingState::CROSSING_PASSAGE);

        /* Guard satisfied: both sides observed, dwell elapsed, verdict PASS. */
        semantic::TraversalGuardValues complete = nominalCrossing();
        complete.bothSidesObserved    = true;
        complete.dwell_s              = tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(
            tracker.applyEvent(semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                               now,
                               complete,
                               passVerdict()),
            semantic::RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected: traversal complete verdict FAILS verification. */
        semantic::RoomTracker second;
        second.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          semantic::TraversalGuardValues(),
                          passVerdict());
        crossed.dwell_s = second.getConfig().crossing_dwell_s;
        second.applyEvent(semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                          now,
                          crossed,
                          passVerdict());
        EXPECT_EQ(
            second.applyEvent(semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                              now,
                              complete,
                              failVerdict()),
            semantic::RoomTrackingState::CROSSING_PASSAGE);
        EXPECT_FALSE(second.getLastEvent().accepted);

        /* Guard rejected: neither side observed. */
        semantic::RoomTracker third;
        third.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                         now,
                         semantic::TraversalGuardValues(),
                         passVerdict());
        crossed.dwell_s = third.getConfig().crossing_dwell_s;
        third.applyEvent(semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                         now,
                         crossed,
                         passVerdict());
        semantic::TraversalGuardValues oneSided = nominalCrossing();
        oneSided.bothSidesObserved    = false;
        oneSided.dwell_s              = third.getConfig().crossing_dwell_s;
        EXPECT_EQ(
            third.applyEvent(semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                             now,
                             oneSided,
                             passVerdict()),
            semantic::RoomTrackingState::CROSSING_PASSAGE);
        EXPECT_FALSE(third.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, GuardedRoomReacquired)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker  tracker;
        const double now = 40.0 + run;
        tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           semantic::TraversalGuardValues(),
                           passVerdict());
        tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                           now,
                           semantic::TraversalGuardValues(),
                           failVerdict());
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::LOST_TIMEOUT,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     failVerdict()),
                  semantic::RoomTrackingState::LOST_WITHOUT_ROOM);

        /* Guard satisfied: verification PASS reacquires the room. */
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::ROOM_REACQUIRED,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     passVerdict(4U)),
                  semantic::RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected: verification FAIL keeps the lost state. */
        semantic::RoomTracker second;
        second.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          semantic::TraversalGuardValues(),
                          passVerdict());
        second.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                          now,
                          semantic::TraversalGuardValues(),
                          failVerdict());
        EXPECT_EQ(second.applyEvent(semantic::RoomTrackingEvent::LOST_TIMEOUT,
                                    now,
                                    semantic::TraversalGuardValues(),
                                    failVerdict()),
                  semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
        EXPECT_EQ(second.applyEvent(semantic::RoomTrackingEvent::ROOM_REACQUIRED,
                                    now + run * 0.1,
                                    semantic::TraversalGuardValues(),
                                    failVerdict()),
                  semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
        EXPECT_FALSE(second.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, GuardedNewMapWithRoomMatch)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker  tracker;
        const double now = 50.0 + run;
        tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           semantic::TraversalGuardValues(),
                           passVerdict());
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     failVerdict()),
                  semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);

        /* Guard satisfied: verification PASS on a new-map match. */
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     passVerdict(3U)),
                  semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected: verification FAIL stays lost. */
        semantic::RoomTracker second;
        second.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          semantic::TraversalGuardValues(),
                          passVerdict());
        second.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                          now,
                          semantic::TraversalGuardValues(),
                          failVerdict());
        EXPECT_EQ(second.applyEvent(semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                                    now + run * 0.1,
                                    semantic::TraversalGuardValues(),
                                    failVerdict()),
                  semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);
        EXPECT_FALSE(second.getLastEvent().accepted);
    }
}

TEST(RoomTrackerTransitions, GuardedVerifiedMatchToLastRoom)
{
    for (int run = 0; run < 5; ++run)
    {
        semantic::RoomTracker  tracker;
        const double now = 60.0 + run;
        tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                           now,
                           semantic::TraversalGuardValues(),
                           passVerdict());
        tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                           now,
                           semantic::TraversalGuardValues(),
                           failVerdict());
        EXPECT_EQ(tracker.applyEvent(semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                                     now,
                                     semantic::TraversalGuardValues(),
                                     passVerdict()),
                  semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);

        /* Guard satisfied: full verification gates PASS. */
        EXPECT_EQ(
            tracker.applyEvent(semantic::RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
                               now,
                               semantic::TraversalGuardValues(),
                               passVerdict(6U)),
            semantic::RoomTrackingState::CONFIRMED_ROOM);
        EXPECT_TRUE(tracker.getLastEvent().accepted);

        /* Guard rejected: verification FAIL stays in reacquire. */
        semantic::RoomTracker second;
        second.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                          now,
                          semantic::TraversalGuardValues(),
                          passVerdict());
        second.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                          now,
                          semantic::TraversalGuardValues(),
                          failVerdict());
        second.applyEvent(semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                          now,
                          semantic::TraversalGuardValues(),
                          passVerdict());
        EXPECT_EQ(
            second.applyEvent(semantic::RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
                              now + run * 0.1,
                              semantic::TraversalGuardValues(),
                              failVerdict()),
            semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        EXPECT_FALSE(second.getLastEvent().accepted);
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

    const std::vector<std::pair<semantic::RoomTrackingState, std::vector<std::string>>>
        definedRows = {
            {semantic::RoomTrackingState::UNKNOWN, {"FIRST_ROOM_CONFIRMED"}},
            {semantic::RoomTrackingState::CONFIRMED_ROOM,
             {"PASSAGE_CROSSING_DETECTED", "TRACKING_LOST"}},
            {semantic::RoomTrackingState::CROSSING_PASSAGE,
             {"PASSAGE_TRAVERSAL_COMPLETE", "TRACKING_LOST"}},
            {semantic::RoomTrackingState::LOST_WITHOUT_ROOM, {"ROOM_REACQUIRED"}},
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
            if (std::find(entry.second.begin(),
                          entry.second.end(),
                          semantic::RoomTracker::eventToString(event)) !=
                entry.second.end())
            {
                continue; /* Defined for this source state. */
            }

            semantic::RoomTracker  tracker;
            const double now = 700.0;

            if (sourceState != semantic::RoomTrackingState::UNKNOWN)
            {
                /* Drive to the source state along a valid path. */
                tracker.applyEvent(semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                                   now,
                                   semantic::TraversalGuardValues(),
                                   passVerdict());
                if (sourceState == semantic::RoomTrackingState::CONFIRMED_ROOM)
                {
                    /* Already reached. */
                }
                else if (sourceState == semantic::RoomTrackingState::CROSSING_PASSAGE)
                {
                    semantic::TraversalGuardValues crossed = nominalCrossing();
                    crossed.dwell_s = tracker.getConfig().crossing_dwell_s;
                    tracker.applyEvent(
                        semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                        now,
                        crossed,
                        passVerdict());
                }
                else if (sourceState == semantic::RoomTrackingState::LOST_WITH_LAST_ROOM)
                {
                    tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                       now,
                                       semantic::TraversalGuardValues(),
                                       failVerdict());
                }
                else if (sourceState == semantic::RoomTrackingState::LOST_WITHOUT_ROOM)
                {
                    tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                       now,
                                       semantic::TraversalGuardValues(),
                                       failVerdict());
                    tracker.applyEvent(semantic::RoomTrackingEvent::LOST_TIMEOUT,
                                       now,
                                       semantic::TraversalGuardValues(),
                                       failVerdict());
                }
                else if (sourceState ==
                         semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP)
                {
                    tracker.applyEvent(semantic::RoomTrackingEvent::TRACKING_LOST,
                                       now,
                                       semantic::TraversalGuardValues(),
                                       failVerdict());
                    tracker.applyEvent(
                        semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                        now,
                        semantic::TraversalGuardValues(),
                        passVerdict());
                }
            }

            EXPECT_EQ(tracker.getState(), sourceState)
                << "setup mismatch before injecting "
                << semantic::RoomTracker::eventToString(event);
            EXPECT_EQ(tracker.applyEvent(event,
                                         now + 1.0,
                                         nominalCrossing(),
                                         passVerdict()),
                      sourceState)
                << "undefined event " << semantic::RoomTracker::eventToString(event)
                << " must not change state "
                << semantic::RoomTracker::stateToString(sourceState);
            EXPECT_FALSE(tracker.getLastEvent().accepted);
            EXPECT_EQ(tracker.getState(), sourceState);
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
        semantic::RoomTracker  tracker;
        const double t0 = 1000.0 + run * 100.0;

        /* UNKNOWN -> CONFIRMED_ROOM once a room is confirmed. */
        EXPECT_EQ(tracker.step(t0,
                               semantic::TraversalGuardValues(),
                               passVerdict(),
                               nominalTracking()),
                  semantic::RoomTrackingState::CONFIRMED_ROOM);

        /* Crossing guard holds; the dwell has not yet elapsed. */
        semantic::TraversalGuardValues crossing = nominalCrossing();
        const double         t1       = t0 + 1.0; /* < crossing_dwell_s */
        EXPECT_EQ(tracker.step(t1, crossing, passVerdict(), nominalTracking()),
                  semantic::RoomTrackingState::CONFIRMED_ROOM);

        /* Now the dwell has elapsed: the crossing commits. */
        const double t2 = t1 + tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(tracker.step(t2, crossing, passVerdict(), nominalTracking()),
                  semantic::RoomTrackingState::CROSSING_PASSAGE);

        /* Traversal completion needs both sides + dwell + verdict PASS. */
        semantic::TraversalGuardValues complete = nominalCrossing();
        complete.bothSidesObserved    = true;
        const double t3 = t2 + tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(tracker.step(t3, complete, passVerdict(), nominalTracking()),
                  semantic::RoomTrackingState::CROSSING_PASSAGE);
        const double t4 = t3 + tracker.getConfig().crossing_dwell_s;
        EXPECT_EQ(tracker.step(t4, complete, passVerdict(), nominalTracking()),
                  semantic::RoomTrackingState::CONFIRMED_ROOM);

        EXPECT_TRUE(tracker.getLastEvent().accepted);
        EXPECT_GE(tracker.getEventHistory().size(), 3U);
    }
}

TEST(RoomTrackerStep, HysteresisResetsDwellOnGuardFailure)
{
    semantic::RoomTracker  tracker;
    const double t0 = 0.0;
    tracker.step(t0, semantic::TraversalGuardValues(), passVerdict(), nominalTracking());

    semantic::TraversalGuardValues crossing = nominalCrossing();
    const double         t1       = t0 + 1.0;
    EXPECT_EQ(tracker.step(t1, crossing, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CONFIRMED_ROOM);

    /* Guard fails (confidence drops below threshold); dwell must reset. */
    semantic::TraversalGuardValues weak = crossing;
    weak.confidence           = 0.0;
    const double t2           = t1 + 1.0;
    EXPECT_EQ(tracker.step(t2, weak, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CONFIRMED_ROOM);

    /* Re-satisfying the guard restarts the full dwell window. */
    const double t3 = t2 + tracker.getConfig().crossing_dwell_s - 0.5;
    EXPECT_EQ(tracker.step(t3, crossing, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    const double t4 = t3 + tracker.getConfig().crossing_dwell_s;
    EXPECT_EQ(tracker.step(t4, crossing, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CROSSING_PASSAGE);
}

TEST(RoomTrackerStep, InvalidCrossingGuardsFailClosed)
{
    semantic::RoomTracker tracker;
    tracker.step(10.0,
                 semantic::TraversalGuardValues(),
                 passVerdict(),
                 nominalTracking());

    semantic::TraversalGuardValues invalid = nominalCrossing();
    invalid.confidence           = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(tracker.step(11.0, invalid, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CONFIRMED_ROOM);

    invalid            = nominalCrossing();
    invalid.confidence = 1.1;
    EXPECT_EQ(tracker.step(12.0, invalid, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CONFIRMED_ROOM);
}

TEST(RoomTrackerStep, BothSidesAndDwellAccumulateAcrossCycles)
{
    semantic::RoomTracker tracker;
    tracker.step(100.0,
                 semantic::TraversalGuardValues(),
                 passVerdict(),
                 nominalTracking());

    semantic::TraversalGuardValues crossing = nominalCrossing();
    EXPECT_EQ(tracker.step(101.0, crossing, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    EXPECT_EQ(tracker.step(103.0, crossing, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CROSSING_PASSAGE);

    semantic::TraversalGuardValues oneSide = nominalCrossing();
    oneSide.bothSidesObserved    = true;
    EXPECT_EQ(tracker.step(104.0, oneSide, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CROSSING_PASSAGE);
    EXPECT_EQ(tracker.step(105.0,
                           nominalCrossing(),
                           passVerdict(),
                           nominalTracking()),
              semantic::RoomTrackingState::CROSSING_PASSAGE);
    EXPECT_EQ(tracker.step(106.0,
                           nominalCrossing(),
                           passVerdict(),
                           nominalTracking()),
              semantic::RoomTrackingState::CONFIRMED_ROOM);
}

TEST(RoomTrackerStep, SteadyDomainInputIgnoresTimestampDiscontinuity)
{
    semantic::RoomTracker tracker;
    tracker.step(1000.0,
                 semantic::TraversalGuardValues(),
                 passVerdict(),
                 nominalTracking());

    semantic::TraversalGuardValues crossing = nominalCrossing();
    EXPECT_EQ(tracker.step(1001.0, crossing, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    /* A SLAM timestamp reset must not make the injected steady-domain clock
     * move backwards or complete the dwell early. */
    EXPECT_EQ(tracker.step(2.0, crossing, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    EXPECT_EQ(tracker.step(1003.0, crossing, passVerdict(), nominalTracking()),
              semantic::RoomTrackingState::CROSSING_PASSAGE);
}

TEST(RoomTrackerStep, TrackingLossIsConsumedOncePerEpisode)
{
    semantic::RoomTracker tracker;
    tracker.step(1.0, semantic::TraversalGuardValues(), passVerdict(), nominalTracking());

    semantic::TrackingStatusInput lost;
    lost.lost = true;
    tracker.step(2.0, semantic::TraversalGuardValues(), failVerdict(), lost);
    const std::size_t eventCountAfterLoss = tracker.getEventHistory().size();
    tracker.step(3.0, semantic::TraversalGuardValues(), failVerdict(), lost);
    EXPECT_EQ(tracker.getEventHistory().size(), eventCountAfterLoss);
}

TEST(RoomTrackerStep, TimeoutDecaysToLostWithoutRoom)
{
    semantic::RoomTracker  tracker;
    const double t0 = 2000.0;
    tracker.step(t0, semantic::TraversalGuardValues(), passVerdict(), nominalTracking());

    /* Tracking is lost once: CONFIRMED_ROOM -> LOST_WITH_LAST_ROOM. */
    semantic::TrackingStatusInput lost;
    lost.lost = true;
    EXPECT_EQ(
        tracker.step(t0 + 1.0, semantic::TraversalGuardValues(), failVerdict(), lost),
        semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);

    /* Loss persists; the lost_timeout decays to LOST_WITHOUT_ROOM. */
    const double tExpire = t0 + 1.0 + tracker.getConfig().lost_timeout_s;
    EXPECT_EQ(
        tracker.step(tExpire, semantic::TraversalGuardValues(), failVerdict(), lost),
        semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
    EXPECT_TRUE(tracker.getLastEvent().accepted);
}

TEST(RoomTrackerStep, UndefinedTrackingLossFromUnknownIsRejected)
{
    semantic::RoomTracker         tracker;
    semantic::TrackingStatusInput lost;
    lost.lost = true;
    EXPECT_EQ(tracker.step(0.0, semantic::TraversalGuardValues(), failVerdict(), lost),
              semantic::RoomTrackingState::UNKNOWN);
    EXPECT_FALSE(tracker.getLastEvent().accepted);
    EXPECT_EQ(tracker.getLastEvent().event, semantic::RoomTrackingEvent::TRACKING_LOST);
}

TEST(RoomTrackerStep, UnavailableVerificationCannotConfirmOrMutateState)
{
    semantic::RoomTracker         tracker;
    semantic::VerificationVerdict unavailable;
    unavailable.pass = true;
    semantic::TraversalGuardValues crossing;
    semantic::TrackingStatusInput  tracking;

    EXPECT_EQ(tracker.step(10.0, crossing, unavailable, tracking),
              semantic::RoomTrackingState::UNKNOWN);
    EXPECT_EQ(tracker.getEventHistory().size(), 0U);
}

TEST(RoomTrackerStep, MalformedPassVerdictFailsClosed)
{
    semantic::VerificationVerdict malformed = passVerdict();
    malformed.inlierRatio         = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(malformed.isPass());

    malformed            = passVerdict();
    malformed.confidence = -0.1;
    EXPECT_FALSE(malformed.isPass());

    malformed      = passVerdict();
    malformed.pass = false;
    EXPECT_FALSE(malformed.isPass());

    malformed        = passVerdict();
    malformed.status = semantic::VerificationStatus::REJECTED;
    EXPECT_FALSE(malformed.isPass());
}

TEST(RoomTrackerStep, ReacquireRetriesThenTimeouts)
{
    semantic::RoomTracker  tracker;
    const double t0 = 3000.0;
    tracker.step(t0, semantic::TraversalGuardValues(), passVerdict(), nominalTracking());

    semantic::TrackingStatusInput lost;
    lost.lost = true;
    EXPECT_EQ(
        tracker.step(t0 + 1.0, semantic::TraversalGuardValues(), failVerdict(), lost),
        semantic::RoomTrackingState::LOST_WITH_LAST_ROOM);

    /* New map (still lost) with a verified match -> REACQUIRING. */
    semantic::TrackingStatusInput newMap;
    newMap.lost          = true;
    newMap.newMapCreated = true;
    EXPECT_EQ(
        tracker.step(t0 + 2.0, semantic::TraversalGuardValues(), passVerdict(), newMap),
        semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);

    /* Failed verification retries at the configured interval; after the
     * configured max retries the reacquire is retired. */
    const double       tRetry = t0 + 2.0;
    double             t      = tRetry;
    const unsigned int expectedRetries =
        tracker.getConfig().reacquire_max_retries;
    for (unsigned int attempt = 0U; attempt < expectedRetries + 1U; ++attempt)
    {
        t += tracker.getConfig().reacquire_retry_interval_s;
        const semantic::RoomTrackingState state =
            tracker.step(t, semantic::TraversalGuardValues(), failVerdict(), newMap);
        if (attempt < expectedRetries)
        {
            EXPECT_EQ(state, semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP);
        }
    }
    EXPECT_EQ(tracker.getState(), semantic::RoomTrackingState::LOST_WITHOUT_ROOM);
    EXPECT_TRUE(tracker.getLastEvent().accepted);
    EXPECT_EQ(tracker.getLastEvent().event,
              semantic::RoomTrackingEvent::REACQUIRE_TIMEOUT);
}

} // namespace core
} // namespace vs_graphs
