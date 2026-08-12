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
 * Phase 0 unit-test skeleton (WP13 Section 19.1). Deterministic helpers only;
 * no Gazebo, no ROS, no multi-process harness. Three of the tests here
 * exercise RoomTracker pure helpers so the GTest baseline has content before
 * the transition oracle tests (test_RoomTracker).
 */

#include "Semantic/RoomTracker.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace ORB_SLAM3
{

/*!
 * @brief State/event literal names are stable, parseable identifiers.
 */
TEST(RoomTrackerSkeleton, StateAndEventLiteralsAreStable)
{
    EXPECT_EQ(RoomTracker::stateToString(RoomTrackingState::UNKNOWN), "UNKNOWN");
    EXPECT_EQ(RoomTracker::stateToString(RoomTrackingState::CONFIRMED_ROOM),
              "CONFIRMED_ROOM");
    EXPECT_EQ(RoomTracker::stateToString(RoomTrackingState::CROSSING_PASSAGE),
              "CROSSING_PASSAGE");
    EXPECT_EQ(RoomTracker::stateToString(RoomTrackingState::LOST_WITHOUT_ROOM),
              "LOST_WITHOUT_ROOM");
    EXPECT_EQ(RoomTracker::stateToString(RoomTrackingState::LOST_WITH_LAST_ROOM),
              "LOST_WITH_LAST_ROOM");
    EXPECT_EQ(
        RoomTracker::stateToString(RoomTrackingState::REACQUIRING_IN_NEW_MAP),
        "REACQUIRING_IN_NEW_MAP");

    EXPECT_EQ(RoomTracker::eventToString(RoomTrackingEvent::FIRST_ROOM_CONFIRMED),
              "FIRST_ROOM_CONFIRMED");
    EXPECT_EQ(
        RoomTracker::eventToString(RoomTrackingEvent::PASSAGE_CROSSING_DETECTED),
        "PASSAGE_CROSSING_DETECTED");
    EXPECT_EQ(RoomTracker::eventToString(RoomTrackingEvent::TRACKING_LOST),
              "TRACKING_LOST");
    EXPECT_EQ(
        RoomTracker::eventToString(RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE),
        "PASSAGE_TRAVERSAL_COMPLETE");
    EXPECT_EQ(RoomTracker::eventToString(RoomTrackingEvent::ROOM_REACQUIRED),
              "ROOM_REACQUIRED");
    EXPECT_EQ(
        RoomTracker::eventToString(RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH),
        "NEW_MAP_WITH_ROOM_MATCH");
    EXPECT_EQ(
        RoomTracker::eventToString(RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM),
        "VERIFIED_MATCH_TO_LAST_ROOM");
    EXPECT_EQ(RoomTracker::eventToString(RoomTrackingEvent::LOST_TIMEOUT),
              "LOST_TIMEOUT");
    EXPECT_EQ(RoomTracker::eventToString(RoomTrackingEvent::REACQUIRE_TIMEOUT),
              "REACQUIRE_TIMEOUT");
}

/*!
 * @brief Section 18.3 confidence formula matches hand-computed values.
 */
TEST(RoomTrackerSkeleton, ConfidenceFormula)
{
    /* Perfect evidence yields full confidence. */
    EXPECT_DOUBLE_EQ(RoomTracker::computeConfidence(1.0, 0.0, 0.0, 0.5), 1.0);

    /* angular_residual > 0 with sigma=0.25: exp(-0.4/0.25)=exp(-1.6). */
    EXPECT_DOUBLE_EQ(RoomTracker::computeConfidence(1.0, 0.0, 0.4, 0.25),
                     std::exp(-1.6));

    /* Poor conditioning scales the inlier ratio back. */
    EXPECT_DOUBLE_EQ(RoomTracker::computeConfidence(0.8, 0.5, 0.0, 0.5), 0.4);

    /* Inputs are clamped to their domains. */
    EXPECT_DOUBLE_EQ(RoomTracker::computeConfidence(2.0, -1.0, 0.0, 0.5), 1.0);
    EXPECT_DOUBLE_EQ(RoomTracker::computeConfidence(0.0, 0.0, 100.0, 0.5),
                     0.0);

    /* A non-positive sigma disables the residual term safely. */
    EXPECT_DOUBLE_EQ(RoomTracker::computeConfidence(0.5, 0.0, 0.0, 0.0), 0.5);
    EXPECT_DOUBLE_EQ(RoomTracker::computeConfidence(0.5, 0.0, 0.3, 0.0), 0.0);
}

/*!
 * @brief TransitionEvent serialises as one JSON object with all fields.
 */
TEST(RoomTrackerSkeleton, EventSerialisationIsJSON)
{
    TransitionEvent event;
    event.timestamp_s      = 12.5;
    event.sourceState      = RoomTrackingState::CONFIRMED_ROOM;
    event.event            = RoomTrackingEvent::TRACKING_LOST;
    event.targetState      = RoomTrackingState::LOST_WITH_LAST_ROOM;
    event.dwell_s          = 0.0;
    event.confidence       = 0.0;
    event.verificationPass = false;
    event.accepted         = true;

    const std::string json = RoomTracker::eventToJSON(event);
    EXPECT_NE(json.find("\"source\":\"CONFIRMED_ROOM\""), std::string::npos);
    EXPECT_NE(json.find("\"event\":\"TRACKING_LOST\""), std::string::npos);
    EXPECT_NE(json.find("\"target\":\"LOST_WITH_LAST_ROOM\""),
              std::string::npos);
    EXPECT_NE(json.find("\"timestamp\":12.5"), std::string::npos);
    EXPECT_NE(json.find("\"accepted\":true"), std::string::npos);
    EXPECT_NE(json.find("\"verification_pass\":false"), std::string::npos);
}

/*!
 * @brief Configuration defaults follow Section 18.4/18.5.
 */
TEST(RoomTrackerSkeleton, DefaultConfiguration)
{
    RoomTracker tracker;
    const RoomTrackerConfig &config = tracker.getConfig();
    EXPECT_DOUBLE_EQ(config.crossing_dwell_s, 2.0);
    EXPECT_DOUBLE_EQ(config.crossing_confidence, 0.7);
    EXPECT_DOUBLE_EQ(config.lost_timeout_s, 30.0);
    EXPECT_DOUBLE_EQ(config.reacquire_timeout_s, 60.0);
    EXPECT_DOUBLE_EQ(config.reacquire_retry_interval_s, 5.0);
    EXPECT_EQ(config.reacquire_max_retries, 3U);
    EXPECT_EQ(config.reacquire_min_planes, 3U);
}

} // namespace ORB_SLAM3