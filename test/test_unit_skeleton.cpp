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
 * Unit-test skeleton. Deterministic helpers only;
 * no Gazebo, no ROS, no multi-process harness. Three of the tests here
 * exercise semantic::RoomTracker pure helpers so the GTest baseline has content
 * before the transition oracle tests (test_RoomTracker).
 */

/*!
 * @file            test_unit_skeleton.cpp
 *
 * @brief           Skeleton unit tests for the room tracker
 *                  (RoomTrackerSkeleton).
 */

#include "Semantic/RoomTracker.h"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief State/event literal names are stable, parseable identifiers.
 */
TEST(RoomTrackerSkeleton, StateAndEventLiteralsAreStable)
{
    std::string text{};
    ASSERT_EQ((semantic::RoomTracker::stateToString(
                  semantic::RoomTrackingState::UNKNOWN,
                  text)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text, "UNKNOWN");
    std::string text2{};
    ASSERT_EQ((semantic::RoomTracker::stateToString(
                  semantic::RoomTrackingState::CONFIRMED_ROOM,
                  text2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text2, "CONFIRMED_ROOM");
    std::string text3{};
    ASSERT_EQ((semantic::RoomTracker::stateToString(
                  semantic::RoomTrackingState::CROSSING_PASSAGE,
                  text3)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text3, "CROSSING_PASSAGE");
    std::string text4{};
    ASSERT_EQ((semantic::RoomTracker::stateToString(
                  semantic::RoomTrackingState::LOST_WITHOUT_ROOM,
                  text4)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text4, "LOST_WITHOUT_ROOM");
    std::string text5{};
    ASSERT_EQ((semantic::RoomTracker::stateToString(
                  semantic::RoomTrackingState::LOST_WITH_LAST_ROOM,
                  text5)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text5, "LOST_WITH_LAST_ROOM");
    std::string text6{};
    ASSERT_EQ((semantic::RoomTracker::stateToString(
                  semantic::RoomTrackingState::REACQUIRING_IN_NEW_MAP,
                  text6)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text6, "REACQUIRING_IN_NEW_MAP");

    std::string text7{};
    ASSERT_EQ((semantic::RoomTracker::eventToString(
                  semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED,
                  text7)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text7, "FIRST_ROOM_CONFIRMED");
    std::string text8{};
    ASSERT_EQ((semantic::RoomTracker::eventToString(
                  semantic::RoomTrackingEvent::PASSAGE_CROSSING_DETECTED,
                  text8)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text8, "PASSAGE_CROSSING_DETECTED");
    std::string text9{};
    ASSERT_EQ((semantic::RoomTracker::eventToString(
                  semantic::RoomTrackingEvent::TRACKING_LOST,
                  text9)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text9, "TRACKING_LOST");
    std::string text10{};
    ASSERT_EQ((semantic::RoomTracker::eventToString(
                  semantic::RoomTrackingEvent::PASSAGE_TRAVERSAL_COMPLETE,
                  text10)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text10, "PASSAGE_TRAVERSAL_COMPLETE");
    std::string text11{};
    ASSERT_EQ((semantic::RoomTracker::eventToString(
                  semantic::RoomTrackingEvent::ROOM_REACQUIRED,
                  text11)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text11, "ROOM_REACQUIRED");
    std::string text12{};
    ASSERT_EQ((semantic::RoomTracker::eventToString(
                  semantic::RoomTrackingEvent::NEW_MAP_WITH_ROOM_MATCH,
                  text12)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text12, "NEW_MAP_WITH_ROOM_MATCH");
    std::string text13{};
    ASSERT_EQ((semantic::RoomTracker::eventToString(
                  semantic::RoomTrackingEvent::VERIFIED_MATCH_TO_LAST_ROOM,
                  text13)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text13, "VERIFIED_MATCH_TO_LAST_ROOM");
    std::string text14{};
    ASSERT_EQ((semantic::RoomTracker::eventToString(
                  semantic::RoomTrackingEvent::LOST_TIMEOUT,
                  text14)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text14, "LOST_TIMEOUT");
    std::string text15{};
    ASSERT_EQ((semantic::RoomTracker::eventToString(
                  semantic::RoomTrackingEvent::REACQUIRE_TIMEOUT,
                  text15)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_EQ(text15, "REACQUIRE_TIMEOUT");
}

/*!
 * @brief Confidence formula matches hand-computed values.
 */
TEST(RoomTrackerSkeleton, ConfidenceFormula)
{
    /* Perfect evidence yields full confidence. */
    double confidence2{};
    ASSERT_EQ((semantic::RoomTracker::computeConfidence(1.0,
                                                        0.0,
                                                        0.0,
                                                        0.5,
                                                        confidence2)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(confidence2, 1.0);

    /* angular_residual > 0 with sigma=0.25: exp(-0.4/0.25)=exp(-1.6). */
    double confidence3{};
    ASSERT_EQ((semantic::RoomTracker::computeConfidence(1.0,
                                                        0.0,
                                                        0.4,
                                                        0.25,
                                                        confidence3)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(confidence3, std::exp(-1.6));

    /* Poor conditioning scales the inlier ratio back. */
    double confidence4{};
    ASSERT_EQ((semantic::RoomTracker::computeConfidence(0.8,
                                                        0.5,
                                                        0.0,
                                                        0.5,
                                                        confidence4)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(confidence4, 0.4);

    /* Inputs are clamped to their domains. */
    double confidence5{};
    ASSERT_EQ((semantic::RoomTracker::computeConfidence(2.0,
                                                        -1.0,
                                                        0.0,
                                                        0.5,
                                                        confidence5)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(confidence5, 1.0);
    double confidence6{};
    ASSERT_EQ((semantic::RoomTracker::computeConfidence(0.0,
                                                        0.0,
                                                        100.0,
                                                        0.5,
                                                        confidence6)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(confidence6, 0.0);

    /* A non-positive sigma disables the residual term safely. */
    double confidence7{};
    ASSERT_EQ((semantic::RoomTracker::computeConfidence(0.5,
                                                        0.0,
                                                        0.0,
                                                        0.0,
                                                        confidence7)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(confidence7, 0.5);
    double confidence8{};
    ASSERT_EQ((semantic::RoomTracker::computeConfidence(0.5,
                                                        0.0,
                                                        0.3,
                                                        0.0,
                                                        confidence8)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(confidence8, 0.0);
}

/*!
 * @brief semantic::TransitionEvent serialises as one JSON object with all
 * fields.
 */
TEST(RoomTrackerSkeleton, EventSerialisationIsJSON)
{
    semantic::TransitionEvent event;
    event.timestamp_s = 12.5;
    event.sourceState = semantic::RoomTrackingState::CONFIRMED_ROOM;
    event.event       = semantic::RoomTrackingEvent::TRACKING_LOST;
    event.targetState = semantic::RoomTrackingState::LOST_WITH_LAST_ROOM;
    event.dwell_s     = 0.0;
    event.confidence  = 0.0;
    event.hasVerificationPassed = false;
    event.isAccepted            = true;

    std::string json{};
    ASSERT_EQ((semantic::RoomTracker::eventToJSON(event, json)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    EXPECT_NE(json.find("\"source\":\"CONFIRMED_ROOM\""), std::string::npos);
    EXPECT_NE(json.find("\"event\":\"TRACKING_LOST\""), std::string::npos);
    EXPECT_NE(json.find("\"target\":\"LOST_WITH_LAST_ROOM\""),
              std::string::npos);
    EXPECT_NE(json.find("\"timestamp\":12.5"), std::string::npos);
    EXPECT_NE(json.find("\"accepted\":true"), std::string::npos);
    EXPECT_NE(json.find("\"verification_pass\":false"), std::string::npos);
}

/*!
 * @brief Configuration defaults match the declared tuning values.
 */
TEST(RoomTrackerSkeleton, DefaultConfiguration)
{
    semantic::RoomTracker                               tracker;
    const vs_graphs::core::semantic::RoomTrackerConfig *p_configRef = nullptr;
    ASSERT_EQ((tracker.getConfig(p_configRef)),
              vs_graphs::core::semantic::RoomTrackerStatus::
                  ROOM_TRACKER_STATUS_SUCCESS);
    const semantic::RoomTrackerConfig &config = *p_configRef;
    EXPECT_DOUBLE_EQ(config.crossing_dwell_s, 2.0);
    EXPECT_DOUBLE_EQ(config.crossing_confidence, 0.7);
    EXPECT_DOUBLE_EQ(config.lost_timeout_s, 30.0);
    EXPECT_DOUBLE_EQ(config.reacquire_timeout_s, 60.0);
    EXPECT_DOUBLE_EQ(config.reacquire_retry_interval_s, 5.0);
    EXPECT_EQ(config.reacquire_max_retries, 3U);
    EXPECT_EQ(config.reacquire_min_planes, 3U);
}

} // namespace core
} // namespace vs_graphs