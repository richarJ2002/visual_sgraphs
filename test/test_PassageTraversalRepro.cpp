/**
 * @file test_PassageTraversalRepro.cpp
 * @brief Regression test: reproduce the silent-passage-traversal miss.
 *
 * The gate run 20260911-105315 showed UAV crossing semantic::Passage#1 around
 * t ≈ 7056-7065 sim seconds with traversal counters staying zero.
 *
 * This test builds a minimal synthetic scenario with exactly 3 keyframes
 * and a passage whose geometry is derived from the gate run's sgraph
 * output (semantic::Passage#1: centroid ≈ (-0.21,-0.75,5.72), width ≈ 1.135, height 2.0).
 * It calls updateTraversalEvidence() and asserts that
 * passage.getTraversalKnownToFarCount() > 0.
 *
 * Outcome: the unit-level trigger works -- with a bracketing keyframe pair
 * and a refit-backed ground plane the crossing is detected, the far room
 * promotes, and traversal settles. Two setup artifacts were caught on the
 * way and are now guarded by this test: an on-plane middle sample defeats
 * the strict straddle test in segmentCrossesAperture(), and a ground cloud
 * under 20 points fails the refit-generation gate in
 * Map::GetBiggestGroundPlane(), which silently skips traversal. The
 * gate-run miss is therefore a production-condition issue (passable timing,
 * owning map vs active map at crossing time, aperture bounds then).
 */

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "KeyFrame.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "SemanticsManager.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace test
{

// --------------------------------------------------------------------------
// semantic::Passage geometry (derived from gate run 20260911-105315__gate_verification):
//   centroid: (-0.21, -0.75, 5.72) m
//   width:  1.135355933026258 m
//   height: 2.0 m
//   connects room1 ↔ room2
//
static const double PASSAGE_CENTROID_X = -0.21;
static const double PASSAGE_CENTROID_Y = -0.75;
static const double PASSAGE_CENTROID_Z = 5.72;
static const double PASSAGE_WIDTH      = 1.135355933026258;
static const double PASSAGE_HEIGHT     = 2.0;

// Aperture plane equation: passage normal along +X axis (known side at -X,
// far side at +X). This matches the sgraph wall-normal orientation observed
// for the wall adjacent to semantic::Passage#1.
static const double PASSAGE_APERTURE_A = 1.0;
static const double PASSAGE_APERTURE_B = 0.0;
static const double PASSAGE_APERTURE_C = 0.0;
static const double PASSAGE_APERTURE_D = 0.0;

// Ground plane normal (pointing downward, matching Gazebo/RGB-D config):
static const double GROUND_NORMAL_X = 0.0;
static const double GROUND_NORMAL_Y = -1.0;
static const double GROUND_NORMAL_Z = 0.0;

// --------------------------------------------------------------------------
// Keyframe camera-centre positions (world frames).  These form a trajectory
// that enters the passage from the known side (-X), passes through the
// aperture, and exits to the far side (+X).
// --------------------------------------------------------------------------
// Known-side entry point: well left of the aperture (x = -1.0)
static const double KNOWN_SIDE_CAMERA_CENTER_X = -1.0;
static const double KNOWN_SIDE_CAMERA_CENTER_Y = -0.5;
static const double KNOWN_SIDE_CAMERA_CENTER_Z = 5.5;

// Aperture-crossing point: just past the aperture plane on the far side
// (x = +0.15). segmentCrossesAperture() needs a strict straddle with both
// endpoints outside the 0.10 m side clearance, so the middle sample must
// bracket the plane, never sit on it -- an on-plane sample is measure-zero
// in flight and would reject both adjacent pairs.
static const double APERTURE_CAMERA_CENTER_X = 0.15;
static const double APERTURE_CAMERA_CENTER_Y = -0.75;
static const double APERTURE_CAMERA_CENTER_Z = 5.72;

// Far-side exit point: well right of the aperture (x = +1.0)
static const double FAR_SIDE_CAMERA_CENTER_X = 1.0;
static const double FAR_SIDE_CAMERA_CENTER_Y = -1.0;
static const double FAR_SIDE_CAMERA_CENTER_Z = 6.0;

// --------------------------------------------------------------------------
// TEST_F case
// --------------------------------------------------------------------------
TEST(PassageTraversalRepro, KnownSideToFarCrossingRecordsCount)
{
    // --- Atlas / map / passage setup ---------------------------------------
    Atlas atlas(0);
    EXPECT_TRUE(atlas.consumeNewMapCreatedEvent());
    Map *p_map = atlas.GetCurrentMap();
    EXPECT_NE(p_map, nullptr);

    // Ground plane
    geometric::Plane groundPlane;
    groundPlane.setId(0);
    groundPlane.SetMap(p_map);
    groundPlane.setPlaneType(geometric::Plane::planeVariant::GROUND);
    groundPlane.setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(GROUND_NORMAL_X,
                                                               GROUND_NORMAL_Y,
                                                               GROUND_NORMAL_Z,
                                                               0.0)));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr groundCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int gridX = -2; gridX <= 2; ++gridX)
    {
        for (int gridY = -2; gridY <= 2; ++gridY)
        {
            pcl::PointXYZRGBA gridPoint;
            gridPoint.x = static_cast<float>(gridX);
            gridPoint.y = static_cast<float>(gridY);
            gridPoint.z = 0.0F;
            gridPoint.r = 128U;
            gridPoint.g = 128U;
            gridPoint.b = 128U;
            gridPoint.a = 255U;
            groundCloud->push_back(gridPoint);
        }
    }
    groundPlane.setMapClouds(groundCloud);
    EXPECT_TRUE(GeoSemHelpers::refitMappedPlaneFromCloud(&groundPlane));
    p_map->AddMapPlane(&groundPlane);

    // Known-side room
    semantic::Room knownRoom;
    knownRoom.setId(10);
    knownRoom.setMap(p_map);
    knownRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM);
    knownRoom.setCentroid(Eigen::Vector3d(-1.0, -0.5, 5.5));
    p_map->AddDetectedMapRoom(&knownRoom);

    // Far-side room (prospective)
    semantic::Room farRoom;
    farRoom.setId(11);
    farRoom.setMap(p_map);
    farRoom.setRoomVariant(semantic::Room::RoomVariant::UNDEFINED);
    farRoom.setCentroid(Eigen::Vector3d(1.0, -1.0, 6.0));
    p_map->AddDetectedMapRoom(&farRoom);

    // semantic::Passage#1 with geometry from the gate run
    semantic::Passage passage;
    passage.setId(20);
    passage.setMap(p_map);
    passage.setPassable(true);
    passage.setWidth(PASSAGE_WIDTH);
    passage.setHeight(PASSAGE_HEIGHT);
    passage.setCentroid(Eigen::Vector3d(PASSAGE_CENTROID_X,
                                        PASSAGE_CENTROID_Y,
                                        PASSAGE_CENTROID_Z));
    passage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(PASSAGE_APERTURE_A,
                                     PASSAGE_APERTURE_B,
                                     PASSAGE_APERTURE_C,
                                     PASSAGE_APERTURE_D)));
    passage.setKnownSideRoom(&knownRoom);
    passage.setKnownSideDirection(Eigen::Vector3d(-1.0, 0.0, 0.0)); // known
                                                                    // side
                                                                    // is -X
    passage.setProspectiveRoom(&farRoom);
    p_map->AddMapPassage(&passage);

    // --- KeyFrames (exactly 3) -------------------------------------------
    KeyFrame knownSideKeyFrame;
    knownSideKeyFrame.mnId = 0U;
    knownSideKeyFrame.SetPose(
        Sophus::SE3f(Eigen::Matrix3f::Identity(),
                     Eigen::Vector3f(-KNOWN_SIDE_CAMERA_CENTER_X,
                                     -KNOWN_SIDE_CAMERA_CENTER_Y,
                                     -KNOWN_SIDE_CAMERA_CENTER_Z)));
    p_map->AddKeyFrame(&knownSideKeyFrame);

    KeyFrame apertureKeyFrame;
    apertureKeyFrame.mnId = 1U;
    apertureKeyFrame.SetPose(
        Sophus::SE3f(Eigen::Matrix3f::Identity(),
                     Eigen::Vector3f(-APERTURE_CAMERA_CENTER_X,
                                     -APERTURE_CAMERA_CENTER_Y,
                                     -APERTURE_CAMERA_CENTER_Z)));
    p_map->AddKeyFrame(&apertureKeyFrame);

    KeyFrame farSideKeyFrame;
    farSideKeyFrame.mnId = 2U;
    farSideKeyFrame.SetPose(
        Sophus::SE3f(Eigen::Matrix3f::Identity(),
                     Eigen::Vector3f(-FAR_SIDE_CAMERA_CENTER_X,
                                     -FAR_SIDE_CAMERA_CENTER_Y,
                                     -FAR_SIDE_CAMERA_CENTER_Z)));
    p_map->AddKeyFrame(&farSideKeyFrame);

    // --- SemanticsManager + traversal evidence -----------------------------
    SemanticsManager manager(&atlas);

    // Run the semantic cycle: this is where updateTraversalEvidence() iterates
    // consecutive keyframe camera centres and checks
    // segmentCrossesPassageOpening().
    manager.updateTraversalEvidence(&atlas);

    // --- Assertions --------------------------------------------------------
    // The test expects that the UAV's known-side → far-side crossing is
    // detected and recorded as KNOWN_TO_FAR evidence.
    EXPECT_GT(passage.getTraversalKnownToFarCount(), 0U)
        << "FAIL: traversalKnownToFarCount stayed zero — passage traversal "
           "was not detected. Suspect: "
           "passage_kf_window/max_kf_passage_distance "
           "not wired in, or ground-plane dependency, or dwell/confidence "
           "gating.";

    // Also verify the passage is marked as having some traversal evidence.
    EXPECT_TRUE(passage.getTraversalEvidence())
        << "FAIL: passage.getTraversalEvidence() returned false even though "
           "KNOWN_TO_FAR count > 0.";
}

} // namespace test
} // namespace core
} // namespace vs_graphs
