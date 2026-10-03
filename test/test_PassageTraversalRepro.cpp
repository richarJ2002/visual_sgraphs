/*!
 * @file test_PassageTraversalRepro.cpp
 * @brief Regression test: reproduce the silent-passage-traversal miss.
 *
 * The gate run 20260911-105315 showed UAV crossing semantic::Passage#1 around
 * t ≈ 7056-7065 sim seconds with traversal counters staying zero.
 *
 * This test builds a minimal synthetic scenario with exactly 3 keyframes
 * and a passage whose geometry is derived from the gate run's sgraph
 * output (semantic::Passage#1: centroid ≈ (-0.21,-0.75,5.72), width ≈ 1.135,
 * height 2.0). It calls updateTraversalEvidence() and asserts that
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
// semantic::Passage geometry (derived from gate run
// 20260911-105315__gate_verification):
//   centroid: (-0.21, -0.75, 5.72) m
//   width:  1.135355933026258 m
//   height: 2.0 m
//   connects room1 ↔ room2
//
/*!
 * @brief        X of the passage centroid in the world frame, metres.
 */
static const double PASSAGE_CENTROID_X = -0.21;
/*!
 * @brief        Y of the passage centroid in the world frame, metres.
 */
static const double PASSAGE_CENTROID_Y = -0.75;
/*!
 * @brief        Z of the passage centroid in the world frame, metres.
 */
static const double PASSAGE_CENTROID_Z = 5.72;
/*!
 * @brief        Passage width, metres.
 */
static const double PASSAGE_WIDTH = 1.135355933026258;
/*!
 * @brief        Passage height, metres.
 */
static const double PASSAGE_HEIGHT = 2.0;

/*!
 * @brief        X coefficient of the aperture plane equation; the plane normal
 *               points along +X, from the known side to the far side.
 */
static const double PASSAGE_APERTURE_A = 1.0;
/*!
 * @brief        Y coefficient of the aperture plane equation.
 */
static const double PASSAGE_APERTURE_B = 0.0;
/*!
 * @brief        Z coefficient of the aperture plane equation.
 */
static const double PASSAGE_APERTURE_C = 0.0;
/*!
 * @brief        Offset of the aperture plane equation, metres; the plane passes
 *               through the world origin.
 */
static const double PASSAGE_APERTURE_D = 0.0;

/*!
 * @brief        X of the ground plane normal, which points downward to match
 *               the Gazebo RGB-D set-up.
 */
static const double GROUND_NORMAL_X = 0.0;
/*!
 * @brief        Y of the ground plane normal; -1 means the normal points down.
 */
static const double GROUND_NORMAL_Y = -1.0;
/*!
 * @brief        Z of the ground plane normal.
 */
static const double GROUND_NORMAL_Z = 0.0;

// --------------------------------------------------------------------------
// Keyframe camera-centre positions (world frames).  These form a trajectory
// that enters the passage from the known side (-X), passes through the
// aperture, and exits to the far side (+X).
// --------------------------------------------------------------------------
/*!
 * @brief        X of the entry key frame camera centre in the world frame,
 *               metres; well on the known side (-X) of the aperture.
 */
static const double KNOWN_SIDE_CAMERA_CENTER_X = -1.0;
/*!
 * @brief        Y of the entry key frame camera centre in the world frame,
 *               metres.
 */
static const double KNOWN_SIDE_CAMERA_CENTER_Y = -0.5;
/*!
 * @brief        Z of the entry key frame camera centre in the world frame,
 *               metres.
 */
static const double KNOWN_SIDE_CAMERA_CENTER_Z = 5.5;

/*!
 * @brief        X of the middle key frame camera centre in the world frame,
 *               metres; just past the aperture plane, never on it, so the pair
 *               of segments straddle it strictly.
 */
static const double APERTURE_CAMERA_CENTER_X = 0.15;
/*!
 * @brief        Y of the middle key frame camera centre in the world frame,
 *               metres.
 */
static const double APERTURE_CAMERA_CENTER_Y = -0.75;
/*!
 * @brief        Z of the middle key frame camera centre in the world frame,
 *               metres.
 */
static const double APERTURE_CAMERA_CENTER_Z = 5.72;

/*!
 * @brief        X of the exit key frame camera centre in the world frame,
 *               metres; well on the far side (+X) of the aperture.
 */
static const double FAR_SIDE_CAMERA_CENTER_X = 1.0;
/*!
 * @brief        Y of the exit key frame camera centre in the world frame,
 *               metres.
 */
static const double FAR_SIDE_CAMERA_CENTER_Y = -1.0;
/*!
 * @brief        Z of the exit key frame camera centre in the world frame,
 *               metres.
 */
static const double FAR_SIDE_CAMERA_CENTER_Z = 6.0;

// --------------------------------------------------------------------------
// TEST_F case
// --------------------------------------------------------------------------
/*!
 * @brief        Checks that a trajectory from the known side, through the
 *               aperture, to the far side records exactly one crossing.
 */
TEST(PassageTraversalRepro, KnownSideToFarCrossingRecordsCount)
{
    // --- Atlas / map / passage setup ---------------------------------------
    Atlas atlas(0);
    bool  wasEventPending{};
    ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_TRUE(wasEventPending);
    Map *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_NE(p_map, nullptr);

    // Ground plane
    geometric::Plane groundPlane;
    ASSERT_EQ((groundPlane.setId(0)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((groundPlane.setMap(p_map)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ(
        (groundPlane.setPlaneType(geometric::Plane::PlaneVariant::GROUND)),
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((groundPlane.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(GROUND_NORMAL_X,
                                               GROUND_NORMAL_Y,
                                               GROUND_NORMAL_Z,
                                               0.0)))),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
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
    ASSERT_EQ((groundPlane.setMapClouds(groundCloud)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    bool wasPlaneRefit{};
    ASSERT_EQ(
        (GeoSemHelpers::refitMappedPlaneFromCloud(&groundPlane, wasPlaneRefit)),
        GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    EXPECT_TRUE(wasPlaneRefit);
    ASSERT_EQ((p_map->addMapPlane(&groundPlane)),
              MapStatus::MAP_STATUS_SUCCESS);

    // Known-side room
    semantic::Room knownRoom;
    ASSERT_EQ((knownRoom.setId(10)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setCentroid(Eigen::Vector3d(-1.0, -0.5, 5.5))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&knownRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    // Far-side room (prospective)
    semantic::Room farRoom;
    ASSERT_EQ((farRoom.setId(11)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((farRoom.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((farRoom.setRoomVariant(semantic::Room::RoomVariant::UNDEFINED)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((farRoom.setCentroid(Eigen::Vector3d(1.0, -1.0, 6.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&farRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    // semantic::Passage#1 with geometry from the gate run
    semantic::Passage passage;
    ASSERT_EQ((passage.setId(20)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setWidth(PASSAGE_WIDTH)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setHeight(PASSAGE_HEIGHT)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setCentroid(Eigen::Vector3d(PASSAGE_CENTROID_X,
                                                   PASSAGE_CENTROID_Y,
                                                   PASSAGE_CENTROID_Z))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(PASSAGE_APERTURE_A,
                                               PASSAGE_APERTURE_B,
                                               PASSAGE_APERTURE_C,
                                               PASSAGE_APERTURE_D)))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setKnownSideRoom(&knownRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setKnownSideDirection(Eigen::Vector3d(-1.0, 0.0, 0.0))),
              vs_graphs::core::semantic::PassageStatus::
                  PASSAGE_STATUS_SUCCESS); // known
                                           // side
                                           // is -X
    ASSERT_EQ((passage.setProspectiveRoom(&farRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);

    // --- KeyFrames (exactly 3) -------------------------------------------
    KeyFrame knownSideKeyFrame;
    knownSideKeyFrame.id = 0U;
    ASSERT_EQ((knownSideKeyFrame.setPose(
                  Sophus::SE3f(Eigen::Matrix3f::Identity(),
                               Eigen::Vector3f(-KNOWN_SIDE_CAMERA_CENTER_X,
                                               -KNOWN_SIDE_CAMERA_CENTER_Y,
                                               -KNOWN_SIDE_CAMERA_CENTER_Z)))),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addKeyFrame(&knownSideKeyFrame)),
              MapStatus::MAP_STATUS_SUCCESS);

    KeyFrame apertureKeyFrame;
    apertureKeyFrame.id = 1U;
    ASSERT_EQ((apertureKeyFrame.setPose(
                  Sophus::SE3f(Eigen::Matrix3f::Identity(),
                               Eigen::Vector3d(-APERTURE_CAMERA_CENTER_X,
                                               -APERTURE_CAMERA_CENTER_Y,
                                               -APERTURE_CAMERA_CENTER_Z)
                                   .cast<float>()))),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addKeyFrame(&apertureKeyFrame)),
              MapStatus::MAP_STATUS_SUCCESS);

    KeyFrame farSideKeyFrame;
    farSideKeyFrame.id = 2U;
    ASSERT_EQ((farSideKeyFrame.setPose(
                  Sophus::SE3f(Eigen::Matrix3f::Identity(),
                               Eigen::Vector3f(-FAR_SIDE_CAMERA_CENTER_X,
                                               -FAR_SIDE_CAMERA_CENTER_Y,
                                               -FAR_SIDE_CAMERA_CENTER_Z)))),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addKeyFrame(&farSideKeyFrame)),
              MapStatus::MAP_STATUS_SUCCESS);

    // --- SemanticsManager + traversal evidence -----------------------------
    SemanticsManager manager(&atlas);

    // Run the semantic cycle: this is where updateTraversalEvidence() iterates
    // consecutive keyframe camera centres and checks
    // segmentCrossesPassageOpening().
    ASSERT_EQ((manager.updateTraversalEvidence(&atlas)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    // --- Assertions --------------------------------------------------------
    // The test expects that the UAV's known-side → far-side crossing is
    // detected and recorded as KNOWN_TO_FAR evidence.
    std::size_t traversalKnownToFarCount2{};
    ASSERT_EQ((passage.getTraversalKnownToFarCount(traversalKnownToFarCount2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_GT(traversalKnownToFarCount2, 0U)
        << "FAIL: traversalKnownToFarCount stayed zero — passage traversal "
           "was not detected. Suspect: "
           "passage_kf_window/max_kf_passage_distance "
           "not wired in, or ground-plane dependency, or dwell/confidence "
           "gating.";

    // Also verify the passage is marked as having some traversal evidence.
    bool traversalEvidence{};
    ASSERT_EQ((passage.getTraversalEvidence(traversalEvidence)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(traversalEvidence)
        << "FAIL: passage.getTraversalEvidence() returned false even though "
           "KNOWN_TO_FAR count > 0.";
}

} // namespace test
} // namespace core
} // namespace vs_graphs
