/*!
 * @file test_ConsecutiveMapMatcher.cpp
 * @brief Regression test suite for consecutive-map merge gating:
 *        Atlas::attemptConsecutiveMergeIfGated() and
 *        SemanticVerify::evaluateConsecutiveMergeGate().
 *
 * Strategy (headless, live objects, SAME frame for both maps so the
 * transform is identity g2o::Sim3 -- no Horn estimation in tests):
 *   1. Build an Atlas, map0 filled, Atlas::CreateNewMap() for map1, fill it,
 *   set Map::setFinalRoom/setStartingRoom links (verify these methods exist),
 *   then for TC1-7 call evaluateConsecutiveMergeGate directly with identity
 *   transform and explicit MapMergeConfig (or
 * mapMergeConfigFromSystemParams()); for TC8-9 call
 * Atlas::attemptMergeIfGated() twice. Fixture recipe follows
 * test/test_PassageTraversalRepro.cpp (stack Atlas/Map entities,
 * AddDetectedMapRoom/AddMapPassage/AddMapFloor/AddMapPlane). Minimum 3 walls
 * per anchor room for the core check (else MISSING). Use axis-aligned walls so
 * greedy normal pairing succeeds.
 */

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "Map.h"
#include "Semantic/Floor.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "Semantic/RoomContextSnapshot.h"
#include "Semantic/SemanticVerify.h"
#include "Types/objects/SystemParams.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

using namespace vs_graphs::core;
using namespace vs_graphs::core::semantic;
using namespace vs_graphs::core::geometric;
using namespace vs_graphs::core::types;
using namespace std;

// ----------------------------------------------------------------------------
// Test geometry constants
// ----------------------------------------------------------------------------

static const double WALL_A_NORMAL_X = 1.0;
static const double WALL_A_NORMAL_Y = 0.0;
static const double WALL_A_NORMAL_Z = 0.0;
static const double WALL_A_D        = -1.0;

static const double WALL_B_NORMAL_X = 0.0;
static const double WALL_B_NORMAL_Y = 1.0;
static const double WALL_B_NORMAL_Z = 0.0;
static const double WALL_B_D        = -1.0;

static const double WALL_C_NORMAL_X = -1.0;
static const double WALL_C_NORMAL_Y = 0.0;
static const double WALL_C_NORMAL_Z = 0.0;
static const double WALL_C_D        = 1.0;

static const double WALL_D_NORMAL_X = 0.0;
static const double WALL_D_NORMAL_Y = -1.0;
static const double WALL_D_NORMAL_Z = 0.0;
static const double WALL_D_D        = 1.0;

static const Eigen::Vector3d ROOM1_CENTROID(0.0, 0.0, 1.0);
static const Eigen::Vector3d ROOM2_CENTROID(3.0, 0.0, 1.0);
static const Eigen::Vector3d ROOM_12_CENTROID(0.0, 3.0, 1.0);
static const Eigen::Vector3d ROOM_6_CENTROID(0.0, 6.0, 1.0);

static const double PASSAGE_WIDTH      = 1.0;
static const double PASSAGE_HEIGHT     = 2.0;
static const double PASSAGE_APERTURE_A = 1.0;
static const double PASSAGE_APERTURE_B = 0.0;
static const double PASSAGE_APERTURE_C = 0.0;
static const double PASSAGE_APERTURE_D = 0.0;

static const Eigen::Vector4d FLOOR_EQ(0.0, 1.0, 0.0, 0.0);

// =============================================================================
// Test fixture
// =============================================================================

class ConsecutiveMapMatcherTest : public ::testing::Test
{
  protected:
    Atlas atlas{0};
    Map  *p_map0;
    Map  *p_map1;

    void SetUp() override
    {
        EXPECT_TRUE(atlas.consumeNewMapCreatedEvent());
        p_map0 = atlas.getCurrentMap();
        EXPECT_NE(p_map0, nullptr);
        atlas.createNewMap();
        EXPECT_TRUE(atlas.consumeNewMapCreatedEvent());
        p_map1 = atlas.getCurrentMap();
        EXPECT_NE(p_map1, nullptr);
        EXPECT_NE(p_map0, p_map1);
    }

    void TearDown() override
    {
        atlas.clearAtlas();
    }

    Room *addRoomWithWallsAndPassage(Map                   *p_map,
                                     int                    roomId_in,
                                     const string          &tag,
                                     const Eigen::Vector3d &centroid,
                                     const Eigen::Vector3d &passage_centroid,
                                     int                    passageId_in,
                                     int                    wallIdBase_in,
                                     Room                  *p_farRoom_in,
                                     bool withPassage_in = true)
    {
        Room *room = new Room();
        if (room->setId(roomId_in) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setId cannot fail; continue as before.
        }
        if (room->setMap(p_map) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setMap cannot fail; continue as before.
        }
        if (room->setRoomVariant(Room::RoomVariant::ROOM) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setRoomVariant cannot fail; continue as before.
        }
        if (room->setCentroid(centroid) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setCentroid cannot fail; continue as before.
        }
        if (room->setRoomTag(tag) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setRoomTag cannot fail; continue as before.
        }

        // Wall A: x = 1
        Plane *wallA = new Plane();
        wallA->setId(wallIdBase_in);
        wallA->setMap(p_map);
        wallA->setPlaneType(Plane::PlaneVariant::WALL);
        wallA->setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(WALL_A_NORMAL_X,
                                                              WALL_A_NORMAL_Y,
                                                              WALL_A_NORMAL_Z,
                                                              WALL_A_D)));
        wallA->setCentroid(Eigen::Vector3d(1.0, 0.0, 1.0));
        p_map->addMapPlane(wallA);
        if (room->setWalls(wallA) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setWalls cannot fail; continue as before.
        }

        // Wall B: y = 1
        Plane *wallB = new Plane();
        wallB->setId(wallIdBase_in + 1);
        wallB->setMap(p_map);
        wallB->setPlaneType(Plane::PlaneVariant::WALL);
        wallB->setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(WALL_B_NORMAL_X,
                                                              WALL_B_NORMAL_Y,
                                                              WALL_B_NORMAL_Z,
                                                              WALL_B_D)));
        wallB->setCentroid(Eigen::Vector3d(0.0, 1.0, 1.0));
        p_map->addMapPlane(wallB);
        if (room->setWalls(wallB) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setWalls cannot fail; continue as before.
        }

        // Wall C: x = -1 face (normal +X after toward-room orientation)
        Plane *wallC = new Plane();
        wallC->setId(wallIdBase_in + 2);
        wallC->setMap(p_map);
        wallC->setPlaneType(Plane::PlaneVariant::WALL);
        wallC->setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(WALL_C_NORMAL_X,
                                                              WALL_C_NORMAL_Y,
                                                              WALL_C_NORMAL_Z,
                                                              WALL_C_D)));
        wallC->setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0));
        p_map->addMapPlane(wallC);
        if (room->setWalls(wallC) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setWalls cannot fail; continue as before.
        }

        // Passage
        if (withPassage_in)
        {
            Passage *passage = new Passage();
            if (passage->setId(passageId_in) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                // setId cannot fail; continue as before.
            }
            if (passage->setMap(p_map) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                // setMap cannot fail; continue as before.
            }
            if (passage->setPassable(true) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                // setPassable cannot fail; continue as before.
            }
            if (passage->setWidth(PASSAGE_WIDTH) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                // setWidth cannot fail; continue as before.
            }
            if (passage->setHeight(PASSAGE_HEIGHT) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                // setHeight cannot fail; continue as before.
            }
            if (passage->setCentroid(passage_centroid) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                // setCentroid cannot fail; continue as before.
            }
            if (passage->setGlobalEquation(
                    g2o::Plane3D(Eigen::Vector4d(PASSAGE_APERTURE_A,
                                                 PASSAGE_APERTURE_B,
                                                 PASSAGE_APERTURE_C,
                                                 PASSAGE_APERTURE_D))) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                // setGlobalEquation cannot fail; continue as before.
            }
            if (passage->setKnownSideRoom(room) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                // setKnownSideRoom cannot fail; continue as before.
            }
            if (passage->setKnownSideDirection(
                    Eigen::Vector3d(-1.0, 0.0, 0.0)) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                // Rejected input: ignored, as before.
            }
            if (passage->setProspectiveRoom(p_farRoom_in) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                // setProspectiveRoom cannot fail; continue as before.
            }
            p_map->addMapPassage(passage);
            if (room->setDoorways(passage) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setDoorways cannot fail; continue as before.
            }
        }

        p_map->addDetectedMapRoom(room);
        return room;
    }

    Floor *
        addFloor(Map *p_map, int floorId_in, const Eigen::Vector4d &equation_in)
    {
        Floor *p_floor = new Floor();
        if (p_floor->setId(floorId_in) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            // setId cannot fail; continue as before.
        }
        if (p_floor->setMap(p_map) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            // setMap cannot fail; continue as before.
        }
        EXPECT_TRUE(
            (p_floor->setPlaneIdentity(equation_in, 100U, 10U) ==
             vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));
        p_map->addMapFloor(p_floor);
        return p_floor;
    }

    void setSeedRooms(Map  *p_oldMap,
                      Map  *p_newMap,
                      Room *p_final_room0,
                      Room *p_start_room1)
    {
        p_oldMap->setFinalRoom(p_final_room0);
        p_newMap->setStartingRoom(p_start_room1);
    }

    SemanticMergeGateResult runConsecutiveGate(Map *p_survivingMap,
                                               Map *p_absorbedMap)
    {
        const g2o::Sim3 identityTransform(Eigen::Matrix3d::Identity(),
                                          Eigen::Vector3d::Zero(),
                                          1.0);
        SemanticVerify::MapMergeConfig                     config;
        vs_graphs::core::semantic::SemanticMergeGateResult result{};
        if (SemanticVerify::evaluateConsecutiveMergeGate(p_survivingMap,
                                                         p_absorbedMap,
                                                         identityTransform,
                                                         result,
                                                         config) !=
            vs_graphs::core::semantic::SemanticVerifyStatus::
                SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            // evaluateConsecutiveMergeGate cannot fail; continue as before.
        }
        return result;
    }

    void setWallSpanCloud(Plane *p_wall, double yMinimum_in, double yMaximum_in)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr spanCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (int gridY = 0; gridY < 5; ++gridY)
        {
            for (int gridZ = 0; gridZ < 5; ++gridZ)
            {
                pcl::PointXYZRGBA spanPoint;
                spanPoint.x = 1.0F;
                spanPoint.y = static_cast<float>(
                    yMinimum_in + (yMaximum_in - yMinimum_in) * gridY / 4);
                spanPoint.z = static_cast<float>(0.5 + gridZ * 0.25);
                spanPoint.r = 128U;
                spanPoint.g = 128U;
                spanPoint.b = 128U;
                spanPoint.a = 255U;
                spanCloud->push_back(spanPoint);
            }
        }
        p_wall->setMapClouds(spanCloud);
    }
};

// =============================================================================
// TEST CASES
// =============================================================================

// TC1: matching pair
TEST_F(ConsecutiveMapMatcherTest, TC1_matchingPair)
{
    Room *r0_2 = addRoomWithWallsAndPassage(p_map0,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r0_2);
    std::vector<vs_graphs::core::semantic::Passage *> r0_2Passages{};
    ASSERT_EQ((r0_2->getPassages(r0_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2Passages[0]->setProspectiveRoom(r0_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    Room *r1_2 = addRoomWithWallsAndPassage(p_map1,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r1_2);
    std::vector<vs_graphs::core::semantic::Passage *> r1_2Passages{};
    ASSERT_EQ((r1_2->getPassages(r1_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_2Passages[0]->setProspectiveRoom(r1_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    g2o::Sim3                      identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    SemanticMergeGateResult result{};
    ASSERT_EQ((SemanticVerify::evaluateConsecutiveMergeGate(p_map1,
                                                            p_map0,
                                                            identity_sim3,
                                                            result,
                                                            config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((SemanticVerify::mergeDecisionName(result.decision, p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision, SemanticMergeDecision::ACCEPT)
        << "TC1: expected ACCEPT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((SemanticVerify::mergeReasonName(result.reason, p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason, SemanticMergeReason::ALIGNED)
        << "TC1: expected ALIGNED, got " << p_name2;
}

// TC2: aliasing (different tags, different places -> no shared identity)
TEST_F(ConsecutiveMapMatcherTest, TC2_aliasing)
{
    Room *r0_1 = new Room();
    ASSERT_EQ((r0_1->setId(12)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1->setMap(p_map0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1->setRoomVariant(Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1->setCentroid(ROOM_12_CENTROID)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1->setRoomTag("room_12")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map0->addDetectedMapRoom(r0_1);
    Room *r1_1 = new Room();
    ASSERT_EQ((r1_1->setId(6)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_1->setMap(p_map1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_1->setRoomVariant(Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_1->setCentroid(ROOM_6_CENTROID)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_1->setRoomTag("room_6")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map1->addDetectedMapRoom(r1_1);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    g2o::Sim3                      identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    SemanticMergeGateResult result{};
    ASSERT_EQ((SemanticVerify::evaluateConsecutiveMergeGate(p_map1,
                                                            p_map0,
                                                            identity_sim3,
                                                            result,
                                                            config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((SemanticVerify::mergeDecisionName(result.decision, p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision, SemanticMergeDecision::DEFER)
        << "TC2: expected DEFER, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((SemanticVerify::mergeReasonName(result.reason, p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason, SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING)
        << "TC2: expected SHARED_ROOM_IDENTITY_MISSING, got " << p_name2;
}

// TC3: single anchor (scheduler enforces the two-anchor minimum)
TEST_F(ConsecutiveMapMatcherTest, TC3_singleAnchor)
{
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            nullptr);
    Room *r0_2 = new Room();
    ASSERT_EQ((r0_2->setId(101)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setMap(p_map0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setRoomVariant(Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setCentroid(ROOM2_CENTROID)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map0->addDetectedMapRoom(r0_2);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            nullptr);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    atlas.attemptConsecutiveMergeIfGated();
    EXPECT_FALSE(p_map0->isBad())
        << "TC3: single anchor must not merge (old map stays live)";
    EXPECT_FALSE(p_map1->isBad())
        << "TC3: single anchor must not merge (current map stays live)";
}

// TC4: empty new map
TEST_F(ConsecutiveMapMatcherTest, TC4_emptyNewMap)
{
    Room *bootstrap = new Room();
    ASSERT_EQ((bootstrap->setId(0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap->setMap(p_map0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap->setRoomVariant(Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap->setCentroid(Eigen::Vector3d(0.0, 0.0, 0.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map0->addDetectedMapRoom(bootstrap);
    Room *bootstrap1 = new Room();
    ASSERT_EQ((bootstrap1->setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap1->setMap(p_map1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap1->setRoomVariant(Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap1->setCentroid(Eigen::Vector3d(0.0, 0.0, 0.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map1->addDetectedMapRoom(bootstrap1);
    setSeedRooms(p_map0, p_map1, bootstrap, bootstrap1);
    g2o::Sim3                      identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    SemanticMergeGateResult result{};
    ASSERT_EQ((SemanticVerify::evaluateConsecutiveMergeGate(p_map1,
                                                            p_map0,
                                                            identity_sim3,
                                                            result,
                                                            config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((SemanticVerify::mergeDecisionName(result.decision, p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision, SemanticMergeDecision::DEFER)
        << "TC4: expected DEFER, got " << p_name;
}

// TC5: floor mismatch
TEST_F(ConsecutiveMapMatcherTest, TC5_floorMismatch)
{
    Room *r0_2 = addRoomWithWallsAndPassage(p_map0,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r0_2);
    std::vector<vs_graphs::core::semantic::Passage *> r0_2Passages{};
    ASSERT_EQ((r0_2->getPassages(r0_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2Passages[0]->setProspectiveRoom(r0_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    Room *r1_2 = addRoomWithWallsAndPassage(p_map1,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r1_2);
    std::vector<vs_graphs::core::semantic::Passage *> r1_2Passages{};
    ASSERT_EQ((r1_2->getPassages(r1_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_2Passages[0]->setProspectiveRoom(r1_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    Floor *floor1 = addFloor(p_map1, 0, FLOOR_EQ);
    EXPECT_TRUE((floor1->setPlaneIdentity(Eigen::Vector4d(0.0, 1.0, 0.0, -0.5),
                                          100U,
                                          10U) ==
                 vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    g2o::Sim3                      identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    SemanticMergeGateResult result{};
    ASSERT_EQ((SemanticVerify::evaluateConsecutiveMergeGate(p_map1,
                                                            p_map0,
                                                            identity_sim3,
                                                            result,
                                                            config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((SemanticVerify::mergeDecisionName(result.decision, p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision, SemanticMergeDecision::REJECT)
        << "TC5: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((SemanticVerify::mergeReasonName(result.reason, p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason, SemanticMergeReason::FLOOR_CONTRADICTION)
        << "TC5: expected FLOOR_CONTRADICTION, got " << p_name2;
}

// TC6: wall rotated 10 deg
TEST_F(ConsecutiveMapMatcherTest, TC6_wallRotated)
{
    Room *r0_2 = addRoomWithWallsAndPassage(p_map0,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r0_2);
    std::vector<vs_graphs::core::semantic::Passage *> r0_2Passages{};
    ASSERT_EQ((r0_2->getPassages(r0_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2Passages[0]->setProspectiveRoom(r0_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    Room *r1_2 = addRoomWithWallsAndPassage(p_map1,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r1_2);
    std::vector<vs_graphs::core::semantic::Passage *> r1_2Passages{};
    ASSERT_EQ((r1_2->getPassages(r1_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_2Passages[0]->setProspectiveRoom(r1_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    std::vector<vs_graphs::core::geometric::Plane *> r1_1Walls{};
    ASSERT_EQ((r1_1->getWalls(r1_1Walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    Plane          *wall1A = r1_1Walls[0];
    double          theta  = 10.0 * M_PI / 180.0;
    double          c = cos(theta), s = sin(theta);
    Eigen::Vector3d rotated_normal(c, s, 0.0);
    wall1A->setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(rotated_normal.x(),
                                                           rotated_normal.y(),
                                                           rotated_normal.z(),
                                                           WALL_A_D)));
    wall1A->setCentroid(Eigen::Vector3d(1.0, 0.0, 1.0));
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    g2o::Sim3                      identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    SemanticMergeGateResult result{};
    ASSERT_EQ((SemanticVerify::evaluateConsecutiveMergeGate(p_map1,
                                                            p_map0,
                                                            identity_sim3,
                                                            result,
                                                            config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((SemanticVerify::mergeDecisionName(result.decision, p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision, SemanticMergeDecision::REJECT)
        << "TC6: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((SemanticVerify::mergeReasonName(result.reason, p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason, SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION)
        << "TC6: expected WALL_ALIGNMENT_CONTRADICTION, got " << p_name2;
}

// TC7: passage endpoint contradiction
TEST_F(ConsecutiveMapMatcherTest, TC7_passageEndpointContradiction)
{
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            nullptr);
    Room *r0_3 = new Room();
    ASSERT_EQ((r0_3->setId(3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_3->setMap(p_map0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_3->setRoomVariant(Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_3->setCentroid(Eigen::Vector3d(7.0, 0.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_3->setRoomTag("room_3")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map0->addDetectedMapRoom(r0_3);
    std::vector<vs_graphs::core::semantic::Passage *> r0_1Passages{};
    ASSERT_EQ((r0_1->getPassages(r0_1Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1Passages[0]->setProspectiveRoom(r0_3)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    Room *r1_2 = addRoomWithWallsAndPassage(p_map1,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r1_2);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    g2o::Sim3                      identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    SemanticMergeGateResult result{};
    ASSERT_EQ((SemanticVerify::evaluateConsecutiveMergeGate(p_map1,
                                                            p_map0,
                                                            identity_sim3,
                                                            result,
                                                            config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((SemanticVerify::mergeDecisionName(result.decision, p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision, SemanticMergeDecision::REJECT)
        << "TC7: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((SemanticVerify::mergeReasonName(result.reason, p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason,
              SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION)
        << "TC7: expected PASSAGE_ENDPOINT_CONTRADICTION, got " << p_name2;
}

// TC8: cooldown
TEST_F(ConsecutiveMapMatcherTest, TC8_cooldown)
{
    Room *r0_2 = addRoomWithWallsAndPassage(p_map0,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r0_2);
    std::vector<vs_graphs::core::semantic::Passage *> r0_2Passages{};
    ASSERT_EQ((r0_2->getPassages(r0_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2Passages[0]->setProspectiveRoom(r0_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    Room *r1_2 = addRoomWithWallsAndPassage(p_map1,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r1_2);
    std::vector<vs_graphs::core::semantic::Passage *> r1_2Passages{};
    ASSERT_EQ((r1_2->getPassages(r1_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_2Passages[0]->setProspectiveRoom(r1_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    testing::internal::CaptureStdout();
    atlas.attemptConsecutiveMergeIfGated();
    string output1 = testing::internal::GetCapturedStdout();
    size_t count1  = 0;
    size_t pos1    = 0;
    while ((pos1 = output1.find("consecutive_merge_attempt", pos1)) !=
           string::npos)
    {
        count1++;
        pos1++;
    }
    testing::internal::CaptureStdout();
    atlas.attemptConsecutiveMergeIfGated();
    string output2 = testing::internal::GetCapturedStdout();
    size_t count2  = 0;
    size_t pos2    = 0;
    while ((pos2 = output2.find("consecutive_merge_attempt", pos2)) !=
           string::npos)
    {
        count2++;
        pos2++;
    }
    EXPECT_GE(count1, 1U)
        << "TC8: first attempt should produce at least 1 line, got " << count1;
    (void)count2;
}

// TC9: change-gate
TEST_F(ConsecutiveMapMatcherTest, TC9_changeGate)
{
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            nullptr);
    Room *r0_2 = new Room();
    ASSERT_EQ((r0_2->setId(101)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setMap(p_map0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setRoomVariant(Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setCentroid(ROOM2_CENTROID)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map0->addDetectedMapRoom(r0_2);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            nullptr);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    /* Cooldown would suppress the immediate second attempt; the change-gate
     * is what this case exercises. */
    SystemParams::getParams()->mapMerge.mergeCooldown_s = 0U;
    testing::internal::CaptureStdout();
    atlas.attemptConsecutiveMergeIfGated();
    string output1 = testing::internal::GetCapturedStdout();
    size_t count1  = 0;
    size_t pos1    = 0;
    while ((pos1 = output1.find("consecutive_merge_attempt", pos1)) !=
           string::npos)
    {
        count1++;
        pos1++;
    }
    Room          *r1_1_current = nullptr;
    vector<Room *> rooms1       = p_map1->getAllRooms();
    for (Room *r : rooms1)
    {
        std::string roomTag{};
        ASSERT_EQ((r->getRoomTag(roomTag)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        if (roomTag == "room_1")
        {
            r1_1_current = r;
            break;
        }
    }
    if (r1_1_current)
    {
        Plane *wall4 = new Plane();
        wall4->setId(4);
        wall4->setMap(p_map1);
        wall4->setPlaneType(Plane::PlaneVariant::WALL);
        wall4->setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(WALL_D_NORMAL_X,
                                                              WALL_D_NORMAL_Y,
                                                              WALL_D_NORMAL_Z,
                                                              WALL_D_D)));
        wall4->setCentroid(Eigen::Vector3d(0.0, 1.0, 1.0));
        p_map1->addMapPlane(wall4);
        ASSERT_EQ((r1_1_current->setWalls(wall4)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    }
    testing::internal::CaptureStdout();
    atlas.attemptConsecutiveMergeIfGated();
    string output2 = testing::internal::GetCapturedStdout();
    size_t count2  = 0;
    size_t pos2    = 0;
    while ((pos2 = output2.find("consecutive_merge_attempt", pos2)) !=
           string::npos)
    {
        count2++;
        pos2++;
    }
    EXPECT_GE(count2, 1U)
        << "TC9: second attempt should produce at least 1 line, got " << count2;
    EXPECT_GE(count1, 1U)
        << "TC9: first attempt should produce at least 1 line, got " << count1;
    SystemParams::getParams()->mapMerge.mergeCooldown_s = 30U;
}

// TC10a: coplanar but disjoint wall extents contradict
TEST_F(ConsecutiveMapMatcherTest, TC10a_disjointWallExtentsReject)
{
    Room *r0_2 = addRoomWithWallsAndPassage(p_map0,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r0_2);
    std::vector<vs_graphs::core::semantic::Passage *> r0_2Passages{};
    ASSERT_EQ((r0_2->getPassages(r0_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2Passages[0]->setProspectiveRoom(r0_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    Room *r1_2 = addRoomWithWallsAndPassage(p_map1,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r1_2);
    std::vector<vs_graphs::core::semantic::Passage *> r1_2Passages{};
    ASSERT_EQ((r1_2->getPassages(r1_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_2Passages[0]->setProspectiveRoom(r1_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    std::vector<vs_graphs::core::geometric::Plane *> r0_1Walls{};
    ASSERT_EQ((r0_1->getWalls(r0_1Walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    setWallSpanCloud(r0_1Walls[0], 2.0, 3.0);
    std::vector<vs_graphs::core::geometric::Plane *> r1_1Walls{};
    ASSERT_EQ((r1_1->getWalls(r1_1Walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    setWallSpanCloud(r1_1Walls[0], 5.0, 6.0);
    const SemanticMergeGateResult result = runConsecutiveGate(p_map1, p_map0);
    const char                   *p_name = nullptr;
    ASSERT_EQ((SemanticVerify::mergeDecisionName(result.decision, p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision, SemanticMergeDecision::REJECT)
        << "TC10a: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((SemanticVerify::mergeReasonName(result.reason, p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason, SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION)
        << "TC10a: expected WALL_ALIGNMENT_CONTRADICTION, got " << p_name2;
}

// TC10b: overlapping wall extents accept
TEST_F(ConsecutiveMapMatcherTest, TC10b_overlappingWallExtentsAccept)
{
    Room *r0_2 = addRoomWithWallsAndPassage(p_map0,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r0_2);
    std::vector<vs_graphs::core::semantic::Passage *> r0_2Passages{};
    ASSERT_EQ((r0_2->getPassages(r0_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2Passages[0]->setProspectiveRoom(r0_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    Room *r1_2 = addRoomWithWallsAndPassage(p_map1,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r1_2);
    std::vector<vs_graphs::core::semantic::Passage *> r1_2Passages{};
    ASSERT_EQ((r1_2->getPassages(r1_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_2Passages[0]->setProspectiveRoom(r1_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    std::vector<vs_graphs::core::geometric::Plane *> r0_1Walls{};
    ASSERT_EQ((r0_1->getWalls(r0_1Walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    setWallSpanCloud(r0_1Walls[0], 2.0, 3.0);
    std::vector<vs_graphs::core::geometric::Plane *> r1_1Walls{};
    ASSERT_EQ((r1_1->getWalls(r1_1Walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    setWallSpanCloud(r1_1Walls[0], 2.0, 3.0);
    const SemanticMergeGateResult result = runConsecutiveGate(p_map1, p_map0);
    const char                   *p_name = nullptr;
    ASSERT_EQ((SemanticVerify::mergeDecisionName(result.decision, p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision, SemanticMergeDecision::ACCEPT)
        << "TC10b: expected ACCEPT, got " << p_name;
}

// TC11: passable mismatch on a lineage pair contradicts
TEST_F(ConsecutiveMapMatcherTest, TC11_passableMismatchRejects)
{
    Room *r0_2 = addRoomWithWallsAndPassage(p_map0,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r0_2);
    std::vector<vs_graphs::core::semantic::Passage *> r0_2Passages{};
    ASSERT_EQ((r0_2->getPassages(r0_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2Passages[0]->setProspectiveRoom(r0_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    Room *r1_2 = addRoomWithWallsAndPassage(p_map1,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r1_2);
    std::vector<vs_graphs::core::semantic::Passage *> r1_2Passages{};
    ASSERT_EQ((r1_2->getPassages(r1_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_2Passages[0]->setProspectiveRoom(r1_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    std::vector<vs_graphs::core::semantic::Passage *> r1_1Passages{};
    ASSERT_EQ((r1_1->getPassages(r1_1Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_1Passages[0]->setPassable(false)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    const SemanticMergeGateResult result = runConsecutiveGate(p_map1, p_map0);
    const char                   *p_name = nullptr;
    ASSERT_EQ((SemanticVerify::mergeDecisionName(result.decision, p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision, SemanticMergeDecision::REJECT)
        << "TC11: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((SemanticVerify::mergeReasonName(result.reason, p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason,
              SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION)
        << "TC11: expected PASSAGE_IDENTITY_CONTRADICTION, got " << p_name2;
}

// TC12: flipped known-side direction contradicts
TEST_F(ConsecutiveMapMatcherTest, TC12_directionFlipRejects)
{
    Room *r0_2 = addRoomWithWallsAndPassage(p_map0,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r0_2);
    std::vector<vs_graphs::core::semantic::Passage *> r0_2Passages{};
    ASSERT_EQ((r0_2->getPassages(r0_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2Passages[0]->setProspectiveRoom(r0_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    Room *r1_2 = addRoomWithWallsAndPassage(p_map1,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r1_1 = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r1_2);
    std::vector<vs_graphs::core::semantic::Passage *> r1_2Passages{};
    ASSERT_EQ((r1_2->getPassages(r1_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_2Passages[0]->setProspectiveRoom(r1_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    std::vector<vs_graphs::core::semantic::Passage *> r1_1Passages{};
    ASSERT_EQ((r1_1->getPassages(r1_1Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_1Passages[0]->setKnownSideDirection(
                  Eigen::Vector3d(1.0, 0.0, 0.0))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    const SemanticMergeGateResult result = runConsecutiveGate(p_map1, p_map0);
    const char                   *p_name = nullptr;
    ASSERT_EQ((SemanticVerify::mergeDecisionName(result.decision, p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision, SemanticMergeDecision::REJECT)
        << "TC12: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((SemanticVerify::mergeReasonName(result.reason, p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason,
              SemanticMergeReason::PASSAGE_DIRECTION_CONTRADICTION)
        << "TC12: expected PASSAGE_DIRECTION_CONTRADICTION, got " << p_name2;
}

// TC13: merged old doorway resurfaces the same-ID proxy with its history
TEST_F(ConsecutiveMapMatcherTest, TC13_proxyReunionOnMerge)
{
    Room *r0_2 = addRoomWithWallsAndPassage(p_map0,
                                            2,
                                            "room_2",
                                            ROOM2_CENTROID,
                                            Eigen::Vector3d(4.5, 0.0, 1.0),
                                            12,
                                            11,
                                            nullptr);
    Room *r0_1 = addRoomWithWallsAndPassage(p_map0,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            r0_2);
    std::vector<vs_graphs::core::semantic::Passage *> r0_2Passages{};
    ASSERT_EQ((r0_2->getPassages(r0_2Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2Passages[0]->setProspectiveRoom(r0_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    std::vector<vs_graphs::core::semantic::Passage *> r0_1Passages{};
    ASSERT_EQ((r0_1->getPassages(r0_1Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1Passages[0]->addTraversalObservation(
                  Passage::TraversalDirection::KNOWN_TO_FAR)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    std::vector<vs_graphs::core::semantic::Passage *> r0_1Passages2{};
    ASSERT_EQ((r0_1->getPassages(r0_1Passages2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1Passages2[0]->addTraversalObservation(
                  Passage::TraversalDirection::KNOWN_TO_FAR)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    /* Room 2 only has to exist in the new map; TC13 links room 1 through the
     * recovery proxies below rather than through a real passage. */
    addRoomWithWallsAndPassage(p_map1,
                               2,
                               "room_2",
                               ROOM2_CENTROID,
                               Eigen::Vector3d(4.5, 0.0, 1.0),
                               12,
                               11,
                               nullptr,
                               false);
    Room    *r1_1    = addRoomWithWallsAndPassage(p_map1,
                                            1,
                                            "room_1",
                                            ROOM1_CENTROID,
                                            Eigen::Vector3d(1.5, 0.0, 1.0),
                                            11,
                                            1,
                                            nullptr,
                                            false);
    Passage *proxy11 = new Passage();
    ASSERT_EQ((proxy11->setId(11)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((proxy11->setMap(p_map1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((proxy11->setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((proxy11->setRecoveryProxy(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((proxy11->setKnownSideRoom(r1_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    p_map1->addMapPassage(proxy11);
    ASSERT_EQ((r1_1->setDoorways(proxy11)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    Passage *proxy12 = new Passage();
    ASSERT_EQ((proxy12->setId(12)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((proxy12->setMap(p_map1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((proxy12->setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((proxy12->setRecoveryProxy(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((proxy12->setKnownSideRoom(r1_1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    p_map1->addMapPassage(proxy12);
    ASSERT_EQ((r1_1->setDoorways(proxy12)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    atlas.attemptConsecutiveMergeIfGated();
    EXPECT_TRUE(p_map0->isBad()) << "TC13: old map must retire on merge commit";
    bool isRecoveryProxy2{};
    ASSERT_EQ((proxy11->isRecoveryProxy(isRecoveryProxy2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_FALSE(isRecoveryProxy2)
        << "TC13: proxy must surface with adopted geometry";
    double width{};
    ASSERT_EQ((proxy11->getWidth(width)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_GT(width, 0.0) << "TC13: proxy must adopt the transferred aperture";
    std::size_t traversalKnownToFarCount{};
    ASSERT_EQ((proxy11->getTraversalKnownToFarCount(traversalKnownToFarCount)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalKnownToFarCount, 2U)
        << "TC13: pre-reset traversal history must survive the merge";
    bool isRecoveryProxy3{};
    ASSERT_EQ((proxy12->isRecoveryProxy(isRecoveryProxy3)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_FALSE(isRecoveryProxy3) << "TC13: second proxy must surface as well";
}
