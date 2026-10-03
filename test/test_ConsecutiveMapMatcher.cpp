/*!
 * @file            test_ConsecutiveMapMatcher.cpp
 *
 * @brief           Regression test suite for consecutive-map merge gating:
 *                         Atlas::attemptConsecutiveMergeIfGated() and
 *                         SemanticVerify::evaluateConsecutiveMergeGate().
 *
 *                  Strategy (headless, live objects, SAME frame for both maps
 *                  so the transform is identity g2o::Sim3 -- no Horn estimation
 *                  in tests):
 *                    1. Build an Atlas, map0 filled, Atlas::CreateNewMap() for
 *                       map1, fill it, set `Map::setFinalRoom`/setStartingRoom
 *                       links (verify these methods exist), then for TC1-7 call
 *                       evaluateConsecutiveMergeGate directly with identity
 *                       transform and explicit MapMergeConfig (or
 *                  mapMergeConfigFromSystemParams()); for TC8-9 call
 *                  Atlas::attemptMergeIfGated() twice. Fixture recipe follows
 *                  test/test_PassageTraversalRepro.cpp (stack Atlas/Map
 *                  entities,
 *                  AddDetectedMapRoom/AddMapPassage/AddMapFloor/AddMapPlane).
 *                  Minimum 3 walls per anchor room for the core check (else
 *                  MISSING). Use axis-aligned walls so greedy normal pairing
 *                  succeeds.
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
#include <rclcpp/logging.hpp>
#include <utility>
#include <vector>

// ----------------------------------------------------------------------------
// Test geometry constants
// ----------------------------------------------------------------------------

/*!
 * @brief           X component of wall A's plane normal; wall A is the plane x
 *                  = 1 with normal (1, 0, 0) and d = -1.
 */
static const double WALL_A_NORMAL_X = 1.0;
/*!
 * @brief           Y component of wall A's plane normal.
 */
static const double WALL_A_NORMAL_Y = 0.0;
/*!
 * @brief           Z component of wall A's plane normal.
 */
static const double WALL_A_NORMAL_Z = 0.0;
/*!
 * @brief           Offset d of wall A's plane equation n.x + d = 0; with the
 *                  normal (1, 0, 0) the plane is x = 1.
 */
static const double WALL_A_D = -1.0;

/*!
 * @brief           X component of wall B's plane normal; wall B is the plane y
 *                  = 1 with normal (0, 1, 0) and d = -1.
 */
static const double WALL_B_NORMAL_X = 0.0;
/*!
 * @brief           Y component of wall B's plane normal.
 */
static const double WALL_B_NORMAL_Y = 1.0;
/*!
 * @brief           Z component of wall B's plane normal.
 */
static const double WALL_B_NORMAL_Z = 0.0;
/*!
 * @brief           Offset d of wall B's plane equation n.x + d = 0; with the
 *                  normal (0, 1, 0) the plane is y = 1.
 */
static const double WALL_B_D = -1.0;

/*!
 * @brief           X component of wall C's plane normal; wall C is the plane x
 *                  = 1 seen from the other side, normal (-1, 0, 0) and d = 1.
 */
static const double WALL_C_NORMAL_X = -1.0;
/*!
 * @brief           Y component of wall C's plane normal.
 */
static const double WALL_C_NORMAL_Y = 0.0;
/*!
 * @brief           Z component of wall C's plane normal.
 */
static const double WALL_C_NORMAL_Z = 0.0;
/*!
 * @brief           Offset d of wall C's plane equation n.x + d = 0; with the
 *                  normal (-1, 0, 0) the plane is x = 1.
 */
static const double WALL_C_D = 1.0;

/*!
 * @brief           X component of wall D's plane normal; wall D is the plane y
 *                  = 1 seen from the other side, normal (0, -1, 0) and d = 1.
 */
static const double WALL_D_NORMAL_X = 0.0;
/*!
 * @brief           Y component of wall D's plane normal.
 */
static const double WALL_D_NORMAL_Y = -1.0;
/*!
 * @brief           Z component of wall D's plane normal.
 */
static const double WALL_D_NORMAL_Z = 0.0;
/*!
 * @brief           Offset d of wall D's plane equation n.x + d = 0; with the
 *                  normal (0, -1, 0) the plane is y = 1.
 */
static const double WALL_D_D = 1.0;

/*!
 * @brief           Centroid of the room tagged room_1, in the world frame,
 *                  metres.
 */
static const Eigen::Vector3d ROOM1_CENTROID(0.0, 0.0, 1.0);
/*!
 * @brief           Centroid of the room tagged room_2, in the world frame,
 *                  metres.
 */
static const Eigen::Vector3d ROOM2_CENTROID(3.0, 0.0, 1.0);
/*!
 * @brief           Centroid of the room tagged room_12 used by the aliasing
 *                  case, in the world frame, metres.
 */
static const Eigen::Vector3d ROOM_12_CENTROID(0.0, 3.0, 1.0);
/*!
 * @brief           Centroid of the room with id 6 used by the aliasing case in
 *                  the newer map, in the world frame, metres.
 */
static const Eigen::Vector3d ROOM_6_CENTROID(0.0, 6.0, 1.0);

/*!
 * @brief           Width given to every test passage, metres.
 */
static const double PASSAGE_WIDTH = 1.0;
/*!
 * @brief           Height given to every test passage, metres.
 */
static const double PASSAGE_HEIGHT = 2.0;
/*!
 * @brief           First coefficient (normal X) of the passage aperture plane
 *                  (1, 0, 0, 0), i.e. the plane x = 0.
 */
static const double PASSAGE_APERTURE_A = 1.0;
/*!
 * @brief           Second coefficient (normal Y) of the passage aperture plane.
 */
static const double PASSAGE_APERTURE_B = 0.0;
/*!
 * @brief           Third coefficient (normal Z) of the passage aperture plane.
 */
static const double PASSAGE_APERTURE_C = 0.0;
/*!
 * @brief           Fourth coefficient (offset d) of the passage aperture plane.
 */
static const double PASSAGE_APERTURE_D = 0.0;

/*!
 * @brief           Floor plane equation (a, b, c, d) with n.x + d = 0 given to
 *                  the test floors: the plane y = 0.
 */
static const Eigen::Vector4d FLOOR_EQ(0.0, 1.0, 0.0, 0.0);

// =============================================================================
// Test fixture
// =============================================================================

/*!
 * @brief           Fixture that builds two maps in one atlas, with helpers to
 *                  add rooms, walls, passages and floors, so each test case can
 *                  run the consecutive-map merge gate on a chosen scenario.
 */
class ConsecutiveMapMatcherTest : public ::testing::Test
{
  protected:
    /*!
     * @brief           Atlas owning both test maps; destroyed with the fixture.
     */
    vs_graphs::core::Atlas atlas{0};
    /*!
     * @brief           Older map, the first one created in the atlas; the merge
     *                  gate treats it as the surviving map.
     */
    vs_graphs::core::Map  *p_map0;
    /*!
     * @brief           Newer map, created second in the atlas; the merge gate
     *                  treats it as the absorbed map.
     */
    vs_graphs::core::Map  *p_map1;

    /*!
     * @brief           Creates the two maps in the atlas, one after the other,
     *                  and stores them as p_map0 and p_map1.
     */
    void SetUp() override
    {
        bool wasEventPending{};
        ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending)),
                  vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
        EXPECT_TRUE(wasEventPending);
        vs_graphs::core::Map *p_atlasCurrentMap = nullptr;
        ASSERT_EQ((atlas.getCurrentMap(p_atlasCurrentMap)),
                  vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
        p_map0 = p_atlasCurrentMap;
        EXPECT_NE(p_map0, nullptr);
        ASSERT_EQ((atlas.createNewMap()),
                  vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
        bool wasEventPending2{};
        ASSERT_EQ((atlas.consumeNewMapCreatedEvent(wasEventPending2)),
                  vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
        EXPECT_TRUE(wasEventPending2);
        vs_graphs::core::Map *p_atlasCurrentMap2 = nullptr;
        ASSERT_EQ((atlas.getCurrentMap(p_atlasCurrentMap2)),
                  vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
        p_map1 = p_atlasCurrentMap2;
        EXPECT_NE(p_map1, nullptr);
        EXPECT_NE(p_map0, p_map1);
    }

    /*!
     * @brief           Clears the atlas, which releases both maps.
     */
    void TearDown() override
    {
        ASSERT_EQ((atlas.clearAtlas()),
                  vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
    }

    /*!
     * @brief           Builds a room with three walls (A, B and C) and,
     *                  optionally, one passage toward a far room, adds it to a
     *                  map and returns it. Wall ids are wallIdBase_in, +1 and
     *                  +2.
     *
     * @param[in]       p_map
     *                  Map that receives the room, its walls and its passage;
     *                  borrowed, not null.
     *
     * @param[in]       roomId_in
     *                  Id given to the room.
     *
     * @param[in]       tag
     *                  Room tag used to recognise the same room across maps.
     *
     * @param[in]       centroid
     *                  Room centroid in the world frame, metres.
     *
     * @param[in]       passage_centroid
     *                  Passage centroid in the world frame, metres.
     *
     * @param[in]       passageId_in
     *                  Id given to the passage; ignored when withPassage_in is
     *                  false.
     *
     * @param[in]       wallIdBase_in
     *                  Id of wall A; walls B and C take the next two ids.
     *
     * @param[in]       p_farRoom_in
     *                  Room on the far side of the passage, set as its
     *                  prospective room; may be null.
     *
     * @param[in]       withPassage_in
     *                  True to also build the passage; false builds the room
     *                  and walls only.
     *
     * @return          The new room, which the map keeps; never null.
     */
    vs_graphs::core::semantic::Room *addRoomWithWallsAndPassage(
        vs_graphs::core::Map            *p_map,
        int                              roomId_in,
        const std::string               &tag,
        const Eigen::Vector3d           &centroid,
        const Eigen::Vector3d           &passage_centroid,
        int                              passageId_in,
        int                              wallIdBase_in,
        vs_graphs::core::semantic::Room *p_farRoom_in,
        bool                             withPassage_in = true)
    {
        vs_graphs::core::semantic::Room *room =
            new vs_graphs::core::semantic::Room();
        if (room->setId(roomId_in) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (room->setMap(p_map) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (room->setRoomVariant(
                vs_graphs::core::semantic::Room::RoomVariant::ROOM) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (room->setCentroid(centroid) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (room->setRoomTag(tag) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRoomTag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        // Wall A: x = 1
        vs_graphs::core::geometric::Plane *wallA =
            new vs_graphs::core::geometric::Plane();
        if (wallA->setId(wallIdBase_in) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (wallA->setMap(p_map) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (wallA->setPlaneType(
                vs_graphs::core::geometric::Plane::PlaneVariant::WALL) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (wallA->setGlobalEquation(
                g2o::Plane3D(Eigen::Vector4d(WALL_A_NORMAL_X,
                                             WALL_A_NORMAL_Y,
                                             WALL_A_NORMAL_Z,
                                             WALL_A_D))) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (wallA->setCentroid(Eigen::Vector3d(1.0, 0.0, 1.0)) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_map->addMapPlane(wallA) !=
            vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPlane returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (room->setWalls(wallA) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        // Wall B: y = 1
        vs_graphs::core::geometric::Plane *wallB =
            new vs_graphs::core::geometric::Plane();
        if (wallB->setId(wallIdBase_in + 1) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (wallB->setMap(p_map) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (wallB->setPlaneType(
                vs_graphs::core::geometric::Plane::PlaneVariant::WALL) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (wallB->setGlobalEquation(
                g2o::Plane3D(Eigen::Vector4d(WALL_B_NORMAL_X,
                                             WALL_B_NORMAL_Y,
                                             WALL_B_NORMAL_Z,
                                             WALL_B_D))) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (wallB->setCentroid(Eigen::Vector3d(0.0, 1.0, 1.0)) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_map->addMapPlane(wallB) !=
            vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPlane returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (room->setWalls(wallB) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        // Wall C: x = -1 face (normal +X after toward-room orientation)
        vs_graphs::core::geometric::Plane *wallC =
            new vs_graphs::core::geometric::Plane();
        if (wallC->setId(wallIdBase_in + 2) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (wallC->setMap(p_map) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (wallC->setPlaneType(
                vs_graphs::core::geometric::Plane::PlaneVariant::WALL) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPlaneType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (wallC->setGlobalEquation(
                g2o::Plane3D(Eigen::Vector4d(WALL_C_NORMAL_X,
                                             WALL_C_NORMAL_Y,
                                             WALL_C_NORMAL_Z,
                                             WALL_C_D))) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (wallC->setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0)) !=
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_map->addMapPlane(wallC) !=
            vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPlane returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (room->setWalls(wallC) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        // Passage
        if (withPassage_in)
        {
            vs_graphs::core::semantic::Passage *passage =
                new vs_graphs::core::semantic::Passage();
            if (passage->setId(passageId_in) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (passage->setMap(p_map) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (passage->setPassable(true) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPassable returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (passage->setWidth(PASSAGE_WIDTH) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWidth returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (passage->setHeight(PASSAGE_HEIGHT) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setHeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (passage->setCentroid(passage_centroid) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (passage->setGlobalEquation(
                    g2o::Plane3D(Eigen::Vector4d(PASSAGE_APERTURE_A,
                                                 PASSAGE_APERTURE_B,
                                                 PASSAGE_APERTURE_C,
                                                 PASSAGE_APERTURE_D))) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (passage->setKnownSideRoom(room) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setKnownSideRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (passage->setKnownSideDirection(
                    Eigen::Vector3d(-1.0, 0.0, 0.0)) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                            "%s: setKnownSideDirection rejected its input; "
                            "continuing as before.",
                            __func__);
            }
            if (passage->setProspectiveRoom(p_farRoom_in) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_map->addMapPassage(passage) !=
                vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPassage returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (room->setDoorways(passage) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setDoorways returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        if (p_map->addDetectedMapRoom(room) !=
            vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addDetectedMapRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        return room;
    }

    vs_graphs::core::semantic::Floor *
        /*!
         * @brief           Builds a floor with the given plane equation and
         *                  adds it to a map.
         *
         * @param[in]       p_map
         *                  Map that receives the floor; borrowed, not null.
         *
         * @param[in]       floorId_in
         *                  Id given to the floor.
         *
         * @param[in]       equation_in
         *                  Floor plane equation (a, b, c, d) with n.x + d = 0.
         *
         * @return          The new floor, which the map keeps; never null.
         */
        addFloor(vs_graphs::core::Map  *p_map,
                 int                    floorId_in,
                 const Eigen::Vector4d &equation_in)
    {
        vs_graphs::core::semantic::Floor *p_floor =
            new vs_graphs::core::semantic::Floor();
        if (p_floor->setId(floorId_in) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_floor->setMap(p_map) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        EXPECT_TRUE(
            (p_floor->setPlaneIdentity(equation_in, 100U, 10U) ==
             vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));
        if (p_map->addMapFloor(p_floor) !=
            vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapFloor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        return p_floor;
    }

    /*!
     * @brief           Marks the rooms where the two maps meet: the final room
     *                  of the old map and the starting room of the new map.
     *
     * @param[in]       p_oldMap
     *                  Older map whose final room is set.
     *
     * @param[in]       p_newMap
     *                  Newer map whose starting room is set.
     *
     * @param[in]       p_final_room0
     *                  Room that becomes the final room of the old map.
     *
     * @param[in]       p_start_room1
     *                  Room that becomes the starting room of the new map.
     */
    void setSeedRooms(vs_graphs::core::Map            *p_oldMap,
                      vs_graphs::core::Map            *p_newMap,
                      vs_graphs::core::semantic::Room *p_final_room0,
                      vs_graphs::core::semantic::Room *p_start_room1)
    {
        ASSERT_EQ((p_oldMap->setFinalRoom(p_final_room0)),
                  vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
        ASSERT_EQ((p_newMap->setStartingRoom(p_start_room1)),
                  vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    }

    vs_graphs::core::semantic::SemanticMergeGateResult
        /*!
         * @brief           Runs the consecutive-map merge gate for two maps
         *                  with an identity map-to-map transform and the
         *                  default merge configuration.
         *
         * @param[in]       p_survivingMap
         *                  Map that would survive a merge.
         *
         * @param[in]       p_absorbedMap
         *                  Map that would be absorbed by a merge.
         *
         * @return          The gate result (decision and reason);
         *                  default-constructed if the gate call itself failed.
         */
        runConsecutiveGate(vs_graphs::core::Map *p_survivingMap,
                           vs_graphs::core::Map *p_absorbedMap)
    {
        const g2o::Sim3 identityTransform(Eigen::Matrix3d::Identity(),
                                          Eigen::Vector3d::Zero(),
                                          1.0);
        vs_graphs::core::semantic::SemanticVerify::MapMergeConfig config;
        vs_graphs::core::semantic::SemanticMergeGateResult        result{};
        if (vs_graphs::core::semantic::SemanticVerify::
                evaluateConsecutiveMergeGate(p_survivingMap,
                                             p_absorbedMap,
                                             identityTransform,
                                             result,
                                             config) !=
            vs_graphs::core::semantic::SemanticVerifyStatus::
                SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: evaluateConsecutiveMergeGate returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        return result;
    }

    /*!
     * @brief           Gives a wall a 5 by 5 grid of cloud points on the plane
     *                  x = 1 spanning the given Y range, so the wall has a
     *                  measurable extent.
     *
     * @param[in]       p_wall
     *                  Wall plane that receives the point cloud.
     *
     * @param[in]       yMinimum_in
     *                  Smallest Y of the grid, metres.
     *
     * @param[in]       yMaximum_in
     *                  Largest Y of the grid, metres.
     */
    void setWallSpanCloud(vs_graphs::core::geometric::Plane *p_wall,
                          double                             yMinimum_in,
                          double                             yMaximum_in)
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
        ASSERT_EQ(
            (p_wall->setMapClouds(spanCloud)),
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    }
};

// =============================================================================
// TEST CASES
// =============================================================================

/*!
 * @brief           Checks that two maps holding the same two rooms with
 *                  consistent walls and passages are accepted as ALIGNED.
 */
TEST_F(ConsecutiveMapMatcherTest, TC1_matchingPair)
{
    vs_graphs::core::semantic::Room *r0_2 =
        addRoomWithWallsAndPassage(p_map0,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
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
    vs_graphs::core::semantic::Room *r1_2 =
        addRoomWithWallsAndPassage(p_map1,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    g2o::Sim3 identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    vs_graphs::core::semantic::SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    vs_graphs::core::semantic::SemanticMergeGateResult result{};
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::
                   evaluateConsecutiveMergeGate(p_map1,
                                                p_map0,
                                                identity_sim3,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeDecisionName(
                  result.decision,
                  p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision,
              vs_graphs::core::semantic::SemanticMergeDecision::ACCEPT)
        << "TC1: expected ACCEPT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeReasonName(
                  result.reason,
                  p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason,
              vs_graphs::core::semantic::SemanticMergeReason::ALIGNED)
        << "TC1: expected ALIGNED, got " << p_name2;
}

/*!
 * @brief           Checks that rooms with different tags in different places
 *                  are deferred with SHARED_ROOM_IDENTITY_MISSING.
 */
TEST_F(ConsecutiveMapMatcherTest, TC2_aliasing)
{
    vs_graphs::core::semantic::Room *r0_1 =
        new vs_graphs::core::semantic::Room();
    ASSERT_EQ((r0_1->setId(12)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1->setMap(p_map0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1->setRoomVariant(
                  vs_graphs::core::semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1->setCentroid(ROOM_12_CENTROID)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1->setRoomTag("room_12")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map0->addDetectedMapRoom(r0_1)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    vs_graphs::core::semantic::Room *r1_1 =
        new vs_graphs::core::semantic::Room();
    ASSERT_EQ((r1_1->setId(6)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_1->setMap(p_map1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_1->setRoomVariant(
                  vs_graphs::core::semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_1->setCentroid(ROOM_6_CENTROID)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r1_1->setRoomTag("room_6")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map1->addDetectedMapRoom(r1_1)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map0, 0, FLOOR_EQ), nullptr);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    g2o::Sim3 identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    vs_graphs::core::semantic::SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    vs_graphs::core::semantic::SemanticMergeGateResult result{};
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::
                   evaluateConsecutiveMergeGate(p_map1,
                                                p_map0,
                                                identity_sim3,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeDecisionName(
                  result.decision,
                  p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision,
              vs_graphs::core::semantic::SemanticMergeDecision::DEFER)
        << "TC2: expected DEFER, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeReasonName(
                  result.reason,
                  p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason,
              vs_graphs::core::semantic::SemanticMergeReason::
                  SHARED_ROOM_IDENTITY_MISSING)
        << "TC2: expected SHARED_ROOM_IDENTITY_MISSING, got " << p_name2;
}

/*!
 * @brief           Checks that a single shared anchor room does not merge the
 *                  maps: both the old and the current map stay live.
 */
TEST_F(ConsecutiveMapMatcherTest, TC3_singleAnchor)
{
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
                                   1,
                                   "room_1",
                                   ROOM1_CENTROID,
                                   Eigen::Vector3d(1.5, 0.0, 1.0),
                                   11,
                                   1,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_2 =
        new vs_graphs::core::semantic::Room();
    ASSERT_EQ((r0_2->setId(101)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setMap(p_map0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setRoomVariant(
                  vs_graphs::core::semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setCentroid(ROOM2_CENTROID)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map0->addDetectedMapRoom(r0_2)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    ASSERT_EQ((atlas.attemptConsecutiveMergeIfGated()),
              vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
    bool isBad2{};
    ASSERT_EQ((p_map0->isBad(isBad2)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_FALSE(isBad2)
        << "TC3: single anchor must not merge (old map stays live)";
    bool isBad3{};
    ASSERT_EQ((p_map1->isBad(isBad3)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_FALSE(isBad3)
        << "TC3: single anchor must not merge (current map stays live)";
}

/*!
 * @brief           Checks that two maps holding only a bare bootstrap room
 *                  each, with no walls or passages, make the merge gate defer.
 */
TEST_F(ConsecutiveMapMatcherTest, TC4_emptyNewMap)
{
    vs_graphs::core::semantic::Room *bootstrap =
        new vs_graphs::core::semantic::Room();
    ASSERT_EQ((bootstrap->setId(0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap->setMap(p_map0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap->setRoomVariant(
                  vs_graphs::core::semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap->setCentroid(Eigen::Vector3d(0.0, 0.0, 0.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map0->addDetectedMapRoom(bootstrap)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    vs_graphs::core::semantic::Room *bootstrap1 =
        new vs_graphs::core::semantic::Room();
    ASSERT_EQ((bootstrap1->setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap1->setMap(p_map1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap1->setRoomVariant(
                  vs_graphs::core::semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bootstrap1->setCentroid(Eigen::Vector3d(0.0, 0.0, 0.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map1->addDetectedMapRoom(bootstrap1)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    setSeedRooms(p_map0, p_map1, bootstrap, bootstrap1);
    g2o::Sim3 identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    vs_graphs::core::semantic::SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    vs_graphs::core::semantic::SemanticMergeGateResult result{};
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::
                   evaluateConsecutiveMergeGate(p_map1,
                                                p_map0,
                                                identity_sim3,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeDecisionName(
                  result.decision,
                  p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision,
              vs_graphs::core::semantic::SemanticMergeDecision::DEFER)
        << "TC4: expected DEFER, got " << p_name;
}

/*!
 * @brief           Checks that floors with different plane equations are
 *                  rejected with FLOOR_CONTRADICTION.
 */
TEST_F(ConsecutiveMapMatcherTest, TC5_floorMismatch)
{
    vs_graphs::core::semantic::Room *r0_2 =
        addRoomWithWallsAndPassage(p_map0,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
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
    vs_graphs::core::semantic::Room *r1_2 =
        addRoomWithWallsAndPassage(p_map1,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    vs_graphs::core::semantic::Floor *floor1 = addFloor(p_map1, 0, FLOOR_EQ);
    EXPECT_TRUE((floor1->setPlaneIdentity(Eigen::Vector4d(0.0, 1.0, 0.0, -0.5),
                                          100U,
                                          10U) ==
                 vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    g2o::Sim3 identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    vs_graphs::core::semantic::SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    vs_graphs::core::semantic::SemanticMergeGateResult result{};
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::
                   evaluateConsecutiveMergeGate(p_map1,
                                                p_map0,
                                                identity_sim3,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeDecisionName(
                  result.decision,
                  p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision,
              vs_graphs::core::semantic::SemanticMergeDecision::REJECT)
        << "TC5: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeReasonName(
                  result.reason,
                  p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(
        result.reason,
        vs_graphs::core::semantic::SemanticMergeReason::FLOOR_CONTRADICTION)
        << "TC5: expected FLOOR_CONTRADICTION, got " << p_name2;
}

/*!
 * @brief           Checks that a wall rotated by 10 degrees between the maps is
 *                  rejected with WALL_ALIGNMENT_CONTRADICTION.
 */
TEST_F(ConsecutiveMapMatcherTest, TC6_wallRotated)
{
    vs_graphs::core::semantic::Room *r0_2 =
        addRoomWithWallsAndPassage(p_map0,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
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
    vs_graphs::core::semantic::Room *r1_2 =
        addRoomWithWallsAndPassage(p_map1,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    vs_graphs::core::geometric::Plane *wall1A = r1_1Walls[0];
    double                             theta  = 10.0 * M_PI / 180.0;
    double                             c = cos(theta), s = sin(theta);
    Eigen::Vector3d                    rotated_normal(c, s, 0.0);
    ASSERT_EQ((wall1A->setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(rotated_normal.x(),
                                               rotated_normal.y(),
                                               rotated_normal.z(),
                                               WALL_A_D)))),
              vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wall1A->setCentroid(Eigen::Vector3d(1.0, 0.0, 1.0))),
              vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    g2o::Sim3 identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    vs_graphs::core::semantic::SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    vs_graphs::core::semantic::SemanticMergeGateResult result{};
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::
                   evaluateConsecutiveMergeGate(p_map1,
                                                p_map0,
                                                identity_sim3,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeDecisionName(
                  result.decision,
                  p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision,
              vs_graphs::core::semantic::SemanticMergeDecision::REJECT)
        << "TC6: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeReasonName(
                  result.reason,
                  p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason,
              vs_graphs::core::semantic::SemanticMergeReason::
                  WALL_ALIGNMENT_CONTRADICTION)
        << "TC6: expected WALL_ALIGNMENT_CONTRADICTION, got " << p_name2;
}

/*!
 * @brief           Checks that contradicting passage endpoints between the maps
 *                  are rejected with PASSAGE_ENDPOINT_CONTRADICTION.
 */
TEST_F(ConsecutiveMapMatcherTest, TC7_passageEndpointContradiction)
{
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
                                   1,
                                   "room_1",
                                   ROOM1_CENTROID,
                                   Eigen::Vector3d(1.5, 0.0, 1.0),
                                   11,
                                   1,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_3 =
        new vs_graphs::core::semantic::Room();
    ASSERT_EQ((r0_3->setId(3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_3->setMap(p_map0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_3->setRoomVariant(
                  vs_graphs::core::semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_3->setCentroid(Eigen::Vector3d(7.0, 0.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_3->setRoomTag("room_3")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map0->addDetectedMapRoom(r0_3)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    std::vector<vs_graphs::core::semantic::Passage *> r0_1Passages{};
    ASSERT_EQ((r0_1->getPassages(r0_1Passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1Passages[0]->setProspectiveRoom(r0_3)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    vs_graphs::core::semantic::Room *r1_2 =
        addRoomWithWallsAndPassage(p_map1,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    g2o::Sim3 identity_sim3(Eigen::Matrix3d::Identity(),
                            Eigen::Vector3d::Zero(),
                            1.0);
    vs_graphs::core::semantic::SemanticVerify::MapMergeConfig config;
    config.wall_coplanar_angle_deg   = 5.0;
    config.wall_edge_overlap_m       = 0.50;
    config.passage_match_tolerance_m = 0.20;
    config.floor_match_tolerance_m   = 0.10;
    vs_graphs::core::semantic::SemanticMergeGateResult result{};
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::
                   evaluateConsecutiveMergeGate(p_map1,
                                                p_map0,
                                                identity_sim3,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    const char *p_name = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeDecisionName(
                  result.decision,
                  p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision,
              vs_graphs::core::semantic::SemanticMergeDecision::REJECT)
        << "TC7: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeReasonName(
                  result.reason,
                  p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason,
              vs_graphs::core::semantic::SemanticMergeReason::
                  PASSAGE_ENDPOINT_CONTRADICTION)
        << "TC7: expected PASSAGE_ENDPOINT_CONTRADICTION, got " << p_name2;
}

/*!
 * @brief           Checks that the first gated merge attempt logs a
 *                  consecutive_merge_attempt line; the immediate second attempt
 *                  exercises the cooldown but is not asserted.
 */
TEST_F(ConsecutiveMapMatcherTest, TC8_cooldown)
{
    vs_graphs::core::semantic::Room *r0_2 =
        addRoomWithWallsAndPassage(p_map0,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
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
    vs_graphs::core::semantic::Room *r1_2 =
        addRoomWithWallsAndPassage(p_map1,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    ASSERT_EQ((atlas.attemptConsecutiveMergeIfGated()),
              vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
    std::string output1 = testing::internal::GetCapturedStdout();
    size_t      count1  = 0;
    size_t      pos1    = 0;
    while ((pos1 = output1.find("consecutive_merge_attempt", pos1)) !=
           std::string::npos)
    {
        count1++;
        pos1++;
    }
    testing::internal::CaptureStdout();
    ASSERT_EQ((atlas.attemptConsecutiveMergeIfGated()),
              vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
    std::string output2 = testing::internal::GetCapturedStdout();
    size_t      count2  = 0;
    size_t      pos2    = 0;
    while ((pos2 = output2.find("consecutive_merge_attempt", pos2)) !=
           std::string::npos)
    {
        count2++;
        pos2++;
    }
    EXPECT_GE(count1, 1U)
        << "TC8: first attempt should produce at least 1 line, got " << count1;
    (void)count2;
}

/*!
 * @brief           Checks that, with the cooldown set to 0, a merge attempt
 *                  after the new map gains a wall still logs a
 *                  consecutive_merge_attempt line.
 */
TEST_F(ConsecutiveMapMatcherTest, TC9_changeGate)
{
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
                                   1,
                                   "room_1",
                                   ROOM1_CENTROID,
                                   Eigen::Vector3d(1.5, 0.0, 1.0),
                                   11,
                                   1,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_2 =
        new vs_graphs::core::semantic::Room();
    ASSERT_EQ((r0_2->setId(101)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setMap(p_map0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setRoomVariant(
                  vs_graphs::core::semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_2->setCentroid(ROOM2_CENTROID)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map0->addDetectedMapRoom(r0_2)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    vs_graphs::core::types::SystemParams *p_params = nullptr;
    ASSERT_EQ((vs_graphs::core::types::SystemParams::getParams(p_params)),
              vs_graphs::core::types::SystemParamsStatus::
                  SYSTEM_PARAMS_STATUS_SUCCESS);
    p_params->mapMerge.mergeCooldown_s = 0U;
    testing::internal::CaptureStdout();
    ASSERT_EQ((atlas.attemptConsecutiveMergeIfGated()),
              vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
    std::string output1 = testing::internal::GetCapturedStdout();
    size_t      count1  = 0;
    size_t      pos1    = 0;
    while ((pos1 = output1.find("consecutive_merge_attempt", pos1)) !=
           std::string::npos)
    {
        count1++;
        pos1++;
    }
    vs_graphs::core::semantic::Room               *r1_1_current = nullptr;
    std::vector<vs_graphs::core::semantic::Room *> rooms1{};
    ASSERT_EQ((p_map1->getAllRooms(rooms1)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    for (vs_graphs::core::semantic::Room *r : rooms1)
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
        vs_graphs::core::geometric::Plane *wall4 =
            new vs_graphs::core::geometric::Plane();
        ASSERT_EQ(
            (wall4->setId(4)),
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ(
            (wall4->setMap(p_map1)),
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ(
            (wall4->setPlaneType(
                vs_graphs::core::geometric::Plane::PlaneVariant::WALL)),
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ(
            (wall4->setGlobalEquation(
                g2o::Plane3D(Eigen::Vector4d(WALL_D_NORMAL_X,
                                             WALL_D_NORMAL_Y,
                                             WALL_D_NORMAL_Z,
                                             WALL_D_D)))),
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ(
            (wall4->setCentroid(Eigen::Vector3d(0.0, 1.0, 1.0))),
            vs_graphs::core::geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ((p_map1->addMapPlane(wall4)),
                  vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
        ASSERT_EQ((r1_1_current->setWalls(wall4)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    }
    testing::internal::CaptureStdout();
    ASSERT_EQ((atlas.attemptConsecutiveMergeIfGated()),
              vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
    std::string output2 = testing::internal::GetCapturedStdout();
    size_t      count2  = 0;
    size_t      pos2    = 0;
    while ((pos2 = output2.find("consecutive_merge_attempt", pos2)) !=
           std::string::npos)
    {
        count2++;
        pos2++;
    }
    EXPECT_GE(count2, 1U)
        << "TC9: second attempt should produce at least 1 line, got " << count2;
    EXPECT_GE(count1, 1U)
        << "TC9: first attempt should produce at least 1 line, got " << count1;
    vs_graphs::core::types::SystemParams *p_params2 = nullptr;
    ASSERT_EQ((vs_graphs::core::types::SystemParams::getParams(p_params2)),
              vs_graphs::core::types::SystemParamsStatus::
                  SYSTEM_PARAMS_STATUS_SUCCESS);
    p_params2->mapMerge.mergeCooldown_s = 30U;
}

/*!
 * @brief           Checks that coplanar walls with disjoint extents are
 *                  rejected with WALL_ALIGNMENT_CONTRADICTION.
 */
TEST_F(ConsecutiveMapMatcherTest, TC10a_disjointWallExtentsReject)
{
    vs_graphs::core::semantic::Room *r0_2 =
        addRoomWithWallsAndPassage(p_map0,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
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
    vs_graphs::core::semantic::Room *r1_2 =
        addRoomWithWallsAndPassage(p_map1,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    const vs_graphs::core::semantic::SemanticMergeGateResult result =
        runConsecutiveGate(p_map1, p_map0);
    const char *p_name = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeDecisionName(
                  result.decision,
                  p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision,
              vs_graphs::core::semantic::SemanticMergeDecision::REJECT)
        << "TC10a: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeReasonName(
                  result.reason,
                  p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason,
              vs_graphs::core::semantic::SemanticMergeReason::
                  WALL_ALIGNMENT_CONTRADICTION)
        << "TC10a: expected WALL_ALIGNMENT_CONTRADICTION, got " << p_name2;
}

/*!
 * @brief           Checks that coplanar walls with overlapping extents are
 *                  accepted.
 */
TEST_F(ConsecutiveMapMatcherTest, TC10b_overlappingWallExtentsAccept)
{
    vs_graphs::core::semantic::Room *r0_2 =
        addRoomWithWallsAndPassage(p_map0,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
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
    vs_graphs::core::semantic::Room *r1_2 =
        addRoomWithWallsAndPassage(p_map1,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    const vs_graphs::core::semantic::SemanticMergeGateResult result =
        runConsecutiveGate(p_map1, p_map0);
    const char *p_name = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeDecisionName(
                  result.decision,
                  p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision,
              vs_graphs::core::semantic::SemanticMergeDecision::ACCEPT)
        << "TC10b: expected ACCEPT, got " << p_name;
}

/*!
 * @brief           Checks that a passage that is passable in one map and not in
 *                  the other is rejected with PASSAGE_IDENTITY_CONTRADICTION.
 */
TEST_F(ConsecutiveMapMatcherTest, TC11_passableMismatchRejects)
{
    vs_graphs::core::semantic::Room *r0_2 =
        addRoomWithWallsAndPassage(p_map0,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
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
    vs_graphs::core::semantic::Room *r1_2 =
        addRoomWithWallsAndPassage(p_map1,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    const vs_graphs::core::semantic::SemanticMergeGateResult result =
        runConsecutiveGate(p_map1, p_map0);
    const char *p_name = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeDecisionName(
                  result.decision,
                  p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision,
              vs_graphs::core::semantic::SemanticMergeDecision::REJECT)
        << "TC11: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeReasonName(
                  result.reason,
                  p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason,
              vs_graphs::core::semantic::SemanticMergeReason::
                  PASSAGE_IDENTITY_CONTRADICTION)
        << "TC11: expected PASSAGE_IDENTITY_CONTRADICTION, got " << p_name2;
}

/*!
 * @brief           Checks that a flipped known-side direction of a passage is
 *                  rejected with PASSAGE_DIRECTION_CONTRADICTION.
 */
TEST_F(ConsecutiveMapMatcherTest, TC12_directionFlipRejects)
{
    vs_graphs::core::semantic::Room *r0_2 =
        addRoomWithWallsAndPassage(p_map0,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
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
    vs_graphs::core::semantic::Room *r1_2 =
        addRoomWithWallsAndPassage(p_map1,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
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
    const vs_graphs::core::semantic::SemanticMergeGateResult result =
        runConsecutiveGate(p_map1, p_map0);
    const char *p_name = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeDecisionName(
                  result.decision,
                  p_name)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.decision,
              vs_graphs::core::semantic::SemanticMergeDecision::REJECT)
        << "TC12: expected REJECT, got " << p_name;
    const char *p_name2 = nullptr;
    ASSERT_EQ((vs_graphs::core::semantic::SemanticVerify::mergeReasonName(
                  result.reason,
                  p_name2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.reason,
              vs_graphs::core::semantic::SemanticMergeReason::
                  PASSAGE_DIRECTION_CONTRADICTION)
        << "TC12: expected PASSAGE_DIRECTION_CONTRADICTION, got " << p_name2;
}

/*!
 * @brief           Checks that merging resurfaces the old map's doorway proxy
 *                  with the same id, the transferred aperture and its traversal
 *                  history.
 */
TEST_F(ConsecutiveMapMatcherTest, TC13_proxyReunionOnMerge)
{
    vs_graphs::core::semantic::Room *r0_2 =
        addRoomWithWallsAndPassage(p_map0,
                                   2,
                                   "room_2",
                                   ROOM2_CENTROID,
                                   Eigen::Vector3d(4.5, 0.0, 1.0),
                                   12,
                                   11,
                                   nullptr);
    vs_graphs::core::semantic::Room *r0_1 =
        addRoomWithWallsAndPassage(p_map0,
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
                  vs_graphs::core::semantic::Passage::TraversalDirection::
                      KNOWN_TO_FAR)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    std::vector<vs_graphs::core::semantic::Passage *> r0_1Passages2{};
    ASSERT_EQ((r0_1->getPassages(r0_1Passages2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((r0_1Passages2[0]->addTraversalObservation(
                  vs_graphs::core::semantic::Passage::TraversalDirection::
                      KNOWN_TO_FAR)),
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
    vs_graphs::core::semantic::Room *r1_1 =
        addRoomWithWallsAndPassage(p_map1,
                                   1,
                                   "room_1",
                                   ROOM1_CENTROID,
                                   Eigen::Vector3d(1.5, 0.0, 1.0),
                                   11,
                                   1,
                                   nullptr,
                                   false);
    vs_graphs::core::semantic::Passage *proxy11 =
        new vs_graphs::core::semantic::Passage();
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
    ASSERT_EQ((p_map1->addMapPassage(proxy11)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((r1_1->setDoorways(proxy11)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    vs_graphs::core::semantic::Passage *proxy12 =
        new vs_graphs::core::semantic::Passage();
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
    ASSERT_EQ((p_map1->addMapPassage(proxy12)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((r1_1->setDoorways(proxy12)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_NE(addFloor(p_map1, 0, FLOOR_EQ), nullptr);
    setSeedRooms(p_map0, p_map1, r0_1, r1_1);
    ASSERT_EQ((atlas.attemptConsecutiveMergeIfGated()),
              vs_graphs::core::AtlasStatus::ATLAS_STATUS_SUCCESS);
    bool isBad2{};
    ASSERT_EQ((p_map0->isBad(isBad2)),
              vs_graphs::core::MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_TRUE(isBad2) << "TC13: old map must retire on merge commit";
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
