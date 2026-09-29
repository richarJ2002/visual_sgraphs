/*!
 * Focused test: proves SemanticsManager::Run()'s new
 * candidate-verification wiring (evaluateTopCandidateVerification) actually
 * drives real Atlas/Map/semantic::Room/geometric::Plane/semantic::Floor objects
 * through semantic::SemanticCandidates -> semantic::SemanticVerify ->
 * submitVerificationVerdict -> semantic::RoomTracker, not just the individual
 * phases in isolation (already covered by test_CandidateGen.cpp /
 * test_GeometricVerify.cpp).
 */

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "Semantic/Floor.h"
#include "Semantic/Room.h"
#include "Semantic/SemanticCandidates.h"
#include "SemanticsManager.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <memory>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{

struct RawWall
{
    int             id;
    Eigen::Vector3d normal;
    double          d;
    Eigen::Vector3d centroid;
};

/*! Same asymmetric 4-wall fixture as test_GeometricVerify.cpp's
 * AcceptsGroundTruthCorrelatedRooms: axis-permutation symmetry broken by
 * distinct non-zero offsets plus one oblique face. */
std::vector<RawWall> makeReferenceWalls()
{
    return {
        {1,
         Eigen::Vector3d(1.0, 0.0, 0.0),
         -0.2,
         Eigen::Vector3d(0.2, 1.5, 0.7)},
        {2,
         Eigen::Vector3d(0.0, 1.0, 0.0),
         -3.1,
         Eigen::Vector3d(1.0, 3.1, 0.7)},
        {3,
         Eigen::Vector3d(0.0, 0.0, 1.0),
         -1.6,
         Eigen::Vector3d(1.0, 1.5, 1.6)},
        {4,
         Eigen::Vector3d(2.0, 1.0, 0.5).normalized(),
         -2.7,
         Eigen::Vector3d(1.1, 0.6, 0.3)},
    };
}

/*! Transforms a raw plane equation the same way
 * geometric::Plane::transformPlaneEquation does (scale fixed at 1): n' = R n;
 * d' = d - n'^T t. */
RawWall transformWall(const RawWall         &source_in,
                      const Eigen::Matrix3d &rotation_in,
                      const Eigen::Vector3d &translation_in,
                      const int              newId_in)
{
    RawWall transformed;
    transformed.id       = newId_in;
    transformed.normal   = rotation_in * source_in.normal;
    transformed.d        = source_in.d - transformed.normal.dot(translation_in);
    transformed.centroid = rotation_in * source_in.centroid + translation_in;
    return transformed;
}

/*! Owns every geometric::Plane/semantic::Floor object one synthetic room needs,
 * and wires them into a real semantic::Room via the genuine setter API so the
 * production wiring's Atlas/Map lookups (SemanticsManager::findRoomByMapAndId,
 * semantic::SemanticVerify::collectWallObservations, the floor gate) see
 * exactly what a live map would produce. */
struct SyntheticRoomFixture
{
    semantic::Room                                 room;
    semantic::Floor                                floor;
    std::vector<std::unique_ptr<geometric::Plane>> ownedWalls;

    void addWall(const RawWall &wall_in)
    {
        auto wall = std::make_unique<geometric::Plane>();
        ASSERT_EQ((wall->setId(wall_in.id)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ((wall->setPlaneType(geometric::Plane::PlaneVariant::WALL)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ((wall->setGlobalEquation(
                      g2o::Plane3D(Eigen::Vector4d(wall_in.normal.x(),
                                                   wall_in.normal.y(),
                                                   wall_in.normal.z(),
                                                   wall_in.d)))),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ((wall->setCentroid(wall_in.centroid)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ((room.setWalls(wall.get())),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ownedWalls.push_back(std::move(wall));
    }
};

std::unique_ptr<SyntheticRoomFixture>
    buildRoom(Map                        *p_map_in,
              int                         roomId_in,
              const std::vector<RawWall> &walls_in,
              const Eigen::Vector3d      &centroid_in,
              const Eigen::Vector4d      &floorEquation_World_in,
              int                         floorId_in)
{
    auto fixture = std::make_unique<SyntheticRoomFixture>();
    if (fixture->room.setId(roomId_in) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (fixture->room.setMap(p_map_in) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (fixture->room.setRoomVariant(semantic::Room::RoomVariant::ROOM) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setRoomVariant returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (fixture->room.setCentroid(centroid_in) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (const RawWall &wall : walls_in)
    {
        fixture->addWall(wall);
    }
    for (const std::unique_ptr<geometric::Plane> &p_wall : fixture->ownedWalls)
    {
        if (p_map_in->addMapPlane(p_wall.get()) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPlane returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }
    if (p_map_in->addDetectedMapRoom(&fixture->room) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addDetectedMapRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    if (fixture->floor.setId(floorId_in) !=
        vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (fixture->floor.setMap(p_map_in) !=
        vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (fixture->floor.setPlaneIdentity(floorEquation_World_in,
                                        /*finiteSupportCount_in=*/100U,
                                        /*observationCount_in=*/5U) !=
        vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("vs_graphs"),
            "%s: setPlaneIdentity rejected its input; continuing as before.",
            __func__);
    }
    if (fixture->room.setFloor(&fixture->floor) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setFloor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_map_in->addMapFloor(&fixture->floor) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addMapFloor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    return fixture;
}

semantic::SemanticCandidate makeCandidate(Map            *p_mapA_in,
                                          semantic::Room *p_roomA_in,
                                          Map            *p_mapB_in,
                                          semantic::Room *p_roomB_in)
{
    semantic::SemanticCandidate candidate;
    unsigned long               mapAId2{};
    if (p_mapA_in->getId(mapAId2) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    candidate.mapAId = mapAId2;
    int roomA_inId{};
    if (p_roomA_in->getId(roomA_inId) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    candidate.roomAId = roomA_inId;
    unsigned long mapBId2{};
    if (p_mapB_in->getId(mapBId2) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    candidate.mapBId = mapBId2;
    int roomB_inId{};
    if (p_roomB_in->getId(roomB_inId) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    candidate.roomBId                    = roomB_inId;
    candidate.isMinimumEvidenceSatisfied = true;
    candidate.isAmbiguous                = true; // see production comment: the
                                                 // winner's own flag is
                                                 // always true and unused as
                                                 // a gate by itself.
    return candidate;
}

} // namespace

TEST(VerificationWiringIntegration,
     MatchingRoomsAcrossMapsDriveRoomTrackerToConfirmed)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_NE(p_mapA, p_mapB);

    const std::vector<RawWall> wallsA = makeReferenceWalls();
    const Eigen::Vector3d      centroidA(0.5, 0.5, 0.5);
    const Eigen::Vector4d      floorEquationA_World(0.0, 0.0, 1.0, -0.4);

    const Eigen::Matrix3d rotationTrue =
        Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, 0.6, 0.3).normalized())
            .toRotationMatrix();
    const Eigen::Vector3d translationTrue(4.0, -2.0, 1.5);

    std::vector<RawWall> wallsB;
    for (const RawWall &wall : wallsA)
    {
        wallsB.push_back(
            transformWall(wall, rotationTrue, translationTrue, wall.id + 10));
    }
    const Eigen::Vector3d centroidB =
        rotationTrue * centroidA + translationTrue;
    const Eigen::Vector3d floorNormalB =
        rotationTrue * floorEquationA_World.head<3>();
    const Eigen::Vector4d floorEquationB_World(
        floorNormalB.x(),
        floorNormalB.y(),
        floorNormalB.z(),
        floorEquationA_World.w() - floorNormalB.dot(translationTrue));

    std::unique_ptr<SyntheticRoomFixture> roomA =
        buildRoom(p_mapA, 10, wallsA, centroidA, floorEquationA_World, 100);
    std::unique_ptr<SyntheticRoomFixture> roomB =
        buildRoom(p_mapB, 20, wallsB, centroidB, floorEquationB_World, 200);

    const std::vector<semantic::SemanticCandidate> candidates = {
        makeCandidate(p_mapA, &roomA->room, p_mapB, &roomB->room)};

    SemanticsManager            manager(&atlas);
    semantic::RoomTrackingState getRoomTrackerStateForTest2{};
    ASSERT_EQ((manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ(getRoomTrackerStateForTest2,
              semantic::RoomTrackingState::UNKNOWN);

    ASSERT_EQ((manager.evaluateTopCandidateVerificationForTest(candidates)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    /* submitVerificationVerdict() only queues the result; semantic::RoomTracker
     * consumes it on the next drained cycle -- confirm the queue, not an
     * instantaneous transition. */
    semantic::RoomTrackingState getRoomTrackerStateForTest3{};
    ASSERT_EQ((manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest3)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerStateForTest3,
              semantic::RoomTrackingState::UNKNOWN);

    ASSERT_EQ((manager.processRoomTrackerPendingForTest(1.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    semantic::RoomTrackingState getRoomTrackerStateForTest4{};
    ASSERT_EQ((manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest4)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerStateForTest4,
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    const std::vector<semantic::TransitionEvent> *p_historyRef = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(p_historyRef)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    const std::vector<semantic::TransitionEvent> &history = *p_historyRef;
    ASSERT_FALSE(history.empty());
    EXPECT_EQ(history.back().event,
              semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED);
    EXPECT_TRUE(history.back().isAccepted);
    EXPECT_TRUE(history.back().hasVerificationPassed);
}

TEST(VerificationWiringIntegration,
     TooFewWallsOnOneSideYieldsRejectedVerdictNoTransition)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    const std::vector<RawWall> wallsA = makeReferenceWalls();
    /* semantic::SemanticVerify::verify() rejects outright below 3 walls per
     * side
     * ("minimal sample: 3 planes ... for full SE(3)"). */
    const std::vector<RawWall> wallsBTooFew = {wallsA[0], wallsA[1]};

    std::unique_ptr<SyntheticRoomFixture> roomA =
        buildRoom(p_mapA,
                  11,
                  wallsA,
                  Eigen::Vector3d(0.5, 0.5, 0.5),
                  Eigen::Vector4d(0.0, 0.0, 1.0, -0.4),
                  101);
    std::unique_ptr<SyntheticRoomFixture> roomB =
        buildRoom(p_mapB,
                  21,
                  wallsBTooFew,
                  Eigen::Vector3d(0.5, 0.5, 0.5),
                  Eigen::Vector4d(0.0, 0.0, 1.0, -0.4),
                  201);

    const std::vector<semantic::SemanticCandidate> candidates = {
        makeCandidate(p_mapA, &roomA->room, p_mapB, &roomB->room)};

    SemanticsManager manager(&atlas);
    ASSERT_EQ((manager.evaluateTopCandidateVerificationForTest(candidates)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(1.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    semantic::RoomTrackingState getRoomTrackerStateForTest2{};
    ASSERT_EQ((manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerStateForTest2,
              semantic::RoomTrackingState::UNKNOWN);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_TRUE((*p_getRoomTrackerEventHistoryForTest).empty());
}

TEST(VerificationWiringIntegration, EmptyCandidateListIsANoOp)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    ASSERT_EQ((manager.evaluateTopCandidateVerificationForTest({})),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(1.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    semantic::RoomTrackingState getRoomTrackerStateForTest2{};
    ASSERT_EQ((manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerStateForTest2,
              semantic::RoomTrackingState::UNKNOWN);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_TRUE((*p_getRoomTrackerEventHistoryForTest).empty());
}

TEST(VerificationWiringIntegration, GenuineTiedLeaderIsSkipped)
{
    /* generateWithStatus() marks the winner's own `ambiguous` field true
     * unconditionally; a real tie is only distinguishable by a SECOND
     * candidate also carrying `ambiguous == true`. Neither candidate here
     * resolves to a real room -- if the gate were wrong (e.g. naively
     * reading candidates[0].ambiguous, which is always true) this would
     * either wrongly skip every candidate ever, or wrongly attempt a room
     * lookup here; either way the tracker must stay UNKNOWN with no crash. */
    semantic::SemanticCandidate first;
    first.mapAId                     = 1U;
    first.roomAId                    = 1;
    first.mapBId                     = 2U;
    first.roomBId                    = 2;
    first.isMinimumEvidenceSatisfied = true;
    first.isAmbiguous                = true;

    semantic::SemanticCandidate second = first;
    second.roomBId                     = 3;
    second.isAmbiguous                 = true;

    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    ASSERT_EQ(
        (manager.evaluateTopCandidateVerificationForTest({first, second})),
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.processRoomTrackerPendingForTest(1.0)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    semantic::RoomTrackingState getRoomTrackerStateForTest2{};
    ASSERT_EQ((manager.getRoomTrackerStateForTest(getRoomTrackerStateForTest2)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_EQ(getRoomTrackerStateForTest2,
              semantic::RoomTrackingState::UNKNOWN);
    const std::vector<semantic::TransitionEvent>
        *p_getRoomTrackerEventHistoryForTest = nullptr;
    ASSERT_EQ((manager.getRoomTrackerEventHistoryForTest(
                  p_getRoomTrackerEventHistoryForTest)),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    EXPECT_TRUE((*p_getRoomTrackerEventHistoryForTest).empty());
}

} // namespace core
} // namespace vs_graphs
