/**
 * Focused test: proves SemanticsManager::Run()'s new
 * candidate-verification wiring (evaluateTopCandidateVerification) actually
 * drives real Atlas/Map/semantic::Room/geometric::Plane/semantic::Floor objects through
 * semantic::SemanticCandidates -> semantic::SemanticVerify -> submitVerificationVerdict ->
 * semantic::RoomTracker, not just the individual phases in isolation (already covered
 * by test_CandidateGen.cpp / test_GeometricVerify.cpp).
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

/** Same asymmetric 4-wall fixture as test_GeometricVerify.cpp's
 * AcceptsGroundTruthCorrelatedRooms: axis-permutation symmetry broken by
 * distinct non-zero offsets plus one oblique face. */
std::vector<RawWall> makeReferenceWalls()
{
    return {
        {1, Eigen::Vector3d(1.0, 0.0, 0.0), -0.2, Eigen::Vector3d(0.2, 1.5, 0.7)},
        {2, Eigen::Vector3d(0.0, 1.0, 0.0), -3.1, Eigen::Vector3d(1.0, 3.1, 0.7)},
        {3, Eigen::Vector3d(0.0, 0.0, 1.0), -1.6, Eigen::Vector3d(1.0, 1.5, 1.6)},
        {4,
         Eigen::Vector3d(2.0, 1.0, 0.5).normalized(),
         -2.7,
         Eigen::Vector3d(1.1, 0.6, 0.3)},
    };
}

/** Transforms a raw plane equation the same way geometric::Plane::transformPlaneEquation
 * does (scale fixed at 1): n' = R n; d' = d - n'^T t. */
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

/** Owns every geometric::Plane/semantic::Floor object one synthetic room needs, and wires them
 * into a real semantic::Room via the genuine setter API so the production wiring's
 * Atlas/Map lookups (SemanticsManager::findRoomByMapAndId,
 * semantic::SemanticVerify::collectWallObservations, the floor gate) see exactly what
 * a live map would produce. */
struct SyntheticRoomFixture
{
    semantic::Room                                 room;
    semantic::Floor                                floor;
    std::vector<std::unique_ptr<geometric::Plane>> ownedWalls;

    void addWall(const RawWall &wall_in)
    {
        auto wall = std::make_unique<geometric::Plane>();
        wall->setId(wall_in.id);
        wall->setPlaneType(geometric::Plane::planeVariant::WALL);
        wall->setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(wall_in.normal.x(),
                                                             wall_in.normal.y(),
                                                             wall_in.normal.z(),
                                                             wall_in.d)));
        wall->setCentroid(wall_in.centroid);
        room.setWalls(wall.get());
        ownedWalls.push_back(std::move(wall));
    }
};

std::unique_ptr<SyntheticRoomFixture>
    buildRoom(Map                        *p_map_in,
             int                          roomId_in,
             const std::vector<RawWall> &walls_in,
             const Eigen::Vector3d      &centroid_in,
             const Eigen::Vector4d      &floorEquation_World_in,
             int                          floorId_in)
{
    auto fixture = std::make_unique<SyntheticRoomFixture>();
    fixture->room.setId(roomId_in);
    fixture->room.setMap(p_map_in);
    fixture->room.setRoomVariant(semantic::Room::RoomVariant::ROOM);
    fixture->room.setCentroid(centroid_in);
    for (const RawWall &wall : walls_in)
    {
        fixture->addWall(wall);
    }
    for (const std::unique_ptr<geometric::Plane> &p_wall : fixture->ownedWalls)
    {
        p_map_in->AddMapPlane(p_wall.get());
    }
    p_map_in->AddDetectedMapRoom(&fixture->room);

    fixture->floor.setId(floorId_in);
    fixture->floor.setMap(p_map_in);
    fixture->floor.setPlaneIdentity(
        floorEquation_World_in, /*finiteSupportCount_in=*/100U,
        /*observationCount_in=*/5U);
    fixture->room.setFloor(&fixture->floor);
    p_map_in->AddMapFloor(&fixture->floor);

    return fixture;
}

semantic::SemanticCandidate makeCandidate(Map *p_mapA_in,
                                semantic::Room *p_roomA_in,
                                Map  *p_mapB_in,
                                semantic::Room *p_roomB_in)
{
    semantic::SemanticCandidate candidate;
    candidate.mapAId                    = p_mapA_in->GetId();
    candidate.roomAId                   = p_roomA_in->getId();
    candidate.mapBId                    = p_mapB_in->GetId();
    candidate.roomBId                   = p_roomB_in->getId();
    candidate.minimumEvidenceSatisfied = true;
    candidate.ambiguous                = true; // see production comment: the
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
    Map  *p_mapA = atlas.GetCurrentMap();
    atlas.CreateNewMap();
    Map *p_mapB = atlas.GetCurrentMap();
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
    const Eigen::Vector3d floorNormalB = rotationTrue * floorEquationA_World.head<3>();
    const Eigen::Vector4d floorEquationB_World(
        floorNormalB.x(),
        floorNormalB.y(),
        floorNormalB.z(),
        floorEquationA_World.w() - floorNormalB.dot(translationTrue));

    std::unique_ptr<SyntheticRoomFixture> roomA = buildRoom(
        p_mapA, 10, wallsA, centroidA, floorEquationA_World, 100);
    std::unique_ptr<SyntheticRoomFixture> roomB = buildRoom(
        p_mapB, 20, wallsB, centroidB, floorEquationB_World, 200);

    const std::vector<semantic::SemanticCandidate> candidates = {
        makeCandidate(p_mapA, &roomA->room, p_mapB, &roomB->room)};

    SemanticsManager manager(&atlas);
    ASSERT_EQ(manager.getRoomTrackerStateForTest(), semantic::RoomTrackingState::UNKNOWN);

    manager.evaluateTopCandidateVerificationForTest(candidates);
    /* submitVerificationVerdict() only queues the result; semantic::RoomTracker
     * consumes it on the next drained cycle -- confirm the queue, not an
     * instantaneous transition. */
    EXPECT_EQ(manager.getRoomTrackerStateForTest(), semantic::RoomTrackingState::UNKNOWN);

    manager.processRoomTrackerPendingForTest(1.0);

    EXPECT_EQ(manager.getRoomTrackerStateForTest(),
              semantic::RoomTrackingState::CONFIRMED_ROOM);
    const std::vector<semantic::TransitionEvent> &history =
        manager.getRoomTrackerEventHistoryForTest();
    ASSERT_FALSE(history.empty());
    EXPECT_EQ(history.back().event, semantic::RoomTrackingEvent::FIRST_ROOM_CONFIRMED);
    EXPECT_TRUE(history.back().accepted);
    EXPECT_TRUE(history.back().verificationPass);
}

TEST(VerificationWiringIntegration,
     TooFewWallsOnOneSideYieldsRejectedVerdictNoTransition)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.GetCurrentMap();
    atlas.CreateNewMap();
    Map *p_mapB = atlas.GetCurrentMap();

    const std::vector<RawWall> wallsA = makeReferenceWalls();
    /* semantic::SemanticVerify::verify() rejects outright below 3 walls per side
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
    manager.evaluateTopCandidateVerificationForTest(candidates);
    manager.processRoomTrackerPendingForTest(1.0);

    EXPECT_EQ(manager.getRoomTrackerStateForTest(), semantic::RoomTrackingState::UNKNOWN);
    EXPECT_TRUE(manager.getRoomTrackerEventHistoryForTest().empty());
}

TEST(VerificationWiringIntegration, EmptyCandidateListIsANoOp)
{
    Atlas             atlas(0);
    SemanticsManager  manager(&atlas);
    manager.evaluateTopCandidateVerificationForTest({});
    manager.processRoomTrackerPendingForTest(1.0);
    EXPECT_EQ(manager.getRoomTrackerStateForTest(), semantic::RoomTrackingState::UNKNOWN);
    EXPECT_TRUE(manager.getRoomTrackerEventHistoryForTest().empty());
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
    first.mapAId                    = 1U;
    first.roomAId                   = 1;
    first.mapBId                    = 2U;
    first.roomBId                   = 2;
    first.minimumEvidenceSatisfied = true;
    first.ambiguous                = true;

    semantic::SemanticCandidate second               = first;
    second.roomBId                          = 3;
    second.ambiguous                        = true;

    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    manager.evaluateTopCandidateVerificationForTest({first, second});
    manager.processRoomTrackerPendingForTest(1.0);
    EXPECT_EQ(manager.getRoomTrackerStateForTest(), semantic::RoomTrackingState::UNKNOWN);
    EXPECT_TRUE(manager.getRoomTrackerEventHistoryForTest().empty());
}

} // namespace core
} // namespace vs_graphs
