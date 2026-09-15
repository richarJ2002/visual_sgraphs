/**
 * Semantic-axiom-reliability-plan.md Phase 1 (P1.1): focused, ROS/Gazebo-free
 * tests for the value-only SemanticGraphSnapshot capture contract. These
 * exercise the genuine production capture entry point
 * (vs_graphs::core::semantic::captureSemanticGraphSnapshot()) against real
 * Atlas/Map/Room/geometric::Plane/Passage/Floor objects built through SemanticFixtures
 * and the model's own setters -- they do not reconstruct the expected
 * snapshot by hand, except where a test white-box-verifies one internal
 * helper directly (documented at each such case).
 *
 * Each TEST below is annotated with the minimum-proof item from
 * claude-resume-phase1-snapshot-repair-prompt.md or the 2026-09-06 residual
 * audit's MUST-CLOSE list that it satisfies.
 */

#include "Semantic/SemanticGraphSnapshot.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "Atlas.h"
#include "AtlasCurrentMapStatus.h"
#include "Geometric/Plane.h"
#include "Map.h"
#include "Semantic/Floor.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "Semantic/SemanticGraphSnapshot/private_functions.h"
#include "SemanticFixtures.h"
#include "SemanticGraphSnapshotTestHelpers.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/* Minimum-proof item 1: the result contains no model pointers/clouds and
 * remains valid after fixture model objects leave scope. */
TEST(SemanticGraphSnapshot, RemainsValidAfterFixtureModelObjectsLeaveScope)
{
    std::optional<SemanticGraphSnapshot> capturedSnapshot;
    long unsigned int                    mapId = 0U;
    {
        Atlas atlas(0);
        Map  *p_map = atlas.GetCurrentMap();
        mapId       = p_map->GetId();

        geometric::Plane wall;
        test::makeWallPlane(wall,
                            1,
                            p_map,
                            Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                            Eigen::Vector3d::UnitY(),
                            Eigen::Vector3d::UnitZ(),
                            1.0,
                            1.0,
                            Eigen::Vector3d(0.0, 0.0, 1.0));
        p_map->AddMapPlane(&wall);

        Room room;
        test::makeRoom(room, 1, p_map, &wall, Eigen::Vector3d(1.0, 0.0, 1.0));
        p_map->AddDetectedMapRoom(&room);

        std::unique_lock<std::mutex> lock = atlas.acquireSemanticUpdateLock();
        capturedSnapshot = captureSemanticGraphSnapshot(&atlas);
        /* wall, room, and atlas are all destroyed here, at scope exit. */
    }

    ASSERT_TRUE(capturedSnapshot.has_value());
    const MapSnapshot *p_mapSnapshot =
        findMapSnapshot(*capturedSnapshot, mapId);
    ASSERT_NE(p_mapSnapshot, nullptr);
    ASSERT_EQ(p_mapSnapshot->rooms.size(), 1U);
    ASSERT_EQ(p_mapSnapshot->walls.size(), 1U);
    EXPECT_EQ(p_mapSnapshot->rooms[0].centroid_World_m,
              Eigen::Vector3d(1.0, 0.0, 1.0));
    EXPECT_EQ(p_mapSnapshot->walls[0].planeType, geometric::Plane::planeVariant::WALL);
    /* updateSizeOfPlane() (called by makeWallPlane()) populates the finite
     * U/V bounds; finiteSupportCount is populated only by the separate
     * beginMapCloudRefit()/completeMapCloudRefit() pipeline, which this
     * fixture does not exercise, so it is correctly captured as 0 here. */
    EXPECT_GT(p_mapSnapshot->walls[0].maxPlaneU_m,
              p_mapSnapshot->walls[0].minPlaneU_m);
    EXPECT_EQ(p_mapSnapshot->walls[0].finiteSupportCount, 0U);
}

/* MUST-CLOSE 6 "value lifetime after source objects are destroyed": extend
 * lifetime coverage beyond the whole-snapshot scope-exit case above to the
 * EntityRef/RawPlaneRef sub-values inside relationship collections
 * specifically (RoomRecord::wallRefs/passageRefs, WallRecord::ownerRoomRefs,
 * FloorRecord::roomRefs), which are the types this residual repair changed
 * from bare EntityKey. */
TEST(SemanticGraphSnapshot,
     RelationshipSubValuesRemainValidAfterFixtureModelObjectsLeaveScope)
{
    std::optional<SemanticGraphSnapshot> capturedSnapshot;
    long unsigned int                    mapId = 0U;
    {
        Atlas atlas(0);
        Map  *p_map = atlas.GetCurrentMap();
        mapId       = p_map->GetId();

        geometric::Plane wall;
        test::makeWallPlane(wall,
                            1,
                            p_map,
                            Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                            Eigen::Vector3d::UnitY(),
                            Eigen::Vector3d::UnitZ(),
                            1.0,
                            1.0,
                            Eigen::Vector3d(0.0, 0.0, 1.0));
        p_map->AddMapPlane(&wall);

        Room room;
        test::makeRoom(room, 2, p_map, &wall, Eigen::Vector3d(1.0, 0.0, 1.0));
        p_map->AddDetectedMapRoom(&room);

        Passage passage;
        passage.setId(3);
        passage.setMap(p_map);
        passage.setKnownSideRoom(&room);
        p_map->AddMapPassage(&passage);
        room.setDoorways(&passage);

        Floor floor;
        test::makeFloor(floor, 4, p_map, {&room});
        p_map->AddMapFloor(&floor);
        room.setFloor(&floor);

        std::unique_lock<std::mutex> lock = atlas.acquireSemanticUpdateLock();
        capturedSnapshot = captureSemanticGraphSnapshot(&atlas);
        /* wall, room, passage, floor, and atlas are all destroyed here. */
    }

    ASSERT_TRUE(capturedSnapshot.has_value());
    const MapSnapshot *p_mapSnapshot =
        findMapSnapshot(*capturedSnapshot, mapId);
    ASSERT_NE(p_mapSnapshot, nullptr);

    const RoomRecord *p_roomRecord = findRoomRecord(*p_mapSnapshot, 2);
    ASSERT_NE(p_roomRecord, nullptr);
    ASSERT_EQ(p_roomRecord->wallRefs.size(), 1U);
    EXPECT_EQ(p_roomRecord->wallRefs[0].planeId, 1);
    ASSERT_EQ(p_roomRecord->passageRefs.size(), 1U);
    ASSERT_TRUE(p_roomRecord->passageRefs[0].key.has_value());
    EXPECT_EQ(p_roomRecord->passageRefs[0].key->entityId, 3);
    ASSERT_TRUE(p_roomRecord->floorRef.key.has_value());
    EXPECT_EQ(p_roomRecord->floorRef.key->entityId, 4);

    const WallRecord *p_wallRecord = findWallRecord(*p_mapSnapshot, 1);
    ASSERT_NE(p_wallRecord, nullptr);
    ASSERT_EQ(p_wallRecord->ownerRoomRefs.size(), 1U);
    ASSERT_TRUE(p_wallRecord->ownerRoomRefs[0].key.has_value());
    EXPECT_EQ(p_wallRecord->ownerRoomRefs[0].key->entityId, 2);

    ASSERT_EQ(p_mapSnapshot->floors.size(), 1U);
    ASSERT_EQ(p_mapSnapshot->floors[0].roomRefs.size(), 1U);
    ASSERT_TRUE(p_mapSnapshot->floors[0].roomRefs[0].key.has_value());
    EXPECT_EQ(p_mapSnapshot->floors[0].roomRefs[0].key->entityId, 2);
}

/* Minimum-proof item 2: equal room/wall/passage/floor local IDs in two maps
 * remain distinct. */
TEST(SemanticGraphSnapshot, EqualLocalIdsInTwoMapsRemainDistinct)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.GetCurrentMap();
    atlas.CreateNewMap();
    Map *p_mapB = atlas.GetCurrentMap();
    ASSERT_NE(p_mapA->GetId(), p_mapB->GetId());

    geometric::Plane wallA;
    test::makeWallPlane(wallA,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(0.0, 0.0, 1.0));
    p_mapA->AddMapPlane(&wallA);
    Room roomA;
    test::makeRoom(roomA, 1, p_mapA, &wallA);
    p_mapA->AddDetectedMapRoom(&roomA);

    geometric::Plane wallB;
    test::makeWallPlane(wallB,
                        1,
                        p_mapB,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(5.0, 0.0, 1.0));
    p_mapB->AddMapPlane(&wallB);
    Room roomB;
    test::makeRoom(roomB, 1, p_mapB, &wallB);
    p_mapB->AddDetectedMapRoom(&roomB);

    Passage passageA;
    passageA.setId(1);
    passageA.setMap(p_mapA);
    p_mapA->AddMapPassage(&passageA);
    Passage passageB;
    passageB.setId(1);
    passageB.setMap(p_mapB);
    p_mapB->AddMapPassage(&passageB);

    Floor floorA;
    floorA.setId(1);
    floorA.setMap(p_mapA);
    p_mapA->AddMapFloor(&floorA);
    Floor floorB;
    floorB.setId(1);
    floorB.setMap(p_mapB);
    p_mapB->AddMapFloor(&floorB);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    ASSERT_EQ(snapshot.maps.size(), 2U);
    const MapSnapshot *p_snapshotA = findMapSnapshot(snapshot, p_mapA->GetId());
    const MapSnapshot *p_snapshotB = findMapSnapshot(snapshot, p_mapB->GetId());
    ASSERT_NE(p_snapshotA, nullptr);
    ASSERT_NE(p_snapshotB, nullptr);

    ASSERT_EQ(p_snapshotA->rooms.size(), 1U);
    ASSERT_EQ(p_snapshotB->rooms.size(), 1U);
    EXPECT_EQ(p_snapshotA->rooms[0].key.entityId, 1);
    EXPECT_EQ(p_snapshotB->rooms[0].key.entityId, 1);
    EXPECT_NE(p_snapshotA->rooms[0].key, p_snapshotB->rooms[0].key);
    EXPECT_NE(p_snapshotA->rooms[0].key.mapId, p_snapshotB->rooms[0].key.mapId);

    ASSERT_EQ(p_snapshotA->walls.size(), 1U);
    ASSERT_EQ(p_snapshotB->walls.size(), 1U);
    EXPECT_NE(p_snapshotA->walls[0].key, p_snapshotB->walls[0].key);

    ASSERT_EQ(p_snapshotA->passages.size(), 1U);
    ASSERT_EQ(p_snapshotB->passages.size(), 1U);
    EXPECT_EQ(p_snapshotA->passages[0].key.entityId, 1);
    EXPECT_EQ(p_snapshotB->passages[0].key.entityId, 1);
    EXPECT_NE(p_snapshotA->passages[0].key, p_snapshotB->passages[0].key);

    ASSERT_EQ(p_snapshotA->floors.size(), 1U);
    ASSERT_EQ(p_snapshotB->floors.size(), 1U);
    EXPECT_EQ(p_snapshotA->floors[0].key.entityId, 1);
    EXPECT_EQ(p_snapshotB->floors[0].key.entityId, 1);
    EXPECT_NE(p_snapshotA->floors[0].key, p_snapshotB->floors[0].key);
}

/* Minimum-proof item 3, extended to full 4-kind x {mismatch, null} coverage
 * (MUST-CLOSE 6 "both null declared-map and containing/declared-map
 * mismatch for every kind"): containing-map versus declared-map mismatch
 * and null declared map are preserved explicitly for every entity kind. */
TEST(SemanticGraphSnapshot,
     ContainingMapVersusDeclaredMapIsPreservedForEveryEntityKind)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.GetCurrentMap();
    atlas.CreateNewMap();
    Map *p_mapB = atlas.GetCurrentMap();

    /* Room: enumerated from mapA, but declares mapB. */
    Room mismatchedRoom;
    mismatchedRoom.setId(1);
    mismatchedRoom.setMap(p_mapB);
    p_mapA->AddDetectedMapRoom(&mismatchedRoom);

    /* Room: enumerated from mapA, declares no map at all. */
    Room noMapRoom;
    noMapRoom.setId(2);
    p_mapA->AddDetectedMapRoom(&noMapRoom);

    /* Wall: enumerated from mapA, but declares mapB. */
    geometric::Plane mismatchedWall;
    test::makeWallPlane(mismatchedWall,
                        3,
                        p_mapB,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(0.0, 0.0, 1.0));
    p_mapA->AddMapPlane(&mismatchedWall);

    /* Wall: enumerated from mapA, declares no map at all. Map::AddMapPlane()
     * only requires a non-null pointer; the plane's own SetMap() is
     * independent (confirmed by direct source read of Map.h/geometric::Plane.h). */
    geometric::Plane noMapWall;
    test::makeWallPlane(noMapWall,
                        6,
                        nullptr,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(2.0, 0.0, 1.0));
    p_mapA->AddMapPlane(&noMapWall);

    /* Passage: enumerated from mapA, declares no map at all. */
    Passage noMapPassage;
    noMapPassage.setId(4);
    p_mapA->AddMapPassage(&noMapPassage);

    /* Passage: enumerated from mapA, but declares mapB. */
    Passage mismatchedPassage;
    mismatchedPassage.setId(7);
    mismatchedPassage.setMap(p_mapB);
    p_mapA->AddMapPassage(&mismatchedPassage);

    /* Floor: enumerated from mapA, but declares mapB. */
    Floor mismatchedFloor;
    mismatchedFloor.setId(5);
    mismatchedFloor.setMap(p_mapB);
    p_mapA->AddMapFloor(&mismatchedFloor);

    /* Floor: enumerated from mapA, declares no map at all. */
    Floor noMapFloor;
    noMapFloor.setId(8);
    p_mapA->AddMapFloor(&noMapFloor);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    const MapSnapshot *p_snapshotA = findMapSnapshot(snapshot, p_mapA->GetId());
    ASSERT_NE(p_snapshotA, nullptr);

    const RoomRecord *p_mismatchedRoom = findRoomRecord(*p_snapshotA, 1);
    ASSERT_NE(p_mismatchedRoom, nullptr);
    EXPECT_EQ(p_mismatchedRoom->key.mapId, p_mapA->GetId());
    ASSERT_TRUE(p_mismatchedRoom->declaredMapId.has_value());
    EXPECT_EQ(*p_mismatchedRoom->declaredMapId, p_mapB->GetId());

    const RoomRecord *p_noMapRoom = findRoomRecord(*p_snapshotA, 2);
    ASSERT_NE(p_noMapRoom, nullptr);
    EXPECT_FALSE(p_noMapRoom->declaredMapId.has_value());

    const WallRecord *p_mismatchedWallRecord = findWallRecord(*p_snapshotA, 3);
    ASSERT_NE(p_mismatchedWallRecord, nullptr);
    EXPECT_EQ(p_mismatchedWallRecord->key.mapId, p_mapA->GetId());
    ASSERT_TRUE(p_mismatchedWallRecord->declaredMapId.has_value());
    EXPECT_EQ(*p_mismatchedWallRecord->declaredMapId, p_mapB->GetId());

    const WallRecord *p_noMapWallRecord = findWallRecord(*p_snapshotA, 6);
    ASSERT_NE(p_noMapWallRecord, nullptr);
    EXPECT_FALSE(p_noMapWallRecord->declaredMapId.has_value());

    const PassageRecord *p_noMapPassage = findPassageRecord(*p_snapshotA, 4);
    ASSERT_NE(p_noMapPassage, nullptr);
    EXPECT_FALSE(p_noMapPassage->declaredMapId.has_value());

    const PassageRecord *p_mismatchedPassage =
        findPassageRecord(*p_snapshotA, 7);
    ASSERT_NE(p_mismatchedPassage, nullptr);
    EXPECT_EQ(p_mismatchedPassage->key.mapId, p_mapA->GetId());
    ASSERT_TRUE(p_mismatchedPassage->declaredMapId.has_value());
    EXPECT_EQ(*p_mismatchedPassage->declaredMapId, p_mapB->GetId());

    ASSERT_EQ(p_snapshotA->floors.size(), 2U);
    const FloorRecord *p_mismatchedFloor  = nullptr;
    const FloorRecord *p_noMapFloorRecord = nullptr;
    for (const FloorRecord &floorRecord : p_snapshotA->floors)
    {
        if (floorRecord.key.entityId == 5)
        {
            p_mismatchedFloor = &floorRecord;
        }
        else if (floorRecord.key.entityId == 8)
        {
            p_noMapFloorRecord = &floorRecord;
        }
    }
    ASSERT_NE(p_mismatchedFloor, nullptr);
    EXPECT_EQ(p_mismatchedFloor->key.mapId, p_mapA->GetId());
    ASSERT_TRUE(p_mismatchedFloor->declaredMapId.has_value());
    EXPECT_EQ(*p_mismatchedFloor->declaredMapId, p_mapB->GetId());
    ASSERT_NE(p_noMapFloorRecord, nullptr);
    EXPECT_FALSE(p_noMapFloorRecord->declaredMapId.has_value());
}

/* Minimum-proof item 4: detected-only, candidate-only, and deliberately
 * both-collection room membership remain distinguishable. */
TEST(SemanticGraphSnapshot,
     DetectedCandidateAndBothCollectionMembershipAreDistinguishable)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    Room detectedOnly;
    detectedOnly.setId(1);
    detectedOnly.setMap(p_map);
    p_map->AddDetectedMapRoom(&detectedOnly);

    Room candidateOnly;
    candidateOnly.setId(2);
    candidateOnly.setMap(p_map);
    p_map->AddCandidateMapRoom(&candidateOnly);

    Room bothCollections;
    bothCollections.setId(3);
    bothCollections.setMap(p_map);
    p_map->AddDetectedMapRoom(&bothCollections);
    p_map->AddCandidateMapRoom(&bothCollections);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    const MapSnapshot *p_mapSnapshot =
        findMapSnapshot(snapshot, p_map->GetId());
    ASSERT_NE(p_mapSnapshot, nullptr);
    ASSERT_EQ(p_mapSnapshot->rooms.size(), 3U);

    const RoomRecord *p_detectedOnly = findRoomRecord(*p_mapSnapshot, 1);
    ASSERT_NE(p_detectedOnly, nullptr);
    EXPECT_TRUE(p_detectedOnly->isDetectedMember);
    EXPECT_FALSE(p_detectedOnly->isMarkerBasedMember);

    const RoomRecord *p_candidateOnly = findRoomRecord(*p_mapSnapshot, 2);
    ASSERT_NE(p_candidateOnly, nullptr);
    EXPECT_FALSE(p_candidateOnly->isDetectedMember);
    EXPECT_TRUE(p_candidateOnly->isMarkerBasedMember);

    const RoomRecord *p_bothCollections = findRoomRecord(*p_mapSnapshot, 3);
    ASSERT_NE(p_bothCollections, nullptr);
    EXPECT_TRUE(p_bothCollections->isDetectedMember);
    EXPECT_TRUE(p_bothCollections->isMarkerBasedMember);
}

/* Minimum-proof item 5: null, unmapped, bad, missing-from-enumeration,
 * cross-map, and wrong geometric::Plane-type relationship targets retain truthful
 * key/reason/liveness/type evidence. */
TEST(SemanticGraphSnapshot,
     RelationshipTargetsRetainTruthfulEvidenceAcrossEveryUnusualCase)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.GetCurrentMap();
    atlas.CreateNewMap();
    Map *p_mapB = atlas.GetCurrentMap();

    /* Room with no ground plane at all -- null RawPlaneRef. */
    Room noGroundRoom;
    test::makeRoom(noGroundRoom, 1, p_mapA, nullptr);
    p_mapA->AddDetectedMapRoom(&noGroundRoom);

    /* Room whose ground plane is a real, but never map-registered
     * ("unmapped"), WALL-typed ("wrong type") plane. */
    geometric::Plane unmappedWrongTypeGround;
    test::makeWallPlane(unmappedWrongTypeGround,
                        2,
                        nullptr,
                        Eigen::Vector4d(0.0, 0.0, 1.0, 0.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitY(),
                        0.5,
                        0.5,
                        Eigen::Vector3d::Zero());
    Room unusualGroundRoom;
    unusualGroundRoom.setId(3);
    unusualGroundRoom.setMap(p_mapA);
    unusualGroundRoom.setGroundPlane(&unmappedWrongTypeGround);
    p_mapA->AddDetectedMapRoom(&unusualGroundRoom);

    /* Room referenced as a "missing-from-enumeration" far-side target: it
     * has a real map but was never added to any Map room collection. */
    Room ghostRoom;
    ghostRoom.setId(99);
    ghostRoom.setMap(p_mapA);

    /* Room in mapB, referenced cross-map from a Passage enumerated under
     * mapA. */
    Room crossMapRoom;
    crossMapRoom.setId(7);
    crossMapRoom.setMap(p_mapB);
    p_mapB->AddDetectedMapRoom(&crossMapRoom);

    /* Bad wall, referenced as another room's ground plane, to prove
     * liveness is captured truthfully even for a retired target. */
    geometric::Plane badGroundPlane;
    test::makeGroundPlane(badGroundPlane, 8, p_mapA);
    p_mapA->AddMapPlane(&badGroundPlane);
    badGroundPlane.setBad();
    Room badGroundOwnerRoom;
    badGroundOwnerRoom.setId(9);
    badGroundOwnerRoom.setMap(p_mapA);
    badGroundOwnerRoom.setGroundPlane(&badGroundPlane);
    p_mapA->AddDetectedMapRoom(&badGroundOwnerRoom);

    Passage passage;
    passage.setId(10);
    passage.setMap(p_mapA);
    passage.setKnownSideRoom(&ghostRoom);
    passage.setProspectiveRoom(&crossMapRoom);
    p_mapA->AddMapPassage(&passage);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    const MapSnapshot *p_snapshotA = findMapSnapshot(snapshot, p_mapA->GetId());
    ASSERT_NE(p_snapshotA, nullptr);

    /* Null. */
    const RoomRecord *p_noGroundRoom = findRoomRecord(*p_snapshotA, 1);
    ASSERT_NE(p_noGroundRoom, nullptr);
    EXPECT_EQ(p_noGroundRoom->groundPlaneRef.reason,
              UnavailableReason::NULL_REFERENCE);

    /* Unmapped + wrong type. */
    const RoomRecord *p_unusualGroundRoom = findRoomRecord(*p_snapshotA, 3);
    ASSERT_NE(p_unusualGroundRoom, nullptr);
    EXPECT_EQ(p_unusualGroundRoom->groundPlaneRef.reason,
              UnavailableReason::NONE);
    EXPECT_FALSE(p_unusualGroundRoom->groundPlaneRef.mapId.has_value());
    EXPECT_EQ(p_unusualGroundRoom->groundPlaneRef.planeType,
              geometric::Plane::planeVariant::WALL);
    EXPECT_TRUE(p_unusualGroundRoom->groundPlaneRef.isLive);

    /* Missing-from-enumeration: the key is still truthfully reported, even
     * though no RoomRecord for it exists in the snapshot -- and, since the
     * residual repair, so is its liveness (captured directly from the
     * pointer, not by joining against the snapshot's own record vectors). */
    const PassageRecord *p_passageRecord = findPassageRecord(*p_snapshotA, 10);
    ASSERT_NE(p_passageRecord, nullptr);
    ASSERT_TRUE(p_passageRecord->knownSideRoomRef.key.has_value());
    EXPECT_EQ(p_passageRecord->knownSideRoomRef.key->entityId, 99);
    EXPECT_EQ(p_passageRecord->knownSideRoomRef.key->mapId, p_mapA->GetId());
    EXPECT_EQ(findRoomRecord(*p_snapshotA, 99), nullptr);
    ASSERT_TRUE(p_passageRecord->knownSideRoomRef.isLive.has_value());
    EXPECT_TRUE(*p_passageRecord->knownSideRoomRef.isLive);

    /* Cross-map: the referenced room's key names mapB, distinct from the
     * passage record's own mapA key. */
    ASSERT_TRUE(p_passageRecord->prospectiveRoomRef.key.has_value());
    EXPECT_EQ(p_passageRecord->prospectiveRoomRef.key->mapId, p_mapB->GetId());
    EXPECT_NE(p_passageRecord->prospectiveRoomRef.key->mapId,
              p_passageRecord->key.mapId);

    /* Bad target retains liveness truthfully rather than being hidden. */
    const RoomRecord *p_badGroundOwnerRoom = findRoomRecord(*p_snapshotA, 9);
    ASSERT_NE(p_badGroundOwnerRoom, nullptr);
    EXPECT_EQ(p_badGroundOwnerRoom->groundPlaneRef.reason,
              UnavailableReason::NONE);
    EXPECT_FALSE(p_badGroundOwnerRoom->groundPlaneRef.isLive);
}

/* Minimum-proof item 6, extended for the tri-state EntityRef liveness
 * introduced by the 2026-09-06 residual repair: default reference
 * invariants are valid, and unknown liveness is never encoded as true. */
TEST(SemanticGraphSnapshot, DefaultReferenceInvariantsAreValid)
{
    const EntityRef defaultEntityRef;
    EXPECT_FALSE(defaultEntityRef.key.has_value());
    EXPECT_EQ(defaultEntityRef.reason, UnavailableReason::NULL_REFERENCE);
    EXPECT_NE(defaultEntityRef.reason, UnavailableReason::NONE);
    EXPECT_FALSE(defaultEntityRef.localId.has_value());
    EXPECT_FALSE(defaultEntityRef.isLive.has_value())
        << "unknown liveness must never default to a value, let alone true";
    EXPECT_EQ(defaultEntityRef.livenessUnavailableReason,
              UnavailableReason::NULL_REFERENCE);

    const RawPlaneRef defaultRawPlaneRef;
    EXPECT_EQ(defaultRawPlaneRef.reason, UnavailableReason::NULL_REFERENCE);
    EXPECT_NE(defaultRawPlaneRef.reason, UnavailableReason::NONE);
    EXPECT_FALSE(defaultRawPlaneRef.mapId.has_value());
    EXPECT_FALSE(defaultRawPlaneRef.wallKey.has_value());
}

/* Corrective-audit (2026-09-05) regression coverage: entityRefForWall() used
 * to label every referenced geometric::Plane as EntityKind::WALL without checking its
 * real geometric::Plane::planeVariant, fabricating a WallRecord identity for a
 * non-WALL target. Wall-shaped references (a wall's twin face, a Room's
 * owned walls, a Passage's associated walls) now always use RawPlaneRef, so
 * a wrong-type target retains its true planeType/isLive/mapId instead of a
 * fabricated WALL key. */
TEST(SemanticGraphSnapshot,
     WrongTypePlaneTargetsInWallShapedReferencesRetainTruthfulEvidence)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    /* A live GROUND-typed plane, never a WallRecord in this snapshot. */
    geometric::Plane groundNotWall;
    ASSERT_TRUE(test::makeGroundPlane(groundNotWall, 1, p_map));
    p_map->AddMapPlane(&groundNotWall);

    /* Case 1: a genuine wall whose twinFace_ is wrongly set to the
     * GROUND-typed plane (geometric::Plane::setTwinFace() has no type check, so this
     * is a real reachable model state, not a fabricated test-only shape). */
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        2,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(0.0, 0.0, 1.0));
    p_map->AddMapPlane(&wall);
    wall.setTwinFace(&groundNotWall);

    /* Case 2: a Room whose getWalls() names the GROUND-typed plane (e.g. a
     * wall-detection contract violation upstream; nothing in Room::setWalls()
     * enforces WALL-typed membership). */
    Room room;
    room.setId(3);
    room.setMap(p_map);
    room.setWalls(&groundNotWall);
    p_map->AddDetectedMapRoom(&room);

    /* Case 3: a Passage whose getAssociateWalls() names the same
     * GROUND-typed plane. */
    Passage passage;
    passage.setId(4);
    passage.setMap(p_map);
    passage.addAssociateWall(&groundNotWall);
    p_map->AddMapPassage(&passage);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    const MapSnapshot *p_mapSnapshot =
        findMapSnapshot(snapshot, p_map->GetId());
    ASSERT_NE(p_mapSnapshot, nullptr);

    /* The GROUND-typed plane must never appear as a WallRecord. */
    EXPECT_EQ(findWallRecord(*p_mapSnapshot, 1), nullptr);

    const WallRecord *p_wallRecord = findWallRecord(*p_mapSnapshot, 2);
    ASSERT_NE(p_wallRecord, nullptr);
    EXPECT_EQ(p_wallRecord->twinRef.reason, UnavailableReason::NONE);
    EXPECT_EQ(p_wallRecord->twinRef.planeId, 1);
    EXPECT_TRUE(p_wallRecord->twinRef.isLive);
    EXPECT_EQ(p_wallRecord->twinRef.planeType, geometric::Plane::planeVariant::GROUND);
    EXPECT_FALSE(p_wallRecord->twinRef.wallKey.has_value());

    const RoomRecord *p_roomRecord = findRoomRecord(*p_mapSnapshot, 3);
    ASSERT_NE(p_roomRecord, nullptr);
    ASSERT_EQ(p_roomRecord->wallRefs.size(), 1U);
    EXPECT_EQ(p_roomRecord->wallRefs[0].planeId, 1);
    EXPECT_EQ(p_roomRecord->wallRefs[0].planeType, geometric::Plane::planeVariant::GROUND);
    EXPECT_FALSE(p_roomRecord->wallRefs[0].wallKey.has_value());

    const PassageRecord *p_passageRecord = findPassageRecord(*p_mapSnapshot, 4);
    ASSERT_NE(p_passageRecord, nullptr);
    ASSERT_EQ(p_passageRecord->associateWallRefs.size(), 1U);
    EXPECT_EQ(p_passageRecord->associateWallRefs[0].planeId, 1);
    EXPECT_EQ(p_passageRecord->associateWallRefs[0].planeType,
              geometric::Plane::planeVariant::GROUND);
    EXPECT_FALSE(p_passageRecord->associateWallRefs[0].wallKey.has_value());
}

/* Corrective-audit (2026-09-05) regression coverage, extended 2026-09-06 for
 * tri-state liveness: entityRefForRoom()/entityRefForFloor() used to discard
 * the target's own local id (and, for Room, liveness) the moment it had no
 * declared map. The residual repair further requires that Floor's liveness,
 * which the model cannot expose at all (Floor has no isBad()), is reported
 * as explicitly unknown -- never fabricated as true. */
TEST(SemanticGraphSnapshot,
     UnmappedNonNullRoomAndFloorReferencesRetainLocalIdentityAndLiveness)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    /* An unmapped, bad, non-null Room, referenced from a Passage. */
    Room unmappedBadRoom;
    unmappedBadRoom.setId(42);
    unmappedBadRoom.setBad();

    Passage passage;
    passage.setId(1);
    passage.setMap(p_map);
    passage.setProspectiveRoom(&unmappedBadRoom);
    p_map->AddMapPassage(&passage);

    /* An unmapped, non-null Floor, referenced from a Room. */
    Floor unmappedFloor;
    unmappedFloor.setId(7);

    Room room;
    room.setId(2);
    room.setMap(p_map);
    room.setFloor(&unmappedFloor);
    p_map->AddDetectedMapRoom(&room);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    const MapSnapshot *p_mapSnapshot =
        findMapSnapshot(snapshot, p_map->GetId());
    ASSERT_NE(p_mapSnapshot, nullptr);

    const PassageRecord *p_passageRecord = findPassageRecord(*p_mapSnapshot, 1);
    ASSERT_NE(p_passageRecord, nullptr);
    EXPECT_EQ(p_passageRecord->prospectiveRoomRef.reason,
              UnavailableReason::ENTITY_HAS_NO_MAP);
    EXPECT_FALSE(p_passageRecord->prospectiveRoomRef.key.has_value());
    ASSERT_TRUE(p_passageRecord->prospectiveRoomRef.localId.has_value());
    EXPECT_EQ(*p_passageRecord->prospectiveRoomRef.localId, 42);
    /* Room has isBad(): liveness is known, and known false. */
    ASSERT_TRUE(p_passageRecord->prospectiveRoomRef.isLive.has_value());
    EXPECT_FALSE(*p_passageRecord->prospectiveRoomRef.isLive);
    EXPECT_EQ(p_passageRecord->prospectiveRoomRef.livenessUnavailableReason,
              UnavailableReason::NONE);

    const RoomRecord *p_roomRecord = findRoomRecord(*p_mapSnapshot, 2);
    ASSERT_NE(p_roomRecord, nullptr);
    EXPECT_EQ(p_roomRecord->floorRef.reason,
              UnavailableReason::ENTITY_HAS_NO_MAP);
    EXPECT_FALSE(p_roomRecord->floorRef.key.has_value());
    ASSERT_TRUE(p_roomRecord->floorRef.localId.has_value());
    EXPECT_EQ(*p_roomRecord->floorRef.localId, 7);
    /* Floor has no isBad(): liveness must be explicitly unknown, never
     * fabricated as true (the exact defect the residual audit found). */
    EXPECT_FALSE(p_roomRecord->floorRef.isLive.has_value());
    EXPECT_EQ(p_roomRecord->floorRef.livenessUnavailableReason,
              UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA);
}

/* Corrective-audit (2026-09-05) regression coverage, extended 2026-09-06:
 * captureSemanticGraphSnapshot() used to call Atlas::GetCurrentMap(), which
 * creates a new map as a side effect whenever mpCurrentMap is null --
 * reachable in production via Atlas::clearAtlas(). Capture must never
 * mutate an Atlas in that state, and must report the coherent-view status
 * truthfully for it. */
TEST(SemanticGraphSnapshot, CaptureAfterAtlasClearedDoesNotCreateAMap)
{
    Atlas atlas(0);
    atlas.clearAtlas();
    ASSERT_EQ(atlas.GetAllMaps().size(), 0U);

    SemanticGraphSnapshot snapshot;
    {
        std::unique_lock<std::mutex> lock = atlas.acquireSemanticUpdateLock();
        snapshot = captureSemanticGraphSnapshot(&atlas);
    }

    EXPECT_FALSE(snapshot.currentMapId.has_value());
    EXPECT_EQ(snapshot.currentMapStatus, AtlasCurrentMapStatus::NO_CURRENT_MAP);
    EXPECT_TRUE(snapshot.maps.empty());
    /* The real proof: capture must not have created a map as a side
     * effect, unlike calling Atlas::GetCurrentMap() would have. */
    EXPECT_EQ(atlas.GetAllMaps().size(), 0U);
}

/* MUST-CLOSE 4: Atlas::SetMapBad(currentMap) erases the map from the active
 * set and marks it bad without clearing Atlas::mpCurrentMap; a later
 * Atlas::ChangeMap() call is what eventually installs a replacement. Between
 * those two events, the snapshot must truthfully report the current map as
 * absent from its own active-map list, via AtlasCurrentMapStatus, rather
 * than silently claiming a consistency invariant that does not hold. */
TEST(SemanticGraphSnapshot,
     CurrentMapMarkedBadBeforeChangeMapIsReportedAsNotActive)
{
    Atlas                   atlas(0);
    Map                    *p_currentMap = atlas.GetCurrentMap();
    const long unsigned int currentMapId = p_currentMap->GetId();

    atlas.SetMapBad(p_currentMap);
    /* No ChangeMap() call yet: mpCurrentMap still points at the now-bad,
     * now-inactive map. */

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    ASSERT_TRUE(snapshot.currentMapId.has_value());
    EXPECT_EQ(*snapshot.currentMapId, currentMapId);
    EXPECT_EQ(snapshot.currentMapStatus,
              AtlasCurrentMapStatus::CURRENT_MAP_NOT_ACTIVE);
    EXPECT_EQ(findMapSnapshot(snapshot, currentMapId), nullptr)
        << "the bad current map must not appear in the active map list";
    EXPECT_TRUE(snapshot.maps.empty());
}

/* MUST-CLOSE 4/6 coherence counterpart: an ordinary current map (no
 * SetMapBad() call) is reported as active and appears in maps. */
TEST(SemanticGraphSnapshot, CurrentMapStatusIsActiveForAnOrdinaryCurrentMap)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    ASSERT_TRUE(snapshot.currentMapId.has_value());
    EXPECT_EQ(*snapshot.currentMapId, p_map->GetId());
    EXPECT_EQ(snapshot.currentMapStatus,
              AtlasCurrentMapStatus::CURRENT_MAP_ACTIVE);
    const MapSnapshot *p_mapSnapshot =
        findMapSnapshot(snapshot, p_map->GetId());
    ASSERT_NE(p_mapSnapshot, nullptr);
    EXPECT_TRUE(p_mapSnapshot->isCurrentMap);
}

/* MUST-CLOSE 2: the schema's still-missing-in-the-foundation-slice fields
 * (quarantine/history, manager-private open-passage and unresolved-wall
 * hypotheses, and per-wall quarantine/observation-ray evidence) report
 * their documented unavailable reason by default, never a fabricated
 * value. */
TEST(SemanticGraphSnapshot,
     ManagerPrivateAndHistoryUnavailableDefaultsAreReported)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(0.0, 0.0, 1.0));
    p_map->AddMapPlane(&wall);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    EXPECT_EQ(snapshot.roomContextHistoryReason,
              UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE);
    EXPECT_EQ(snapshot.managerPrivateOpenPassageHypothesesReason,
              UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE);
    EXPECT_EQ(snapshot.managerPrivateUnresolvedWallHypothesesReason,
              UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE);

    const MapSnapshot *p_mapSnapshot =
        findMapSnapshot(snapshot, p_map->GetId());
    ASSERT_NE(p_mapSnapshot, nullptr);
    const WallRecord *p_wallRecord = findWallRecord(*p_mapSnapshot, 1);
    ASSERT_NE(p_wallRecord, nullptr);
    EXPECT_EQ(p_wallRecord->quarantineReason,
              UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA);
    EXPECT_EQ(p_wallRecord->observationRayEvidenceReason,
              UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA);
}

/* Corrective-audit (2026-09-05) regression coverage, upgraded 2026-09-06
 * from a reproducibility-only check to a genuine value-based determinism
 * proof: distinct source objects captured with a colliding EntityKey (same
 * kind, mapId, and local id) must both be retained, never deduplicated, and
 * the pair's relative order must be a function of their captured VALUES
 * (via isValueLessForCollisionTiebreak()), not of insertion/pointer order --
 * proven here by capturing the identical pair through two atlases built
 * with reversed Room-construction order and asserting both captures agree
 * on the same value-determined order. */
TEST(SemanticGraphSnapshot, CollidingEntityKeyRoomsAreBothRetained)
{
    auto buildAndCapture = [](bool constructLowerCentroidFirst_in)
    {
        Atlas atlas(0);
        Map  *p_map = atlas.GetCurrentMap();

        std::unique_ptr<Room> p_lowerCentroidRoom = std::make_unique<Room>();
        p_lowerCentroidRoom->setId(1);
        p_lowerCentroidRoom->setMap(p_map);
        p_lowerCentroidRoom->setCentroid(Eigen::Vector3d(1.0, 0.0, 0.0));

        std::unique_ptr<Room> p_higherCentroidRoom = std::make_unique<Room>();
        p_higherCentroidRoom->setId(1);
        p_higherCentroidRoom->setMap(p_map);
        p_higherCentroidRoom->setCentroid(Eigen::Vector3d(2.0, 0.0, 0.0));

        if (constructLowerCentroidFirst_in)
        {
            p_map->AddDetectedMapRoom(p_lowerCentroidRoom.get());
            p_map->AddDetectedMapRoom(p_higherCentroidRoom.get());
        }
        else
        {
            p_map->AddDetectedMapRoom(p_higherCentroidRoom.get());
            p_map->AddDetectedMapRoom(p_lowerCentroidRoom.get());
        }

        std::unique_lock<std::mutex> lock = atlas.acquireSemanticUpdateLock();
        return captureSemanticGraphSnapshot(&atlas);
    };

    const SemanticGraphSnapshot lowerFirstCapture  = buildAndCapture(true);
    const SemanticGraphSnapshot higherFirstCapture = buildAndCapture(false);

    for (const SemanticGraphSnapshot *p_capture :
         {&lowerFirstCapture, &higherFirstCapture})
    {
        ASSERT_EQ(p_capture->maps.size(), 1U);
        ASSERT_EQ(p_capture->maps[0].rooms.size(), 2U)
            << "both colliding-key rooms must be retained, never deduplicated";
        EXPECT_EQ(p_capture->maps[0].rooms[0].key,
                  p_capture->maps[0].rooms[1].key);
        /* Value-based canonical order: isLive/isDetectedMember/
         * isMarkerBasedMember/declaredMapId/variant are equal for both
         * rooms, so centroid_World_m is the first differentiator --
         * (1,0,0) sorts before (2,0,0) regardless of which object was
         * constructed or registered first. */
        EXPECT_EQ(p_capture->maps[0].rooms[0].centroid_World_m,
                  Eigen::Vector3d(1.0, 0.0, 0.0));
        EXPECT_EQ(p_capture->maps[0].rooms[1].centroid_World_m,
                  Eigen::Vector3d(2.0, 0.0, 0.0));
    }
}

/* MUST-CLOSE 3/6, white-box: the same value-based collision determinism as
 * the Room case above, for Wall, Passage, and Floor, proven directly against
 * the actual production sortByKey<RecordT>() template rather than through
 * Map's insertion API. Map::AddMapPlane()/AddMapPassage()/AddMapFloor() each
 * maintain their own id->pointer index and actively prevent two live
 * objects from sharing one id within a single map: AddMapPlane()/
 * AddMapFloor() silently reassign the incoming object's id to a fresh
 * unique one (confirmed by direct source read of Map.cc's AddMapPlane()/
 * AddMapFloor(), and observed directly: constructing two Planes/Floors with
 * id 1 and registering both leaves the second with a renumbered id), and
 * AddMapPassage() logs a collision and refuses the second insertion
 * (observed directly: passages.size() stays 1). Unlike Room
 * (AddDetectedMapRoom()/AddCandidateMapRoom() insert into a plain
 * std::set<Room *> with no id bookkeeping at all), a genuine same-map
 * EntityKey collision for Wall/Passage/Floor is therefore not reachable
 * through the production insertion path -- so, per this residual repair's
 * own instruction ("Add a deterministic failing test by sorting/capturing
 * logically identical colliding records supplied in opposite orders and
 * comparing every resulting value"), the collision is supplied directly to
 * sortByKey<RecordT>(), the same template captureSemanticGraphSnapshot()
 * uses for every one of these four record types. */
TEST(SemanticGraphSnapshot,
     CollidingWallRecordsAreBothRetainedAndDeterministicallyOrdered)
{
    WallRecord badWall;
    badWall.key    = makeKey(EntityKind::WALL, 1U, 7);
    badWall.isLive = false;

    WallRecord liveWall;
    liveWall.key    = makeKey(EntityKind::WALL, 1U, 7);
    liveWall.isLive = true;

    std::vector<WallRecord> ascending  = {badWall, liveWall};
    std::vector<WallRecord> descending = {liveWall, badWall};
    sortByKey(ascending);
    sortByKey(descending);

    for (const std::vector<WallRecord> *p_records : {&ascending, &descending})
    {
        ASSERT_EQ(p_records->size(), 2U);
        EXPECT_EQ((*p_records)[0].key, (*p_records)[1].key);
        /* isLive is the first field compared: false (bad) sorts before
         * true (live), regardless of the supplied input order. */
        EXPECT_FALSE((*p_records)[0].isLive);
        EXPECT_TRUE((*p_records)[1].isLive);
    }
}

TEST(SemanticGraphSnapshot,
     CollidingPassageRecordsAreBothRetainedAndDeterministicallyOrdered)
{
    PassageRecord narrowPassage;
    narrowPassage.key     = makeKey(EntityKind::PASSAGE, 1U, 7);
    narrowPassage.width_m = 0.5;

    PassageRecord widePassage;
    widePassage.key     = makeKey(EntityKind::PASSAGE, 1U, 7);
    widePassage.width_m = 1.5;

    std::vector<PassageRecord> ascending  = {narrowPassage, widePassage};
    std::vector<PassageRecord> descending = {widePassage, narrowPassage};
    sortByKey(ascending);
    sortByKey(descending);

    for (const std::vector<PassageRecord> *p_records :
         {&ascending, &descending})
    {
        ASSERT_EQ(p_records->size(), 2U);
        EXPECT_EQ((*p_records)[0].key, (*p_records)[1].key);
        /* isLive, declaredMapId, passageType, equation_World, and
         * centroid_World_m are all equal defaults for both; width_m is the
         * first differentiator. */
        EXPECT_DOUBLE_EQ((*p_records)[0].width_m, 0.5);
        EXPECT_DOUBLE_EQ((*p_records)[1].width_m, 1.5);
    }
}

TEST(SemanticGraphSnapshot,
     CollidingFloorRecordsAreBothRetainedAndDeterministicallyOrdered)
{
    FloorRecord lowerFloor;
    lowerFloor.key              = makeKey(EntityKind::FLOOR, 1U, 7);
    lowerFloor.centroid_World_m = Eigen::Vector3d(0.0, 0.0, 0.0);

    FloorRecord higherFloor;
    higherFloor.key              = makeKey(EntityKind::FLOOR, 1U, 7);
    higherFloor.centroid_World_m = Eigen::Vector3d(0.0, 0.0, 3.0);

    std::vector<FloorRecord> ascending  = {lowerFloor, higherFloor};
    std::vector<FloorRecord> descending = {higherFloor, lowerFloor};
    sortByKey(ascending);
    sortByKey(descending);

    for (const std::vector<FloorRecord> *p_records : {&ascending, &descending})
    {
        ASSERT_EQ(p_records->size(), 2U);
        EXPECT_EQ((*p_records)[0].key, (*p_records)[1].key);
        EXPECT_EQ((*p_records)[0].centroid_World_m,
                  Eigen::Vector3d(0.0, 0.0, 0.0));
        EXPECT_EQ((*p_records)[1].centroid_World_m,
                  Eigen::Vector3d(0.0, 0.0, 3.0));
    }
}

/* MUST-CLOSE 6 "duplicate relationship references": two distinct rooms
 * sharing a colliding EntityKey both own the same wall. The wall's
 * ownerRoomRefs must retain both -- not deduplicate them merely because
 * they carry the same key -- since they are genuinely different source
 * objects with different liveness. */
TEST(SemanticGraphSnapshot, CollidingOwnerRoomRefsForOneWallAreRetained)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    geometric::Plane sharedWall;
    test::makeWallPlane(sharedWall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(0.0, 0.0, 1.0));
    p_map->AddMapPlane(&sharedWall);

    Room liveOwner;
    test::makeRoom(liveOwner, 2, p_map, &sharedWall, Eigen::Vector3d(1, 0, 1));
    p_map->AddDetectedMapRoom(&liveOwner);

    Room badOwnerSameId;
    test::makeRoom(badOwnerSameId,
                   2,
                   p_map,
                   &sharedWall,
                   Eigen::Vector3d(-1, 0, 1));
    badOwnerSameId.setBad();
    p_map->AddDetectedMapRoom(&badOwnerSameId);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    const MapSnapshot *p_mapSnapshot =
        findMapSnapshot(snapshot, p_map->GetId());
    ASSERT_NE(p_mapSnapshot, nullptr);
    const WallRecord *p_wallRecord = findWallRecord(*p_mapSnapshot, 1);
    ASSERT_NE(p_wallRecord, nullptr);
    ASSERT_EQ(p_wallRecord->ownerRoomRefs.size(), 2U)
        << "both colliding-key owners must be retained, never deduplicated";
    EXPECT_EQ(p_wallRecord->ownerRoomRefs[0].key,
              p_wallRecord->ownerRoomRefs[1].key);
    ASSERT_TRUE(p_wallRecord->ownerRoomRefs[0].isLive.has_value());
    ASSERT_TRUE(p_wallRecord->ownerRoomRefs[1].isLive.has_value());
    /* isLive is isEntityRefLess()'s tiebreak after key: false before true. */
    EXPECT_FALSE(*p_wallRecord->ownerRoomRefs[0].isLive);
    EXPECT_TRUE(*p_wallRecord->ownerRoomRefs[1].isLive);
}

/* Schema/reference/concurrency reviewer's non-blocking observation on the
 * 2026-09-06 residual repair: WallRecord::ownerRoomRefs' key must come from
 * the containing map used to enumerate the owning room (matching that
 * room's own RoomRecord::key), never from the room's own possibly-different
 * declared map -- captureSemanticGraphSnapshot.cc's wall-ownership
 * inversion pass builds this key from its own per-map loop variable, not
 * from entityRefForRoom()'s declared-map semantics, specifically to keep
 * this fact true even for a mismatched room. */
TEST(SemanticGraphSnapshot,
     OwnerRoomRefIsKeyedByContainingMapEvenWhenRoomDeclaresADifferentMap)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.GetCurrentMap();
    atlas.CreateNewMap();
    Map *p_mapB = atlas.GetCurrentMap();

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(0.0, 0.0, 1.0));
    p_mapA->AddMapPlane(&wall);

    /* Enumerated from mapA (AddDetectedMapRoom), but declares mapB. */
    Room mismatchedOwner;
    mismatchedOwner.setId(2);
    mismatchedOwner.setMap(p_mapB);
    mismatchedOwner.setWalls(&wall);
    p_mapA->AddDetectedMapRoom(&mismatchedOwner);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    const MapSnapshot *p_mapSnapshotA =
        findMapSnapshot(snapshot, p_mapA->GetId());
    ASSERT_NE(p_mapSnapshotA, nullptr);
    const RoomRecord *p_ownerRecord = findRoomRecord(*p_mapSnapshotA, 2);
    ASSERT_NE(p_ownerRecord, nullptr);
    EXPECT_EQ(p_ownerRecord->key.mapId, p_mapA->GetId());
    ASSERT_TRUE(p_ownerRecord->declaredMapId.has_value());
    EXPECT_EQ(*p_ownerRecord->declaredMapId, p_mapB->GetId());

    const WallRecord *p_wallRecord = findWallRecord(*p_mapSnapshotA, 1);
    ASSERT_NE(p_wallRecord, nullptr);
    ASSERT_EQ(p_wallRecord->ownerRoomRefs.size(), 1U);
    ASSERT_TRUE(p_wallRecord->ownerRoomRefs[0].key.has_value());
    /* The owner ref's key must equal the owner's own RoomRecord::key
     * (mapA-qualified), not a mapB-qualified key derived from the room's
     * own declared map. */
    EXPECT_EQ(*p_wallRecord->ownerRoomRefs[0].key, p_ownerRecord->key);
    EXPECT_EQ(p_wallRecord->ownerRoomRefs[0].key->mapId, p_mapA->GetId());
}

/* Minimum-proof item 7: bad room owners and duplicate/colliding relationship
 * evidence are retained. */
TEST(SemanticGraphSnapshot, BadRoomOwnersAndDuplicateWallOwnershipAreRetained)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    geometric::Plane sharedWall;
    test::makeWallPlane(sharedWall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(0.0, 0.0, 1.0));
    p_map->AddMapPlane(&sharedWall);

    /* Two distinct, live rooms both claim the same wall -- a real AX-WALL-01
     * violation that capture must retain, not silently resolve. */
    Room firstOwner;
    test::makeRoom(firstOwner, 2, p_map, &sharedWall, Eigen::Vector3d(1, 0, 1));
    p_map->AddDetectedMapRoom(&firstOwner);

    Room secondOwner;
    test::makeRoom(secondOwner,
                   3,
                   p_map,
                   &sharedWall,
                   Eigen::Vector3d(-1, 0, 1));
    p_map->AddDetectedMapRoom(&secondOwner);

    /* A retired (bad) room that still lists the wall. */
    Room badOwner;
    test::makeRoom(badOwner, 4, p_map, &sharedWall, Eigen::Vector3d(0, 1, 1));
    badOwner.setBad();
    p_map->AddDetectedMapRoom(&badOwner);

    /* A room registered in BOTH Map::GetAllDetectedMapRooms() and
     * Map::GetAllMarkerBasedMapRooms(), owning its own wall. This is the
     * exact case captureSemanticGraphSnapshot.cc's dedup-by-pointer wall-
     * ownership pass (not Map::GetAllRooms()'s duplicating union) exists to
     * handle: the owner must appear exactly once in ownerRoomRefs, not
     * twice for the two collections it happens to be registered in. */
    geometric::Plane doublyRegisteredOwnerWall;
    test::makeWallPlane(doublyRegisteredOwnerWall,
                        5,
                        p_map,
                        Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(2.0, 0.0, 1.0));
    p_map->AddMapPlane(&doublyRegisteredOwnerWall);
    Room doublyRegisteredOwner;
    test::makeRoom(doublyRegisteredOwner,
                   6,
                   p_map,
                   &doublyRegisteredOwnerWall,
                   Eigen::Vector3d(2, 0, 1));
    p_map->AddDetectedMapRoom(&doublyRegisteredOwner);
    p_map->AddCandidateMapRoom(&doublyRegisteredOwner);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    const MapSnapshot *p_mapSnapshot =
        findMapSnapshot(snapshot, p_map->GetId());
    ASSERT_NE(p_mapSnapshot, nullptr);

    const WallRecord *p_sharedWallRecord = findWallRecord(*p_mapSnapshot, 1);
    ASSERT_NE(p_sharedWallRecord, nullptr);
    ASSERT_EQ(p_sharedWallRecord->ownerRoomRefs.size(), 3U);

    const RoomRecord *p_badOwnerRecord = findRoomRecord(*p_mapSnapshot, 4);
    ASSERT_NE(p_badOwnerRecord, nullptr);
    EXPECT_FALSE(p_badOwnerRecord->isLive);
    ASSERT_EQ(p_badOwnerRecord->wallRefs.size(), 1U);

    const RoomRecord *p_doublyRegisteredOwnerRecord =
        findRoomRecord(*p_mapSnapshot, 6);
    ASSERT_NE(p_doublyRegisteredOwnerRecord, nullptr);
    EXPECT_TRUE(p_doublyRegisteredOwnerRecord->isDetectedMember);
    EXPECT_TRUE(p_doublyRegisteredOwnerRecord->isMarkerBasedMember);
    const WallRecord *p_doublyRegisteredOwnerWallRecord =
        findWallRecord(*p_mapSnapshot, 5);
    ASSERT_NE(p_doublyRegisteredOwnerWallRecord, nullptr);
    ASSERT_EQ(p_doublyRegisteredOwnerWallRecord->ownerRoomRefs.size(), 1U);
    ASSERT_TRUE(
        p_doublyRegisteredOwnerWallRecord->ownerRoomRefs[0].key.has_value());
    EXPECT_EQ(*p_doublyRegisteredOwnerWallRecord->ownerRoomRefs[0].key,
              p_doublyRegisteredOwnerRecord->key);
    ASSERT_TRUE(p_badOwnerRecord->wallRefs[0].wallKey.has_value());
    EXPECT_EQ(*p_badOwnerRecord->wallRefs[0].wallKey, p_sharedWallRecord->key);

    bool badOwnerKeyPresent = false;
    for (const EntityRef &ownerRef : p_sharedWallRecord->ownerRoomRefs)
    {
        if (ownerRef.key.has_value() && *ownerRef.key == p_badOwnerRecord->key)
        {
            badOwnerKeyPresent = true;
            ASSERT_TRUE(ownerRef.isLive.has_value());
            EXPECT_FALSE(*ownerRef.isLive);
        }
    }
    EXPECT_TRUE(badOwnerKeyPresent);
}

/* Minimum-proof item 8: all records and relationship collections are
 * deterministically sorted for permuted insertion order without mutating
 * the source graph. */
TEST(
    SemanticGraphSnapshot,
    RecordsAreDeterministicallySortedRegardlessOfInsertionOrderAndSourceIsUnmutated)
{
    auto buildAndCapture = [](const std::vector<int> &roomIdInsertionOrder_in)
    {
        Atlas                              atlas(0);
        Map                               *p_map = atlas.GetCurrentMap();
        std::vector<std::unique_ptr<Room>> rooms;
        for (int roomId : roomIdInsertionOrder_in)
        {
            std::unique_ptr<Room> p_room = std::make_unique<Room>();
            p_room->setId(roomId);
            p_room->setMap(p_map);
            p_map->AddDetectedMapRoom(p_room.get());
            rooms.push_back(std::move(p_room));
        }
        std::unique_lock<std::mutex> lock = atlas.acquireSemanticUpdateLock();
        return captureSemanticGraphSnapshot(&atlas);
    };

    const SemanticGraphSnapshot ascending  = buildAndCapture({1, 2, 3});
    const SemanticGraphSnapshot descending = buildAndCapture({3, 2, 1});

    ASSERT_EQ(ascending.maps.size(), 1U);
    ASSERT_EQ(descending.maps.size(), 1U);
    ASSERT_EQ(ascending.maps[0].rooms.size(), 3U);
    ASSERT_EQ(descending.maps[0].rooms.size(), 3U);

    for (std::size_t index = 0U; index < 3U; ++index)
    {
        EXPECT_EQ(ascending.maps[0].rooms[index].key.entityId,
                  static_cast<int>(index) + 1);
        EXPECT_EQ(descending.maps[0].rooms[index].key.entityId,
                  static_cast<int>(index) + 1);
    }
    EXPECT_TRUE(
        std::is_sorted(ascending.maps[0].rooms.begin(),
                       ascending.maps[0].rooms.end(),
                       [](const RoomRecord &lhs_in, const RoomRecord &rhs_in)
                       { return lhs_in.key < rhs_in.key; }));

    /* Non-mutation: capturing must not have changed the source graph. */
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();
    Room  room;
    room.setId(1);
    room.setMap(p_map);
    room.setCentroid(Eigen::Vector3d(3.0, 4.0, 5.0));
    p_map->AddDetectedMapRoom(&room);
    {
        std::unique_lock<std::mutex> lock = atlas.acquireSemanticUpdateLock();
        (void)captureSemanticGraphSnapshot(&atlas);
    }
    EXPECT_EQ(room.getCentroid(), Eigen::Vector3d(3.0, 4.0, 5.0));
    EXPECT_FALSE(room.isBad());
    EXPECT_EQ(p_map->GetAllDetectedMapRooms().size(), 1U);
}

/* MUST-CLOSE 6 "ordering of maps and every record/relationship collection":
 * extends coverage beyond rooms (above) to walls, passages, floors, and the
 * top-level maps vector, each for permuted insertion order. A genuine map-
 * id collision is not exercised: Atlas::Map::nNextId is a monotonically
 * increasing static counter (confirmed by direct source read of Map.h), so
 * two distinct Map objects sharing one Atlas can never collide -- there is
 * no reachable state to construct here, unlike the room/wall/passage/floor
 * local-id collisions above (which are per-map local counters an external
 * caller can set directly via setId()). */
TEST(
    SemanticGraphSnapshot,
    WallsPassagesFloorsAndMapsAreDeterministicallySortedRegardlessOfInsertionOrder)
{
    auto buildAndCapture = [](bool ascendingInsertionOrder_in)
    {
        Atlas atlas(0);
        Map  *p_mapA = atlas.GetCurrentMap();
        /* A second, otherwise-unused map only to give the top-level maps
         * vector two entries to check std::is_sorted() over. */
        atlas.CreateNewMap();

        std::vector<std::unique_ptr<geometric::Plane>>   walls;
        std::vector<std::unique_ptr<Passage>> passages;
        std::vector<std::unique_ptr<Floor>>   floors;
        for (int index = 0; index < 3; ++index)
        {
            const int entityId =
                ascendingInsertionOrder_in ? index + 1 : 3 - index;

            std::unique_ptr<geometric::Plane> p_wall = std::make_unique<geometric::Plane>();
            test::makeWallPlane(
                *p_wall,
                entityId,
                p_mapA,
                Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                Eigen::Vector3d::UnitY(),
                Eigen::Vector3d::UnitZ(),
                1.0,
                1.0,
                Eigen::Vector3d(static_cast<double>(index), 0.0, 1.0));
            p_mapA->AddMapPlane(p_wall.get());
            walls.push_back(std::move(p_wall));

            std::unique_ptr<Passage> p_passage = std::make_unique<Passage>();
            p_passage->setId(entityId);
            p_passage->setMap(p_mapA);
            p_mapA->AddMapPassage(p_passage.get());
            passages.push_back(std::move(p_passage));

            std::unique_ptr<Floor> p_floor = std::make_unique<Floor>();
            p_floor->setId(entityId);
            p_floor->setMap(p_mapA);
            p_mapA->AddMapFloor(p_floor.get());
            floors.push_back(std::move(p_floor));
        }

        std::unique_lock<std::mutex> lock = atlas.acquireSemanticUpdateLock();
        SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
        return std::make_pair(std::move(snapshot), p_mapA->GetId());
    };

    const auto [ascendingSnapshot, ascendingMapAId]   = buildAndCapture(true);
    const auto [descendingSnapshot, descendingMapAId] = buildAndCapture(false);

    EXPECT_TRUE(
        std::is_sorted(ascendingSnapshot.maps.begin(),
                       ascendingSnapshot.maps.end(),
                       [](const MapSnapshot &lhs_in, const MapSnapshot &rhs_in)
                       { return lhs_in.mapId < rhs_in.mapId; }));
    EXPECT_TRUE(
        std::is_sorted(descendingSnapshot.maps.begin(),
                       descendingSnapshot.maps.end(),
                       [](const MapSnapshot &lhs_in, const MapSnapshot &rhs_in)
                       { return lhs_in.mapId < rhs_in.mapId; }));
    ASSERT_EQ(ascendingSnapshot.maps.size(), 2U);
    ASSERT_EQ(descendingSnapshot.maps.size(), 2U);

    const std::pair<const SemanticGraphSnapshot *, long unsigned int>
        capturesWithMapAId[] = {{&ascendingSnapshot, ascendingMapAId},
                                {&descendingSnapshot, descendingMapAId}};
    for (const auto &[p_capture, mapAId] : capturesWithMapAId)
    {
        const MapSnapshot *p_mapSnapshotPtr =
            findMapSnapshot(*p_capture, mapAId);
        ASSERT_NE(p_mapSnapshotPtr, nullptr);
        const MapSnapshot &mapSnapshot = *p_mapSnapshotPtr;
        ASSERT_EQ(mapSnapshot.walls.size(), 3U);
        ASSERT_EQ(mapSnapshot.passages.size(), 3U);
        ASSERT_EQ(mapSnapshot.floors.size(), 3U);
        for (int index = 0; index < 3; ++index)
        {
            EXPECT_EQ(
                mapSnapshot.walls[static_cast<std::size_t>(index)].key.entityId,
                index + 1);
            EXPECT_EQ(mapSnapshot.passages[static_cast<std::size_t>(index)]
                          .key.entityId,
                      index + 1);
            EXPECT_EQ(mapSnapshot.floors[static_cast<std::size_t>(index)]
                          .key.entityId,
                      index + 1);
        }
    }
}

/* MUST-CLOSE 6 "ordering of ... every record/relationship collection",
 * closing a gap the final C++17/determinism reviewer found: the tests above
 * prove permuted-insertion-order determinism for the top-level per-map
 * record vectors, but not yet for a *relationship* collection built from
 * several distinct (non-colliding) members -- RoomRecord::wallRefs/
 * passageRefs, WallRecord::ownerRoomRefs, FloorRecord::roomRefs, and
 * PassageRecord::associateWallRefs. Each is built here from three distinct
 * members added in reversed order across two captures. */
TEST(
    SemanticGraphSnapshot,
    RelationshipCollectionsAreDeterministicallySortedRegardlessOfInsertionOrder)
{
    auto buildAndCapture = [](bool ascendingInsertionOrder_in)
    {
        Atlas atlas(0);
        Map  *p_map = atlas.GetCurrentMap();

        std::vector<std::unique_ptr<geometric::Plane>> walls;
        for (int index = 0; index < 3; ++index)
        {
            std::unique_ptr<geometric::Plane> p_wall = std::make_unique<geometric::Plane>();
            test::makeWallPlane(
                *p_wall,
                index + 1,
                p_map,
                Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                Eigen::Vector3d::UnitY(),
                Eigen::Vector3d::UnitZ(),
                1.0,
                1.0,
                Eigen::Vector3d(static_cast<double>(index), 0.0, 1.0));
            p_map->AddMapPlane(p_wall.get());
            walls.push_back(std::move(p_wall));
        }

        std::vector<std::unique_ptr<Passage>> passages;
        for (int index = 0; index < 3; ++index)
        {
            std::unique_ptr<Passage> p_passage = std::make_unique<Passage>();
            p_passage->setId(index + 1);
            p_passage->setMap(p_map);
            p_map->AddMapPassage(p_passage.get());
            passages.push_back(std::move(p_passage));
        }

        std::vector<std::unique_ptr<Room>> owningRooms;
        for (int index = 0; index < 3; ++index)
        {
            std::unique_ptr<Room> p_room = std::make_unique<Room>();
            p_room->setId(index + 20);
            p_room->setMap(p_map);
            owningRooms.push_back(std::move(p_room));
        }

        std::unique_ptr<geometric::Plane> p_sharedWall = std::make_unique<geometric::Plane>();
        test::makeWallPlane(*p_sharedWall,
                            4,
                            p_map,
                            Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                            Eigen::Vector3d::UnitY(),
                            Eigen::Vector3d::UnitZ(),
                            1.0,
                            1.0,
                            Eigen::Vector3d(4.0, 0.0, 1.0));
        p_map->AddMapPlane(p_sharedWall.get());

        Room subjectRoom;
        subjectRoom.setId(10);
        subjectRoom.setMap(p_map);

        Floor subjectFloor;
        subjectFloor.setId(30);
        subjectFloor.setMap(p_map);

        Passage subjectPassage;
        subjectPassage.setId(40);
        subjectPassage.setMap(p_map);

        const std::vector<int> memberOrder = ascendingInsertionOrder_in
                                                 ? std::vector<int>{0, 1, 2}
                                                 : std::vector<int>{2, 1, 0};
        for (int index : memberOrder)
        {
            const std::size_t memberIndex = static_cast<std::size_t>(index);
            subjectRoom.setWalls(walls[memberIndex].get());
            subjectRoom.setDoorways(passages[memberIndex].get());
            owningRooms[memberIndex]->setWalls(p_sharedWall.get());
            subjectFloor.addRoom(owningRooms[memberIndex].get());
            subjectPassage.addAssociateWall(walls[memberIndex].get());
        }
        p_map->AddDetectedMapRoom(&subjectRoom);
        for (const std::unique_ptr<Room> &p_owningRoom : owningRooms)
        {
            p_map->AddDetectedMapRoom(p_owningRoom.get());
        }
        p_map->AddMapFloor(&subjectFloor);
        p_map->AddMapPassage(&subjectPassage);

        std::unique_lock<std::mutex> lock = atlas.acquireSemanticUpdateLock();
        return captureSemanticGraphSnapshot(&atlas);
    };

    const SemanticGraphSnapshot ascending  = buildAndCapture(true);
    const SemanticGraphSnapshot descending = buildAndCapture(false);

    for (const SemanticGraphSnapshot *p_capture : {&ascending, &descending})
    {
        ASSERT_EQ(p_capture->maps.size(), 1U);
        const MapSnapshot &mapSnapshot = p_capture->maps[0];

        const RoomRecord *p_subjectRoom = findRoomRecord(mapSnapshot, 10);
        ASSERT_NE(p_subjectRoom, nullptr);
        ASSERT_EQ(p_subjectRoom->wallRefs.size(), 3U);
        ASSERT_EQ(p_subjectRoom->passageRefs.size(), 3U);
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            EXPECT_EQ(p_subjectRoom->wallRefs[index].planeId,
                      static_cast<int>(index) + 1);
            ASSERT_TRUE(p_subjectRoom->passageRefs[index].key.has_value());
            EXPECT_EQ(p_subjectRoom->passageRefs[index].key->entityId,
                      static_cast<int>(index) + 1);
        }

        const WallRecord *p_sharedWallRecord = findWallRecord(mapSnapshot, 4);
        ASSERT_NE(p_sharedWallRecord, nullptr);
        ASSERT_EQ(p_sharedWallRecord->ownerRoomRefs.size(), 3U);
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            ASSERT_TRUE(
                p_sharedWallRecord->ownerRoomRefs[index].key.has_value());
            EXPECT_EQ(p_sharedWallRecord->ownerRoomRefs[index].key->entityId,
                      static_cast<int>(index) + 20);
        }

        ASSERT_EQ(mapSnapshot.floors.size(), 1U);
        ASSERT_EQ(mapSnapshot.floors[0].roomRefs.size(), 3U);
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            ASSERT_TRUE(mapSnapshot.floors[0].roomRefs[index].key.has_value());
            EXPECT_EQ(mapSnapshot.floors[0].roomRefs[index].key->entityId,
                      static_cast<int>(index) + 20);
        }

        const PassageRecord *p_subjectPassage =
            findPassageRecord(mapSnapshot, 40);
        ASSERT_NE(p_subjectPassage, nullptr);
        ASSERT_EQ(p_subjectPassage->associateWallRefs.size(), 3U);
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            EXPECT_EQ(p_subjectPassage->associateWallRefs[index].planeId,
                      static_cast<int>(index) + 1);
        }
    }
}

/* Minimum-proof item 9: capture requires the caller-held semantic lock and
 * does not try to reacquire it. Atlas::mMutexSemanticUpdate is a plain,
 * non-recursive std::mutex, so if captureSemanticGraphSnapshot() ever tried
 * to acquire it again on this thread, this test would deadlock rather than
 * fail cleanly -- reaching the final assertion is itself the proof. */
TEST(SemanticGraphSnapshot,
     CaptureUnderHeldSemanticLockCompletesWithoutReacquiring)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();
    Room  room;
    room.setId(1);
    room.setMap(p_map);
    p_map->AddDetectedMapRoom(&room);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    ASSERT_EQ(snapshot.maps.size(), 1U);
    EXPECT_EQ(snapshot.maps[0].rooms.size(), 1U);
}

/* Minimum-proof item 10: the cheap geometric::Plane accessor agrees with the scalar
 * fields in the full geometry snapshot while returning no cloud payload. */
TEST(SemanticGraphSnapshot,
     CheapPlaneAccessorAgreesWithFullGeometrySnapshotScalars)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(0.0, 1.0, 0.0, -2.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitZ(),
                        1.5,
                        0.75,
                        Eigen::Vector3d(0.0, 2.0, 0.0));
    p_map->AddMapPlane(&wall);

    const geometric::Plane::GeometrySnapshot fullSnapshot = wall.getGeometrySnapshot();
    const geometric::PlaneGeometryMetadataSnapshot metadata =
        wall.getGeometryMetadataSnapshot();

    EXPECT_EQ(metadata.equation_World, fullSnapshot.equation_World);
    EXPECT_EQ(metadata.centroid_World_m, fullSnapshot.centroid_World_m);
    EXPECT_DOUBLE_EQ(metadata.minPlaneU_m, fullSnapshot.minPlaneU_m);
    EXPECT_DOUBLE_EQ(metadata.maxPlaneU_m, fullSnapshot.maxPlaneU_m);
    EXPECT_DOUBLE_EQ(metadata.minPlaneV_m, fullSnapshot.minPlaneV_m);
    EXPECT_DOUBLE_EQ(metadata.maxPlaneV_m, fullSnapshot.maxPlaneV_m);
    EXPECT_EQ(metadata.finiteSupportCount, fullSnapshot.finiteSupportCount);
    EXPECT_EQ(metadata.observationCount, fullSnapshot.observationCount);
    EXPECT_EQ(metadata.cloudGeneration, fullSnapshot.cloudGeneration);
    EXPECT_EQ(metadata.successfulRefitGeneration,
              fullSnapshot.successfulRefitGeneration);
    ASSERT_NE(fullSnapshot.supportCloud, nullptr);
    EXPECT_GT(fullSnapshot.supportCloud->size(), 0U);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    const MapSnapshot          *p_mapSnapshot =
        findMapSnapshot(snapshot, p_map->GetId());
    ASSERT_NE(p_mapSnapshot, nullptr);
    const WallRecord *p_wallRecord = findWallRecord(*p_mapSnapshot, 1);
    ASSERT_NE(p_wallRecord, nullptr);

    /* Every one of the 10 geometric::PlaneGeometryMetadataSnapshot fields, not just a
     * convenient subset -- a field-swap bug touching any single one of
     * these (e.g. minPlaneV_m/maxPlaneV_m, observationCount, or
     * successfulRefitGeneration) must fail this test. */
    EXPECT_EQ(p_wallRecord->equation_World, metadata.equation_World);
    EXPECT_EQ(p_wallRecord->centroid_World_m, metadata.centroid_World_m);
    EXPECT_DOUBLE_EQ(p_wallRecord->minPlaneU_m, metadata.minPlaneU_m);
    EXPECT_DOUBLE_EQ(p_wallRecord->maxPlaneU_m, metadata.maxPlaneU_m);
    EXPECT_DOUBLE_EQ(p_wallRecord->minPlaneV_m, metadata.minPlaneV_m);
    EXPECT_DOUBLE_EQ(p_wallRecord->maxPlaneV_m, metadata.maxPlaneV_m);
    EXPECT_EQ(p_wallRecord->finiteSupportCount, metadata.finiteSupportCount);
    EXPECT_EQ(p_wallRecord->observationCount, metadata.observationCount);
    EXPECT_EQ(p_wallRecord->cloudGeneration, metadata.cloudGeneration);
    EXPECT_EQ(p_wallRecord->successfulRefitGeneration,
              metadata.successfulRefitGeneration);
}

/* Additional coverage: a null Atlas pointer returns a default (empty)
 * snapshot rather than crashing. */
TEST(SemanticGraphSnapshot, NullAtlasReturnsEmptyDefaultSnapshot)
{
    const SemanticGraphSnapshot snapshot =
        captureSemanticGraphSnapshot(nullptr);
    EXPECT_FALSE(snapshot.currentMapId.has_value());
    EXPECT_EQ(snapshot.currentMapStatus, AtlasCurrentMapStatus::NO_CURRENT_MAP);
    EXPECT_TRUE(snapshot.maps.empty());
}

/* MUST-CLOSE 1 "helper-level null/drop test": appendWallRef()/
 * appendRoomRef()/appendPassageRef() must never dereference a null pointer
 * and must append nothing for one. Room::setWalls(), Passage::
 * addAssociateWall(), Floor::addRoom(), and Floor::setRooms() all reject a
 * null pointer before insertion (confirmed by direct source read of
 * Room.cc:273-279, Passage.cc:357-362, Floor.cc:289-294,316-326), so this
 * path is not reachable through the production capture entry point today --
 * it is proven directly against the helper itself instead, white-box, via
 * private_functions.h. */
TEST(SemanticGraphSnapshot, AppendHelpersDropNullPointersWithoutAppending)
{
    std::vector<RawPlaneRef> wallRefs;
    appendWallRef(nullptr, wallRefs);
    EXPECT_TRUE(wallRefs.empty());

    std::vector<EntityRef> roomRefs;
    appendRoomRef(nullptr, roomRefs);
    EXPECT_TRUE(roomRefs.empty());

    std::vector<EntityRef> passageRefs;
    appendPassageRef(nullptr, passageRefs);
    EXPECT_TRUE(passageRefs.empty());
}

/* MUST-CLOSE 3 "finite, NaN, infinity and signed-zero ordering": white-box
 * test of doubleTotalOrderKey()/isDoubleLess() directly, since these are the
 * primitives every other determinism guarantee in this module is built on. */
TEST(SemanticGraphSnapshot, FloatTotalOrderHandlesNaNInfinityAndSignedZero)
{
    const double negativeInfinity = -std::numeric_limits<double>::infinity();
    const double positiveInfinity = std::numeric_limits<double>::infinity();
    const double negativeZero     = -0.0;
    const double positiveZero     = 0.0;
    const double negativeNaN      = -std::numeric_limits<double>::quiet_NaN();
    const double positiveNaN      = std::numeric_limits<double>::quiet_NaN();

    /* -0.0 sorts strictly before +0.0, even though they compare `==` under
     * ordinary IEEE 754 comparison. */
    EXPECT_TRUE(isDoubleLess(negativeZero, positiveZero));
    EXPECT_FALSE(isDoubleLess(positiveZero, negativeZero));

    /* Every negative NaN sorts before every finite/infinite value, which
     * sorts before every positive NaN -- a single consistent, reproducible
     * total order, unlike ordinary `<`, under which every NaN comparison is
     * false in both directions. */
    const std::vector<double> ascendingSample = {negativeNaN,
                                                 negativeInfinity,
                                                 -1.0,
                                                 negativeZero,
                                                 positiveZero,
                                                 1.0,
                                                 positiveInfinity,
                                                 positiveNaN};
    for (std::size_t lowerIndex = 0U; lowerIndex < ascendingSample.size();
         ++lowerIndex)
    {
        for (std::size_t upperIndex = lowerIndex + 1U;
             upperIndex < ascendingSample.size();
             ++upperIndex)
        {
            EXPECT_TRUE(isDoubleLess(ascendingSample[lowerIndex],
                                     ascendingSample[upperIndex]))
                << "index " << lowerIndex << " should sort before index "
                << upperIndex;
            EXPECT_FALSE(isDoubleLess(ascendingSample[upperIndex],
                                      ascendingSample[lowerIndex]));
        }
    }

    /* Irreflexivity: a value is never less than itself, including NaN. */
    for (double value : ascendingSample)
    {
        EXPECT_FALSE(isDoubleLess(value, value));
    }

    /* Ordinary finite values still compare exactly as expected. */
    EXPECT_TRUE(isDoubleLess(1.0, 2.0));
    EXPECT_FALSE(isDoubleLess(2.0, 1.0));
    EXPECT_FALSE(isDoubleLess(1.0, 1.0));
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
