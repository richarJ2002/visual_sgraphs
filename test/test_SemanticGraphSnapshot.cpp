/*!
 * @file            test_SemanticGraphSnapshot.cpp
 *
 * @brief           Unit tests for semantic graph snapshots
 *                  (SemanticGraphSnapshot).
 */

/*
 * Focused, ROS/Gazebo-free tests for the value-only SemanticGraphSnapshot
 * capture contract. These
 * exercise the genuine production capture entry point
 * (vs_graphs::core::semantic::captureSemanticGraphSnapshot()) against real
 * Atlas/Map/Room/`geometric::Plane`/Passage/Floor objects built through
 * SemanticFixtures and the model's own setters -- they do not reconstruct the
 * expected snapshot by hand, except where a test white-box-verifies one
 * internal helper directly (documented at each such case).
 *
 * Each TEST below is annotated with the minimum-proof item it satisfies.
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
#include <rclcpp/logging.hpp>

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
        Map  *p_map = nullptr;
        ASSERT_EQ((atlas.getCurrentMap(p_map)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        unsigned long mapId2{};
        ASSERT_EQ((p_map->getId(mapId2)), MapStatus::MAP_STATUS_SUCCESS);
        mapId = mapId2;

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
        ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

        Room room;
        test::makeRoom(room, 1, p_map, &wall, Eigen::Vector3d(1.0, 0.0, 1.0));
        ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
                  MapStatus::MAP_STATUS_SUCCESS);

        std::unique_lock<std::mutex> lock{};
        ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        SemanticGraphSnapshot snapshot{};
        ASSERT_EQ((captureSemanticGraphSnapshot(&atlas, snapshot)),
                  SemanticGraphSnapshotStatus::
                      SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
        capturedSnapshot = snapshot;
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
    EXPECT_EQ(p_mapSnapshot->walls[0].planeType,
              geometric::Plane::PlaneVariant::WALL);
    /* updateSizeOfPlane() (called by makeWallPlane()) populates the finite
     * U/V bounds; finiteSupportCount is populated only by the separate
     * beginMapCloudRefit()/completeMapCloudRefit() pipeline, which this
     * fixture does not exercise, so it is correctly captured as 0 here. */
    EXPECT_GT(p_mapSnapshot->walls[0].maxPlaneU_m,
              p_mapSnapshot->walls[0].minPlaneU_m);
    EXPECT_EQ(p_mapSnapshot->walls[0].finiteSupportCount, 0U);
}

/* "value lifetime after source objects are destroyed": extend
 * lifetime coverage beyond the whole-snapshot scope-exit case above to the
 * EntityRef/RawPlaneRef sub-values inside relationship collections
 * specifically (RoomRecord::wallRefs/passageRefs, WallRecord::ownerRoomRefs,
 * FloorRecord::roomRefs), which hold EntityRef/RawPlaneRef sub-values
 * rather than bare EntityKey. */
TEST(SemanticGraphSnapshot,
     RelationshipSubValuesRemainValidAfterFixtureModelObjectsLeaveScope)
{
    std::optional<SemanticGraphSnapshot> capturedSnapshot;
    long unsigned int                    mapId = 0U;
    {
        Atlas atlas(0);
        Map  *p_map = nullptr;
        ASSERT_EQ((atlas.getCurrentMap(p_map)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        unsigned long mapId2{};
        ASSERT_EQ((p_map->getId(mapId2)), MapStatus::MAP_STATUS_SUCCESS);
        mapId = mapId2;

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
        ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

        Room room;
        test::makeRoom(room, 2, p_map, &wall, Eigen::Vector3d(1.0, 0.0, 1.0));
        ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
                  MapStatus::MAP_STATUS_SUCCESS);

        Passage passage;
        ASSERT_EQ(
            (passage.setId(3)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        ASSERT_EQ(
            (passage.setMap(p_map)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        ASSERT_EQ(
            (passage.setKnownSideRoom(&room)),
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
        ASSERT_EQ((p_map->addMapPassage(&passage)),
                  MapStatus::MAP_STATUS_SUCCESS);
        ASSERT_EQ((room.setDoorways(&passage)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

        Floor floor;
        test::makeFloor(floor, 4, p_map, {&room});
        ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);
        ASSERT_EQ((room.setFloor(&floor)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

        std::unique_lock<std::mutex> lock{};
        ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        SemanticGraphSnapshot snapshot{};
        ASSERT_EQ((captureSemanticGraphSnapshot(&atlas, snapshot)),
                  SemanticGraphSnapshotStatus::
                      SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
        capturedSnapshot = snapshot;
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
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    unsigned long id{};
    ASSERT_EQ((p_mapA->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    unsigned long id2{};
    ASSERT_EQ((p_mapB->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_NE(id, id2);

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
    ASSERT_EQ((p_mapA->addMapPlane(&wallA)), MapStatus::MAP_STATUS_SUCCESS);
    Room roomA;
    test::makeRoom(roomA, 1, p_mapA, &wallA);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&roomA)),
              MapStatus::MAP_STATUS_SUCCESS);

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
    ASSERT_EQ((p_mapB->addMapPlane(&wallB)), MapStatus::MAP_STATUS_SUCCESS);
    Room roomB;
    test::makeRoom(roomB, 1, p_mapB, &wallB);
    ASSERT_EQ((p_mapB->addDetectedMapRoom(&roomB)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passageA;
    ASSERT_EQ((passageA.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passageA.setMap(p_mapA)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addMapPassage(&passageA)),
              MapStatus::MAP_STATUS_SUCCESS);
    Passage passageB;
    ASSERT_EQ((passageB.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passageB.setMap(p_mapB)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_mapB->addMapPassage(&passageB)),
              MapStatus::MAP_STATUS_SUCCESS);

    Floor floorA;
    ASSERT_EQ((floorA.setId(1)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ((floorA.setMap(p_mapA)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addMapFloor(&floorA)), MapStatus::MAP_STATUS_SUCCESS);
    Floor floorB;
    ASSERT_EQ((floorB.setId(1)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ((floorB.setMap(p_mapB)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ((p_mapB->addMapFloor(&floorB)), MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    ASSERT_EQ(snapshot.maps.size(), 2U);
    unsigned long mapAId{};
    ASSERT_EQ((p_mapA->getId(mapAId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_snapshotA = findMapSnapshot(snapshot, mapAId);
    unsigned long      mapBId{};
    ASSERT_EQ((p_mapB->getId(mapBId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_snapshotB = findMapSnapshot(snapshot, mapBId);
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
 * ("both null declared-map and containing/declared-map mismatch for
 * every kind"): containing-map versus declared-map mismatch
 * and null declared map are preserved explicitly for every entity kind. */
TEST(SemanticGraphSnapshot,
     ContainingMapVersusDeclaredMapIsPreservedForEveryEntityKind)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    /* Room: enumerated from mapA, but declares mapB. */
    Room mismatchedRoom;
    ASSERT_EQ((mismatchedRoom.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((mismatchedRoom.setMap(p_mapB)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&mismatchedRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Room: enumerated from mapA, declares no map at all. */
    Room noMapRoom;
    ASSERT_EQ((noMapRoom.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&noMapRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

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
    ASSERT_EQ((p_mapA->addMapPlane(&mismatchedWall)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Wall: enumerated from mapA, declares no map at all. Map::AddMapPlane()
     * only requires a non-null pointer; the plane's own setMap() is
     * independent (confirmed by direct source read of
     * Map.h/geometric::Plane.h). */
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
    ASSERT_EQ((p_mapA->addMapPlane(&noMapWall)), MapStatus::MAP_STATUS_SUCCESS);

    /* Passage: enumerated from mapA, declares no map at all. */
    Passage noMapPassage;
    ASSERT_EQ((noMapPassage.setId(4)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addMapPassage(&noMapPassage)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Passage: enumerated from mapA, but declares mapB. */
    Passage mismatchedPassage;
    ASSERT_EQ((mismatchedPassage.setId(7)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((mismatchedPassage.setMap(p_mapB)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addMapPassage(&mismatchedPassage)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Floor: enumerated from mapA, but declares mapB. */
    Floor mismatchedFloor;
    ASSERT_EQ((mismatchedFloor.setId(5)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ((mismatchedFloor.setMap(p_mapB)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addMapFloor(&mismatchedFloor)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Floor: enumerated from mapA, declares no map at all. */
    Floor noMapFloor;
    ASSERT_EQ((noMapFloor.setId(8)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addMapFloor(&noMapFloor)),
              MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    unsigned long mapAId{};
    ASSERT_EQ((p_mapA->getId(mapAId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_snapshotA = findMapSnapshot(snapshot, mapAId);
    ASSERT_NE(p_snapshotA, nullptr);

    const RoomRecord *p_mismatchedRoom = findRoomRecord(*p_snapshotA, 1);
    ASSERT_NE(p_mismatchedRoom, nullptr);
    unsigned long id{};
    ASSERT_EQ((p_mapA->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_mismatchedRoom->key.mapId, id);
    ASSERT_TRUE(p_mismatchedRoom->declaredMapId.has_value());
    unsigned long id2{};
    ASSERT_EQ((p_mapB->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(*p_mismatchedRoom->declaredMapId, id2);

    const RoomRecord *p_noMapRoom = findRoomRecord(*p_snapshotA, 2);
    ASSERT_NE(p_noMapRoom, nullptr);
    EXPECT_FALSE(p_noMapRoom->declaredMapId.has_value());

    const WallRecord *p_mismatchedWallRecord = findWallRecord(*p_snapshotA, 3);
    ASSERT_NE(p_mismatchedWallRecord, nullptr);
    unsigned long id3{};
    ASSERT_EQ((p_mapA->getId(id3)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_mismatchedWallRecord->key.mapId, id3);
    ASSERT_TRUE(p_mismatchedWallRecord->declaredMapId.has_value());
    unsigned long id4{};
    ASSERT_EQ((p_mapB->getId(id4)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(*p_mismatchedWallRecord->declaredMapId, id4);

    const WallRecord *p_noMapWallRecord = findWallRecord(*p_snapshotA, 6);
    ASSERT_NE(p_noMapWallRecord, nullptr);
    EXPECT_FALSE(p_noMapWallRecord->declaredMapId.has_value());

    const PassageRecord *p_noMapPassage = findPassageRecord(*p_snapshotA, 4);
    ASSERT_NE(p_noMapPassage, nullptr);
    EXPECT_FALSE(p_noMapPassage->declaredMapId.has_value());

    const PassageRecord *p_mismatchedPassage =
        findPassageRecord(*p_snapshotA, 7);
    ASSERT_NE(p_mismatchedPassage, nullptr);
    unsigned long id5{};
    ASSERT_EQ((p_mapA->getId(id5)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_mismatchedPassage->key.mapId, id5);
    ASSERT_TRUE(p_mismatchedPassage->declaredMapId.has_value());
    unsigned long id6{};
    ASSERT_EQ((p_mapB->getId(id6)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(*p_mismatchedPassage->declaredMapId, id6);

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
    unsigned long id7{};
    ASSERT_EQ((p_mapA->getId(id7)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_mismatchedFloor->key.mapId, id7);
    ASSERT_TRUE(p_mismatchedFloor->declaredMapId.has_value());
    unsigned long id8{};
    ASSERT_EQ((p_mapB->getId(id8)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(*p_mismatchedFloor->declaredMapId, id8);
    ASSERT_NE(p_noMapFloorRecord, nullptr);
    EXPECT_FALSE(p_noMapFloorRecord->declaredMapId.has_value());
}

/* Minimum-proof item 4: detected-only, candidate-only, and deliberately
 * both-collection room membership remain distinguishable. */
TEST(SemanticGraphSnapshot,
     DetectedCandidateAndBothCollectionMembershipAreDistinguishable)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    Room detectedOnly;
    ASSERT_EQ((detectedOnly.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((detectedOnly.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&detectedOnly)),
              MapStatus::MAP_STATUS_SUCCESS);

    Room candidateOnly;
    ASSERT_EQ((candidateOnly.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((candidateOnly.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addCandidateMapRoom(&candidateOnly)),
              MapStatus::MAP_STATUS_SUCCESS);

    Room bothCollections;
    ASSERT_EQ((bothCollections.setId(3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bothCollections.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&bothCollections)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addCandidateMapRoom(&bothCollections)),
              MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_mapSnapshot = findMapSnapshot(snapshot, mapId);
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
 * cross-map, and wrong geometric::Plane-type relationship targets retain
 * truthful key/reason/liveness/type evidence. */
TEST(SemanticGraphSnapshot,
     RelationshipTargetsRetainTruthfulEvidenceAcrossEveryUnusualCase)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    /* Room with no ground plane at all -- null RawPlaneRef. */
    Room noGroundRoom;
    test::makeRoom(noGroundRoom, 1, p_mapA, nullptr);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&noGroundRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

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
    ASSERT_EQ((unusualGroundRoom.setId(3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((unusualGroundRoom.setMap(p_mapA)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((unusualGroundRoom.setGroundPlane(&unmappedWrongTypeGround)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&unusualGroundRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Room referenced as a "missing-from-enumeration" far-side target: it
     * has a real map but was never added to any Map room collection. */
    Room ghostRoom;
    ASSERT_EQ((ghostRoom.setId(99)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((ghostRoom.setMap(p_mapA)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    /* Room in mapB, referenced cross-map from a Passage enumerated under
     * mapA. */
    Room crossMapRoom;
    ASSERT_EQ((crossMapRoom.setId(7)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((crossMapRoom.setMap(p_mapB)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_mapB->addDetectedMapRoom(&crossMapRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Bad wall, referenced as another room's ground plane, to prove
     * liveness is captured truthfully even for a retired target. */
    geometric::Plane badGroundPlane;
    test::makeGroundPlane(badGroundPlane, 8, p_mapA);
    ASSERT_EQ((p_mapA->addMapPlane(&badGroundPlane)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((badGroundPlane.setBad()),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    Room badGroundOwnerRoom;
    ASSERT_EQ((badGroundOwnerRoom.setId(9)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((badGroundOwnerRoom.setMap(p_mapA)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((badGroundOwnerRoom.setGroundPlane(&badGroundPlane)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&badGroundOwnerRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    ASSERT_EQ((passage.setId(10)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_mapA)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setKnownSideRoom(&ghostRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setProspectiveRoom(&crossMapRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    unsigned long mapAId{};
    ASSERT_EQ((p_mapA->getId(mapAId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_snapshotA = findMapSnapshot(snapshot, mapAId);
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
              geometric::Plane::PlaneVariant::WALL);
    EXPECT_TRUE(p_unusualGroundRoom->groundPlaneRef.isLive);

    /* Missing-from-enumeration: the key is still truthfully reported, even
     * though no RoomRecord for it exists in the snapshot -- and so is its
     * liveness (captured directly from the pointer, not by joining against
     * the snapshot's own record vectors). */
    const PassageRecord *p_passageRecord = findPassageRecord(*p_snapshotA, 10);
    ASSERT_NE(p_passageRecord, nullptr);
    ASSERT_TRUE(p_passageRecord->knownSideRoomRef.key.has_value());
    EXPECT_EQ(p_passageRecord->knownSideRoomRef.key->entityId, 99);
    unsigned long id{};
    ASSERT_EQ((p_mapA->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_passageRecord->knownSideRoomRef.key->mapId, id);
    EXPECT_EQ(findRoomRecord(*p_snapshotA, 99), nullptr);
    ASSERT_TRUE(p_passageRecord->knownSideRoomRef.isLive.has_value());
    EXPECT_TRUE(*p_passageRecord->knownSideRoomRef.isLive);

    /* Cross-map: the referenced room's key names mapB, distinct from the
     * passage record's own mapA key. */
    ASSERT_TRUE(p_passageRecord->prospectiveRoomRef.key.has_value());
    unsigned long id2{};
    ASSERT_EQ((p_mapB->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_passageRecord->prospectiveRoomRef.key->mapId, id2);
    EXPECT_NE(p_passageRecord->prospectiveRoomRef.key->mapId,
              p_passageRecord->key.mapId);

    /* Bad target retains liveness truthfully rather than being hidden. */
    const RoomRecord *p_badGroundOwnerRoom = findRoomRecord(*p_snapshotA, 9);
    ASSERT_NE(p_badGroundOwnerRoom, nullptr);
    EXPECT_EQ(p_badGroundOwnerRoom->groundPlaneRef.reason,
              UnavailableReason::NONE);
    EXPECT_FALSE(p_badGroundOwnerRoom->groundPlaneRef.isLive);
}

/* Tri-state EntityRef liveness: default reference invariants are valid,
 * and unknown liveness is never encoded as true. */
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

/* Regression coverage: entityRefForWall() used
 * to label every referenced geometric::Plane as EntityKind::WALL without
 * checking its real geometric::Plane::PlaneVariant, fabricating a WallRecord
 * identity for a non-WALL target. Wall-shaped references (a wall's twin face, a
 * Room's owned walls, a Passage's associated walls) always use RawPlaneRef, so
 * a wrong-type target retains its true planeType/isLive/mapId instead of a
 * fabricated WALL key. */
TEST(SemanticGraphSnapshot,
     WrongTypePlaneTargetsInWallShapedReferencesRetainTruthfulEvidence)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    /* A live GROUND-typed plane, never a WallRecord in this snapshot. */
    geometric::Plane groundNotWall;
    ASSERT_TRUE(test::makeGroundPlane(groundNotWall, 1, p_map));
    ASSERT_EQ((p_map->addMapPlane(&groundNotWall)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Case 1: a genuine wall whose twinFace_ is wrongly set to the
     * GROUND-typed plane (geometric::Plane::setTwinFace() has no type check, so
     * this is a real reachable model state, not a fabricated test-only shape).
     */
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
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((wall.setTwinFace(&groundNotWall)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);

    /* Case 2: a Room whose getWalls() names the GROUND-typed plane (e.g. a
     * wall-detection contract violation upstream; nothing in Room::setWalls()
     * enforces WALL-typed membership). */
    Room room;
    ASSERT_EQ((room.setId(3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(&groundNotWall)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Case 3: a Passage whose getAssociateWalls() names the same
     * GROUND-typed plane. */
    Passage passage;
    ASSERT_EQ((passage.setId(4)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.addAssociateWall(&groundNotWall)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_mapSnapshot = findMapSnapshot(snapshot, mapId);
    ASSERT_NE(p_mapSnapshot, nullptr);

    /* The GROUND-typed plane must never appear as a WallRecord. */
    EXPECT_EQ(findWallRecord(*p_mapSnapshot, 1), nullptr);

    const WallRecord *p_wallRecord = findWallRecord(*p_mapSnapshot, 2);
    ASSERT_NE(p_wallRecord, nullptr);
    EXPECT_EQ(p_wallRecord->twinRef.reason, UnavailableReason::NONE);
    EXPECT_EQ(p_wallRecord->twinRef.planeId, 1);
    EXPECT_TRUE(p_wallRecord->twinRef.isLive);
    EXPECT_EQ(p_wallRecord->twinRef.planeType,
              geometric::Plane::PlaneVariant::GROUND);
    EXPECT_FALSE(p_wallRecord->twinRef.wallKey.has_value());

    const RoomRecord *p_roomRecord = findRoomRecord(*p_mapSnapshot, 3);
    ASSERT_NE(p_roomRecord, nullptr);
    ASSERT_EQ(p_roomRecord->wallRefs.size(), 1U);
    EXPECT_EQ(p_roomRecord->wallRefs[0].planeId, 1);
    EXPECT_EQ(p_roomRecord->wallRefs[0].planeType,
              geometric::Plane::PlaneVariant::GROUND);
    EXPECT_FALSE(p_roomRecord->wallRefs[0].wallKey.has_value());

    const PassageRecord *p_passageRecord = findPassageRecord(*p_mapSnapshot, 4);
    ASSERT_NE(p_passageRecord, nullptr);
    ASSERT_EQ(p_passageRecord->associateWallRefs.size(), 1U);
    EXPECT_EQ(p_passageRecord->associateWallRefs[0].planeId, 1);
    EXPECT_EQ(p_passageRecord->associateWallRefs[0].planeType,
              geometric::Plane::PlaneVariant::GROUND);
    EXPECT_FALSE(p_passageRecord->associateWallRefs[0].wallKey.has_value());
}

/* Regression coverage for tri-state liveness:
 * entityRefForRoom()/entityRefForFloor() used to discard the target's own
 * local id (and, for Room, liveness) the moment it had no declared map.
 * Floor's liveness, which the model cannot expose at all (Floor has no
 * isBad()), is reported as explicitly unknown -- never fabricated as
 * true. */
TEST(SemanticGraphSnapshot,
     UnmappedNonNullRoomAndFloorReferencesRetainLocalIdentityAndLiveness)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    /* An unmapped, bad, non-null Room, referenced from a Passage. */
    Room unmappedBadRoom;
    ASSERT_EQ((unmappedBadRoom.setId(42)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((unmappedBadRoom.setBad()),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    Passage passage;
    ASSERT_EQ((passage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setProspectiveRoom(&unmappedBadRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);

    /* An unmapped, non-null Floor, referenced from a Room. */
    Floor unmappedFloor;
    ASSERT_EQ((unmappedFloor.setId(7)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);

    Room room;
    ASSERT_EQ((room.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setFloor(&unmappedFloor)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_mapSnapshot = findMapSnapshot(snapshot, mapId);
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
     * fabricated as true. */
    EXPECT_FALSE(p_roomRecord->floorRef.isLive.has_value());
    EXPECT_EQ(p_roomRecord->floorRef.livenessUnavailableReason,
              UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA);
}

/* Regression coverage: captureSemanticGraphSnapshot() used to call
 * Atlas::GetCurrentMap(), which creates a new map as a side effect whenever
 * mpCurrentMap is null -- reachable in production via Atlas::clearAtlas().
 * Capture must never mutate an Atlas in that state, and must report the
 * coherent-view status truthfully for it. */
TEST(SemanticGraphSnapshot, CaptureAfterAtlasClearedDoesNotCreateAMap)
{
    Atlas atlas(0);
    ASSERT_EQ((atlas.clearAtlas()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    std::vector<Map *> allMaps{};
    ASSERT_EQ((atlas.getAllMaps(allMaps)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(allMaps.size(), 0U);

    SemanticGraphSnapshot snapshot;
    {
        std::unique_lock<std::mutex> lock{};
        ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        SemanticGraphSnapshot snapshot2{};
        ASSERT_EQ((captureSemanticGraphSnapshot(&atlas, snapshot2)),
                  SemanticGraphSnapshotStatus::
                      SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
        snapshot = snapshot2;
    }

    EXPECT_FALSE(snapshot.currentMapId.has_value());
    EXPECT_EQ(snapshot.currentMapStatus, AtlasCurrentMapStatus::NO_CURRENT_MAP);
    EXPECT_TRUE(snapshot.maps.empty());
    /* The real proof: capture must not have created a map as a side
     * effect, unlike calling Atlas::GetCurrentMap() would have. */
    std::vector<Map *> allMaps2{};
    ASSERT_EQ((atlas.getAllMaps(allMaps2)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_EQ(allMaps2.size(), 0U);
}

/* Atlas::SetMapBad(currentMap) erases the map from the active
 * set and marks it bad without clearing Atlas::mpCurrentMap; a later
 * Atlas::ChangeMap() call is what eventually installs a replacement. Between
 * those two events, the snapshot must truthfully report the current map as
 * absent from its own active-map list, via AtlasCurrentMapStatus, rather
 * than silently claiming a consistency invariant that does not hold. */
TEST(SemanticGraphSnapshot,
     CurrentMapMarkedBadBeforeChangeMapIsReportedAsNotActive)
{
    Atlas atlas(0);
    Map  *p_currentMap = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_currentMap)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    unsigned long currentMapIdValue{};
    ASSERT_EQ((p_currentMap->getId(currentMapIdValue)),
              MapStatus::MAP_STATUS_SUCCESS);
    const long unsigned int currentMapId =
        static_cast<long unsigned int>(currentMapIdValue);

    ASSERT_EQ((atlas.setMapBad(p_currentMap)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    /* No ChangeMap() call yet: mpCurrentMap still points at the now-bad,
     * now-inactive map. */

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    ASSERT_TRUE(snapshot.currentMapId.has_value());
    EXPECT_EQ(*snapshot.currentMapId, currentMapId);
    EXPECT_EQ(snapshot.currentMapStatus,
              AtlasCurrentMapStatus::CURRENT_MAP_NOT_ACTIVE);
    EXPECT_EQ(findMapSnapshot(snapshot, currentMapId), nullptr)
        << "the bad current map must not appear in the active map list";
    EXPECT_TRUE(snapshot.maps.empty());
}

/* Coherence counterpart: an ordinary current map (no
 * SetMapBad() call) is reported as active and appears in maps. */
TEST(SemanticGraphSnapshot, CurrentMapStatusIsActiveForAnOrdinaryCurrentMap)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    ASSERT_TRUE(snapshot.currentMapId.has_value());
    unsigned long id{};
    ASSERT_EQ((p_map->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(*snapshot.currentMapId, id);
    EXPECT_EQ(snapshot.currentMapStatus,
              AtlasCurrentMapStatus::CURRENT_MAP_ACTIVE);
    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_mapSnapshot = findMapSnapshot(snapshot, mapId);
    ASSERT_NE(p_mapSnapshot, nullptr);
    EXPECT_TRUE(p_mapSnapshot->isCurrentMap);
}

/* The schema's still-missing-in-the-foundation-slice fields
 * (quarantine/history, manager-private open-passage and unresolved-wall
 * hypotheses, and per-wall quarantine/observation-ray evidence) report
 * their documented unavailable reason by default, never a fabricated
 * value. */
TEST(SemanticGraphSnapshot,
     ManagerPrivateAndHistoryUnavailableDefaultsAreReported)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    EXPECT_EQ(snapshot.roomContextHistoryReason,
              UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE);
    EXPECT_EQ(snapshot.managerPrivateOpenPassageHypothesesReason,
              UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE);
    EXPECT_EQ(snapshot.managerPrivateUnresolvedWallHypothesesReason,
              UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE);

    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_mapSnapshot = findMapSnapshot(snapshot, mapId);
    ASSERT_NE(p_mapSnapshot, nullptr);
    const WallRecord *p_wallRecord = findWallRecord(*p_mapSnapshot, 1);
    ASSERT_NE(p_wallRecord, nullptr);
    EXPECT_EQ(p_wallRecord->quarantineReason,
              UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA);
    EXPECT_EQ(p_wallRecord->observationRayEvidenceReason,
              UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA);
}

/* Value-based determinism: distinct source objects captured with a
 * colliding EntityKey (same kind, mapId, and local id) must both be
 * retained, never deduplicated, and the pair's relative order must be a
 * function of their captured VALUES (via isValueLessForCollisionTiebreak()),
 * not of insertion/pointer order -- proven here by capturing the identical
 * pair through two atlases built with reversed Room-construction order and
 * asserting both captures agree on the same value-determined order. */
TEST(SemanticGraphSnapshot, CollidingEntityKeyRoomsAreBothRetained)
{
    auto buildAndCapture = [](bool constructLowerCentroidFirst_in)
    {
        Atlas atlas(0);
        Map  *p_map = nullptr;
        if (atlas.getCurrentMap(p_map) != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        std::unique_ptr<Room> p_lowerCentroidRoom = std::make_unique<Room>();
        if (p_lowerCentroidRoom->setId(1) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_lowerCentroidRoom->setMap(p_map) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_lowerCentroidRoom->setCentroid(Eigen::Vector3d(1.0, 0.0, 0.0)) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        std::unique_ptr<Room> p_higherCentroidRoom = std::make_unique<Room>();
        if (p_higherCentroidRoom->setId(1) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_higherCentroidRoom->setMap(p_map) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_higherCentroidRoom->setCentroid(Eigen::Vector3d(2.0, 0.0, 0.0)) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (constructLowerCentroidFirst_in)
        {
            if (p_map->addDetectedMapRoom(p_lowerCentroidRoom.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addDetectedMapRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_map->addDetectedMapRoom(p_higherCentroidRoom.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addDetectedMapRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {
            if (p_map->addDetectedMapRoom(p_higherCentroidRoom.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addDetectedMapRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_map->addDetectedMapRoom(p_lowerCentroidRoom.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addDetectedMapRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        std::unique_lock<std::mutex> lock{};
        if (atlas.acquireSemanticUpdateLock(lock) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: acquireSemanticUpdateLock returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        SemanticGraphSnapshot snapshot{};
        if (captureSemanticGraphSnapshot(&atlas, snapshot) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: captureSemanticGraphSnapshot returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        return snapshot;
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

/* White-box: the same value-based collision determinism as
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
 * through the production insertion path -- so the collision is supplied
 * directly to sortByKey<RecordT>() (the same template
 * captureSemanticGraphSnapshot() uses for every one of these four record
 * types), sorting/capturing logically identical colliding records supplied
 * in opposite orders and comparing every resulting value. */
TEST(SemanticGraphSnapshot,
     CollidingWallRecordsAreBothRetainedAndDeterministicallyOrdered)
{
    WallRecord badWall;
    EntityKey  key2{};
    ASSERT_EQ(
        (makeKey(EntityKind::WALL, 1U, 7, key2)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    badWall.key    = key2;
    badWall.isLive = false;

    WallRecord liveWall;
    EntityKey  key3{};
    ASSERT_EQ(
        (makeKey(EntityKind::WALL, 1U, 7, key3)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    liveWall.key    = key3;
    liveWall.isLive = true;

    std::vector<WallRecord> ascending  = {badWall, liveWall};
    std::vector<WallRecord> descending = {liveWall, badWall};
    ASSERT_EQ(
        (sortByKey(ascending)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(
        (sortByKey(descending)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

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
    EntityKey     key2{};
    ASSERT_EQ(
        (makeKey(EntityKind::PASSAGE, 1U, 7, key2)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    narrowPassage.key     = key2;
    narrowPassage.width_m = 0.5;

    PassageRecord widePassage;
    EntityKey     key3{};
    ASSERT_EQ(
        (makeKey(EntityKind::PASSAGE, 1U, 7, key3)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    widePassage.key     = key3;
    widePassage.width_m = 1.5;

    std::vector<PassageRecord> ascending  = {narrowPassage, widePassage};
    std::vector<PassageRecord> descending = {widePassage, narrowPassage};
    ASSERT_EQ(
        (sortByKey(ascending)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(
        (sortByKey(descending)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

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
    EntityKey   key2{};
    ASSERT_EQ(
        (makeKey(EntityKind::FLOOR, 1U, 7, key2)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    lowerFloor.key              = key2;
    lowerFloor.centroid_World_m = Eigen::Vector3d(0.0, 0.0, 0.0);

    FloorRecord higherFloor;
    EntityKey   key3{};
    ASSERT_EQ(
        (makeKey(EntityKind::FLOOR, 1U, 7, key3)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    higherFloor.key              = key3;
    higherFloor.centroid_World_m = Eigen::Vector3d(0.0, 0.0, 3.0);

    std::vector<FloorRecord> ascending  = {lowerFloor, higherFloor};
    std::vector<FloorRecord> descending = {higherFloor, lowerFloor};
    ASSERT_EQ(
        (sortByKey(ascending)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(
        (sortByKey(descending)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

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

/* "duplicate relationship references": two distinct rooms
 * sharing a colliding EntityKey both own the same wall. The wall's
 * ownerRoomRefs must retain both -- not deduplicate them merely because
 * they carry the same key -- since they are genuinely different source
 * objects with different liveness. */
TEST(SemanticGraphSnapshot, CollidingOwnerRoomRefsForOneWallAreRetained)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPlane(&sharedWall)), MapStatus::MAP_STATUS_SUCCESS);

    Room liveOwner;
    test::makeRoom(liveOwner, 2, p_map, &sharedWall, Eigen::Vector3d(1, 0, 1));
    ASSERT_EQ((p_map->addDetectedMapRoom(&liveOwner)),
              MapStatus::MAP_STATUS_SUCCESS);

    Room badOwnerSameId;
    test::makeRoom(badOwnerSameId,
                   2,
                   p_map,
                   &sharedWall,
                   Eigen::Vector3d(-1, 0, 1));
    ASSERT_EQ((badOwnerSameId.setBad()),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&badOwnerSameId)),
              MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_mapSnapshot = findMapSnapshot(snapshot, mapId);
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

/* WallRecord::ownerRoomRefs' key must come from the containing map used to
 * enumerate the owning room (matching that room's own RoomRecord::key),
 * never from the room's own possibly-different declared map --
 * captureSemanticGraphSnapshot.cc's wall-ownership inversion pass builds
 * this key from its own per-map loop variable, not from
 * entityRefForRoom()'s declared-map semantics, specifically to keep this
 * fact true even for a mismatched room. */
TEST(SemanticGraphSnapshot,
     OwnerRoomRefIsKeyedByContainingMapEvenWhenRoomDeclaresADifferentMap)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);

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
    ASSERT_EQ((p_mapA->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    /* Enumerated from mapA (AddDetectedMapRoom), but declares mapB. */
    Room mismatchedOwner;
    ASSERT_EQ((mismatchedOwner.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((mismatchedOwner.setMap(p_mapB)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((mismatchedOwner.setWalls(&wall)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&mismatchedOwner)),
              MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    unsigned long mapAId{};
    ASSERT_EQ((p_mapA->getId(mapAId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_mapSnapshotA = findMapSnapshot(snapshot, mapAId);
    ASSERT_NE(p_mapSnapshotA, nullptr);
    const RoomRecord *p_ownerRecord = findRoomRecord(*p_mapSnapshotA, 2);
    ASSERT_NE(p_ownerRecord, nullptr);
    unsigned long id{};
    ASSERT_EQ((p_mapA->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_ownerRecord->key.mapId, id);
    ASSERT_TRUE(p_ownerRecord->declaredMapId.has_value());
    unsigned long id2{};
    ASSERT_EQ((p_mapB->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(*p_ownerRecord->declaredMapId, id2);

    const WallRecord *p_wallRecord = findWallRecord(*p_mapSnapshotA, 1);
    ASSERT_NE(p_wallRecord, nullptr);
    ASSERT_EQ(p_wallRecord->ownerRoomRefs.size(), 1U);
    ASSERT_TRUE(p_wallRecord->ownerRoomRefs[0].key.has_value());
    /* The owner ref's key must equal the owner's own RoomRecord::key
     * (mapA-qualified), not a mapB-qualified key derived from the room's
     * own declared map. */
    EXPECT_EQ(*p_wallRecord->ownerRoomRefs[0].key, p_ownerRecord->key);
    unsigned long id3{};
    ASSERT_EQ((p_mapA->getId(id3)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_wallRecord->ownerRoomRefs[0].key->mapId, id3);
}

/* Minimum-proof item 7: bad room owners and duplicate/colliding relationship
 * evidence are retained. */
TEST(SemanticGraphSnapshot, BadRoomOwnersAndDuplicateWallOwnershipAreRetained)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPlane(&sharedWall)), MapStatus::MAP_STATUS_SUCCESS);

    /* Two distinct, live rooms both claim the same wall -- a real AX-WALL-01
     * violation that capture must retain, not silently resolve. */
    Room firstOwner;
    test::makeRoom(firstOwner, 2, p_map, &sharedWall, Eigen::Vector3d(1, 0, 1));
    ASSERT_EQ((p_map->addDetectedMapRoom(&firstOwner)),
              MapStatus::MAP_STATUS_SUCCESS);

    Room secondOwner;
    test::makeRoom(secondOwner,
                   3,
                   p_map,
                   &sharedWall,
                   Eigen::Vector3d(-1, 0, 1));
    ASSERT_EQ((p_map->addDetectedMapRoom(&secondOwner)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* A retired (bad) room that still lists the wall. */
    Room badOwner;
    test::makeRoom(badOwner, 4, p_map, &sharedWall, Eigen::Vector3d(0, 1, 1));
    ASSERT_EQ((badOwner.setBad()),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&badOwner)),
              MapStatus::MAP_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPlane(&doublyRegisteredOwnerWall)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room doublyRegisteredOwner;
    test::makeRoom(doublyRegisteredOwner,
                   6,
                   p_map,
                   &doublyRegisteredOwnerWall,
                   Eigen::Vector3d(2, 0, 1));
    ASSERT_EQ((p_map->addDetectedMapRoom(&doublyRegisteredOwner)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addCandidateMapRoom(&doublyRegisteredOwner)),
              MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_mapSnapshot = findMapSnapshot(snapshot, mapId);
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
        Atlas atlas(0);
        Map  *p_map = nullptr;
        if (atlas.getCurrentMap(p_map) != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<std::unique_ptr<Room>> rooms;
        for (int roomId : roomIdInsertionOrder_in)
        {
            std::unique_ptr<Room> p_room = std::make_unique<Room>();
            if (p_room->setId(roomId) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room->setMap(p_map) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_map->addDetectedMapRoom(p_room.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addDetectedMapRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            rooms.push_back(std::move(p_room));
        }
        std::unique_lock<std::mutex> lock{};
        if (atlas.acquireSemanticUpdateLock(lock) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: acquireSemanticUpdateLock returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        SemanticGraphSnapshot snapshot{};
        if (captureSemanticGraphSnapshot(&atlas, snapshot) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: captureSemanticGraphSnapshot returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        return snapshot;
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
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    ASSERT_EQ((room.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(3.0, 4.0, 5.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    {
        std::unique_lock<std::mutex> lock{};
        ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        SemanticGraphSnapshot snapshot{};
        ASSERT_EQ((captureSemanticGraphSnapshot(&atlas, snapshot)),
                  SemanticGraphSnapshotStatus::
                      SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    }
    Eigen::Vector3d centroid{};
    ASSERT_EQ((room.getCentroid(centroid)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(centroid, Eigen::Vector3d(3.0, 4.0, 5.0));
    bool isBad2{};
    ASSERT_EQ((room.isBad(isBad2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(isBad2);
    std::vector<semantic::Room *> allDetectedMapRooms{};
    ASSERT_EQ((p_map->getAllDetectedMapRooms(allDetectedMapRooms)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(allDetectedMapRooms.size(), 1U);
}

/* "ordering of maps and every record/relationship collection":
 * extends coverage beyond rooms (above) to walls, passages, floors, and the
 * top-level maps vector, each for permuted insertion order. A genuine map-
 * id collision is not exercised: Atlas::Map::nextId is a monotonically
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
        Map  *p_mapA = nullptr;
        if (atlas.getCurrentMap(p_mapA) != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        /* A second, otherwise-unused map only to give the top-level maps
         * vector two entries to check std::is_sorted() over. */
        if (atlas.createNewMap() != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: createNewMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        std::vector<std::unique_ptr<geometric::Plane>> walls;
        std::vector<std::unique_ptr<Passage>>          passages;
        std::vector<std::unique_ptr<Floor>>            floors;
        for (int index = 0; index < 3; ++index)
        {
            const int entityId =
                ascendingInsertionOrder_in ? index + 1 : 3 - index;

            std::unique_ptr<geometric::Plane> p_wall =
                std::make_unique<geometric::Plane>();
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
            if (p_mapA->addMapPlane(p_wall.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            walls.push_back(std::move(p_wall));

            std::unique_ptr<Passage> p_passage = std::make_unique<Passage>();
            if (p_passage->setId(entityId) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage->setMap(p_mapA) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapA->addMapPassage(p_passage.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPassage returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            passages.push_back(std::move(p_passage));

            std::unique_ptr<Floor> p_floor = std::make_unique<Floor>();
            if (p_floor->setId(entityId) !=
                vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_floor->setMap(p_mapA) !=
                vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapA->addMapFloor(p_floor.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapFloor returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            floors.push_back(std::move(p_floor));
        }

        std::unique_lock<std::mutex> lock{};
        if (atlas.acquireSemanticUpdateLock(lock) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: acquireSemanticUpdateLock returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        SemanticGraphSnapshot snapshot{};
        if (captureSemanticGraphSnapshot(&atlas, snapshot) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: captureSemanticGraphSnapshot returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        unsigned long mapAId{};
        if (p_mapA->getId(mapAId) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        return std::make_pair(std::move(snapshot), mapAId);
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

/* "ordering of ... every record/relationship collection",
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
        Map  *p_map = nullptr;
        if (atlas.getCurrentMap(p_map) != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        std::vector<std::unique_ptr<geometric::Plane>> walls;
        for (int index = 0; index < 3; ++index)
        {
            std::unique_ptr<geometric::Plane> p_wall =
                std::make_unique<geometric::Plane>();
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
            if (p_map->addMapPlane(p_wall.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            walls.push_back(std::move(p_wall));
        }

        std::vector<std::unique_ptr<Passage>> passages;
        for (int index = 0; index < 3; ++index)
        {
            std::unique_ptr<Passage> p_passage = std::make_unique<Passage>();
            if (p_passage->setId(index + 1) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage->setMap(p_map) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_map->addMapPassage(p_passage.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPassage returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            passages.push_back(std::move(p_passage));
        }

        std::vector<std::unique_ptr<Room>> owningRooms;
        for (int index = 0; index < 3; ++index)
        {
            std::unique_ptr<Room> p_room = std::make_unique<Room>();
            if (p_room->setId(index + 20) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room->setMap(p_map) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            owningRooms.push_back(std::move(p_room));
        }

        std::unique_ptr<geometric::Plane> p_sharedWall =
            std::make_unique<geometric::Plane>();
        test::makeWallPlane(*p_sharedWall,
                            4,
                            p_map,
                            Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                            Eigen::Vector3d::UnitY(),
                            Eigen::Vector3d::UnitZ(),
                            1.0,
                            1.0,
                            Eigen::Vector3d(4.0, 0.0, 1.0));
        if (p_map->addMapPlane(p_sharedWall.get()) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPlane returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        Room subjectRoom;
        if (subjectRoom.setId(10) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (subjectRoom.setMap(p_map) !=
            vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        Floor subjectFloor;
        if (subjectFloor.setId(30) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (subjectFloor.setMap(p_map) !=
            vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        Passage subjectPassage;
        if (subjectPassage.setId(40) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (subjectPassage.setMap(p_map) !=
            vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        const std::vector<int> memberOrder = ascendingInsertionOrder_in
                                                 ? std::vector<int>{0, 1, 2}
                                                 : std::vector<int>{2, 1, 0};
        for (int index : memberOrder)
        {
            const std::size_t memberIndex = static_cast<std::size_t>(index);
            if (subjectRoom.setWalls(walls[memberIndex].get()) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (subjectRoom.setDoorways(passages[memberIndex].get()) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setDoorways returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (owningRooms[memberIndex]->setWalls(p_sharedWall.get()) !=
                vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (subjectFloor.addRoom(owningRooms[memberIndex].get()) !=
                vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addRoom returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (subjectPassage.addAssociateWall(walls[memberIndex].get()) !=
                vs_graphs::core::semantic::PassageStatus::
                    PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addAssociateWall returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        if (p_map->addDetectedMapRoom(&subjectRoom) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addDetectedMapRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (const std::unique_ptr<Room> &p_owningRoom : owningRooms)
        {
            if (p_map->addDetectedMapRoom(p_owningRoom.get()) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addDetectedMapRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        if (p_map->addMapFloor(&subjectFloor) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapFloor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_map->addMapPassage(&subjectPassage) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPassage returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        std::unique_lock<std::mutex> lock{};
        if (atlas.acquireSemanticUpdateLock(lock) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: acquireSemanticUpdateLock returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        SemanticGraphSnapshot snapshot{};
        if (captureSemanticGraphSnapshot(&atlas, snapshot) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: captureSemanticGraphSnapshot returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        return snapshot;
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
 * does not try to reacquire it. Atlas::semanticUpdateMutex is a plain,
 * non-recursive std::mutex, so if captureSemanticGraphSnapshot() ever tried
 * to acquire it again on this thread, this test would deadlock rather than
 * fail cleanly -- reaching the final assertion is itself the proof. */
TEST(SemanticGraphSnapshot,
     CaptureUnderHeldSemanticLockCompletesWithoutReacquiring)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    ASSERT_EQ((room.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    ASSERT_EQ(snapshot.maps.size(), 1U);
    EXPECT_EQ(snapshot.maps[0].rooms.size(), 1U);
}

/* Minimum-proof item 10: the cheap geometric::Plane accessor agrees with the
 * scalar fields in the full geometry snapshot while returning no cloud payload.
 */
TEST(SemanticGraphSnapshot,
     CheapPlaneAccessorAgreesWithFullGeometrySnapshotScalars)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    geometric::Plane::GeometrySnapshot fullSnapshot{};
    ASSERT_EQ((wall.getGeometrySnapshot(fullSnapshot)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    geometric::PlaneGeometryMetadataSnapshot metadata{};
    ASSERT_EQ((wall.getGeometryMetadataSnapshot(metadata)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);

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

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    const MapSnapshot *p_mapSnapshot = findMapSnapshot(snapshot, mapId);
    ASSERT_NE(p_mapSnapshot, nullptr);
    const WallRecord *p_wallRecord = findWallRecord(*p_mapSnapshot, 1);
    ASSERT_NE(p_wallRecord, nullptr);

    /* Every one of the 10 geometric::PlaneGeometryMetadataSnapshot fields, not
     * just a convenient subset -- a field-swap bug touching any single one of
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
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(nullptr, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    EXPECT_FALSE(snapshot.currentMapId.has_value());
    EXPECT_EQ(snapshot.currentMapStatus, AtlasCurrentMapStatus::NO_CURRENT_MAP);
    EXPECT_TRUE(snapshot.maps.empty());
}

/* "helper-level null/drop test": appendWallRef()/
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
    ASSERT_EQ(
        (appendWallRef(nullptr, wallRefs)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    EXPECT_TRUE(wallRefs.empty());

    std::vector<EntityRef> roomRefs;
    ASSERT_EQ(
        (appendRoomRef(nullptr, roomRefs)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    EXPECT_TRUE(roomRefs.empty());

    std::vector<EntityRef> passageRefs;
    ASSERT_EQ(
        (appendPassageRef(nullptr, passageRefs)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    EXPECT_TRUE(passageRefs.empty());
}

/* "finite, NaN, infinity and signed-zero ordering": white-box
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
