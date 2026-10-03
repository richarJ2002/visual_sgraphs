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

/*!
 * @brief           Checks that a captured snapshot holds plain values, not
 *                  pointers into the model, so a room and wall still read
 *                  correctly after the source objects and atlas are destroyed.
 */
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
    EXPECT_EQ(p_mapSnapshot->rooms[0].roomCentroid_world_m,
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

/*!
 * @brief           Checks that the wall, passage, floor and owner references
 *                  inside a snapshot's relationship lists stay valid after the
 *                  source objects are destroyed.
 *
 *                  Covers RoomRecord::wallRefs and passageRefs,
 *                  WallRecord::ownerRoomRefs and FloorRecord::roomRefs, which
 *                  hold EntityRef or RawPlaneRef values rather than bare keys.
 */
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

/*!
 * @brief           Checks that rooms, walls, passages and floors that share a
 *                  local id in two maps stay distinct in the snapshot because
 *                  their keys include the map id.
 */
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

/*!
 * @brief           Checks that for rooms, walls, passages and floors the
 *                  snapshot records both the map that contains the entity and
 *                  the map it declares, including a different declared map and
 *                  none at all.
 */
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

/*!
 * @brief           Checks that rooms held only in the detected set, only in the
 *                  candidate set, or in both are told apart by their membership
 *                  flags.
 */
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

/*!
 * @brief           Checks that a relationship pointing at a null, unmapped,
 *                  retired, unlisted, other-map or wrong-type target still
 *                  records the true key, reason, liveness and plane type.
 */
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

/*!
 * @brief           Checks that a default-constructed entity reference and raw
 *                  plane reference report a null-reference reason and unknown
 *                  liveness, never a live target.
 */
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

/*!
 * @brief           Checks that a non-wall plane named as a wall's twin, a
 *                  room's wall or a passage's wall keeps its true plane type
 *                  and is never recorded as a wall.
 *
 *                  Regression test: entityRefForWall() once labelled every
 *                  referenced plane a wall without checking its plane type.
 *                  Wall-shaped references now use RawPlaneRef, which keeps the
 *                  true type.
 */
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

/*!
 * @brief           Checks that references to a room or floor with no map keep
 *                  the target's local id, and that liveness is known for the
 *                  room but explicitly unknown for the floor.
 *
 *                  Regression test: entityRefForRoom() and entityRefForFloor()
 *                  once dropped the local id (and, for a room, the liveness) of
 *                  a target with no map. A floor has no bad flag, so its
 *                  liveness is reported as unknown, never as live.
 */
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

/*!
 * @brief           Checks that capturing a snapshot from an atlas with no maps
 *                  reports no current map and does not create one.
 *
 *                  Regression test: capture once called Atlas::getCurrentMap(),
 *                  which creates a map when there is none. An atlas without
 *                  maps is reachable in production through Atlas::clearAtlas().
 */
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

/*!
 * @brief           Checks that a current map marked bad before a map change is
 *                  reported as not active and is left out of the snapshot's map
 *                  list.
 *
 *                  Atlas::setMapBad() removes the current map from the active
 *                  set but keeps it as the current map until Atlas::changeMap()
 *                  installs a replacement. In between, the snapshot must say so
 *                  through AtlasCurrentMapStatus.
 */
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

/*!
 * @brief           Checks that an ordinary current map is reported as active
 *                  and flagged as the current map in the snapshot.
 */
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

/*!
 * @brief           Checks that snapshot fields not yet captured report their
 *                  documented unavailable reasons by default rather than
 *                  invented values.
 *
 *                  Covers quarantine and history, the semantics manager's
 *                  private open-passage and unresolved-wall hypotheses, and the
 *                  per-wall quarantine and observation-ray evidence.
 */
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

/*!
 * @brief           Checks that two rooms with the same entity key are both kept
 *                  and are ordered by their values, whichever was registered
 *                  first.
 *
 *                  Captures the same pair through two atlases built in opposite
 *                  orders: both captures must give the order that
 *                  isValueLessForCollisionTiebreak() decides from the values,
 *                  not the insertion or pointer order.
 */
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
         * rooms, so roomCentroid_world_m is the first differentiator --
         * (1,0,0) sorts before (2,0,0) regardless of which object was
         * constructed or registered first. */
        EXPECT_EQ(p_capture->maps[0].rooms[0].roomCentroid_world_m,
                  Eigen::Vector3d(1.0, 0.0, 0.0));
        EXPECT_EQ(p_capture->maps[0].rooms[1].roomCentroid_world_m,
                  Eigen::Vector3d(2.0, 0.0, 0.0));
    }
}

/*!
 * @brief           Checks that two wall records with the same key are both kept
 *                  and sorted by value, bad before live, whatever the input
 *                  order.
 *
 *                  Calls sortByKey() directly, the template the capture uses
 *                  for all four record types. Walls, passages and floors cannot
 *                  collide through the map: Map::addMapPlane() and
 *                  Map::addMapFloor() give a colliding object a new id and
 *                  Map::addMapPassage() refuses it. So the colliding records
 *                  are sorted directly, in both input orders.
 */
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

/*!
 * @brief           Checks that two passage records with the same key are both
 *                  kept and sorted by value, narrower before wider, whatever
 *                  the input order.
 */
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
        /* isLive, declaredMapId, passageType, planeEquation_world, and
         * passageCentroid_world_m are all equal defaults for both; width_m is
         * the first differentiator. */
        EXPECT_DOUBLE_EQ((*p_records)[0].width_m, 0.5);
        EXPECT_DOUBLE_EQ((*p_records)[1].width_m, 1.5);
    }
}

/*!
 * @brief           Checks that two floor records with the same key are both
 *                  kept and sorted by value, lower centroid before higher,
 *                  whatever the input order.
 */
TEST(SemanticGraphSnapshot,
     CollidingFloorRecordsAreBothRetainedAndDeterministicallyOrdered)
{
    FloorRecord lowerFloor;
    EntityKey   key2{};
    ASSERT_EQ(
        (makeKey(EntityKind::FLOOR, 1U, 7, key2)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    lowerFloor.key                   = key2;
    lowerFloor.floorCentroid_world_m = Eigen::Vector3d(0.0, 0.0, 0.0);

    FloorRecord higherFloor;
    EntityKey   key3{};
    ASSERT_EQ(
        (makeKey(EntityKind::FLOOR, 1U, 7, key3)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    higherFloor.key                   = key3;
    higherFloor.floorCentroid_world_m = Eigen::Vector3d(0.0, 0.0, 3.0);

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
        EXPECT_EQ((*p_records)[0].floorCentroid_world_m,
                  Eigen::Vector3d(0.0, 0.0, 0.0));
        EXPECT_EQ((*p_records)[1].floorCentroid_world_m,
                  Eigen::Vector3d(0.0, 0.0, 3.0));
    }
}

/*!
 * @brief           Checks that two rooms with the same key that own one wall
 *                  both stay in the wall's owner list, ordered with the bad one
 *                  before the live one.
 *
 *                  The two rooms are different objects with different liveness,
 *                  so sharing a key must not merge them.
 */
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

/*!
 * @brief           Checks that a wall's owner reference is keyed by the map the
 *                  owning room was listed under, even when that room declares a
 *                  different map.
 *
 *                  The capture builds the owner key from the map it is walking
 *                  through, not from the room's declared map, so the key
 *                  matches the room's own RoomRecord::key.
 */
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

/*!
 * @brief           Checks that a wall claimed by several rooms, including a
 *                  retired one, keeps every owner, and that a room in both room
 *                  sets appears once as owner.
 */
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

/*!
 * @brief           Checks that rooms registered in ascending or descending id
 *                  order end up in the same sorted order in the snapshot, and
 *                  that capturing leaves the source room unchanged.
 */
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

/*!
 * @brief           Checks that walls, passages and floors registered in
 *                  ascending or descending id order, and the maps list, come
 *                  out sorted the same way in the snapshot.
 *
 *                  Two maps never share an id, because map ids come from a
 *                  counter that only increases (Map::nextId), so no map-id
 *                  collision is tested.
 */
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

/*!
 * @brief           Checks that a room's wall and passage references, a wall's
 *                  owner rooms, a floor's rooms and a passage's walls come out
 *                  sorted the same way whatever order they were added in.
 *
 *                  Each collection gets three distinct members, added in
 *                  opposite orders in the two captures.
 */
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

/*!
 * @brief           Checks that capture works while the caller already holds the
 *                  semantic update lock and does not try to take it again,
 *                  which would deadlock.
 *
 *                  Atlas::semanticUpdateMutex is a plain, non-recursive
 *                  std::mutex, so taking it again would hang this test:
 *                  reaching the last assertion is the proof.
 */
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

/*!
 * @brief           Checks that the cheap plane metadata accessor returns the
 *                  same scalars as the full geometry snapshot, and that the
 *                  wall record carries those same values.
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

    EXPECT_EQ(metadata.planeEquation_world, fullSnapshot.planeEquation_world);
    EXPECT_EQ(metadata.planeCentroid_world_m,
              fullSnapshot.planeCentroid_world_m);
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
    EXPECT_EQ(p_wallRecord->planeEquation_world, metadata.planeEquation_world);
    EXPECT_EQ(p_wallRecord->planeCentroid_world_m,
              metadata.planeCentroid_world_m);
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

/*!
 * @brief           Checks that capturing from a null atlas returns an empty
 *                  snapshot with no current map instead of crashing.
 */
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

/*!
 * @brief           Checks that the wall, room and passage reference append
 *                  helpers ignore a null pointer and add nothing.
 *
 *                  Room::setWalls(), Passage::addAssociateWall(),
 *                  Floor::addRoom() and Floor::setRooms() already refuse null
 *                  pointers, so capture never reaches this case; the helpers
 *                  are called directly instead.
 */
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

/*!
 * @brief           Checks that the double ordering function puts negative zero
 *                  before positive zero and negative NaN, the finite values and
 *                  positive NaN in one fixed order, with no value less than
 *                  itself.
 *
 *                  doubleTotalOrderKey() and isDoubleLess() are called
 *                  directly, because every ordering guarantee of the snapshot
 *                  is built on them.
 */
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
