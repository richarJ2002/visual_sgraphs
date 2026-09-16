/**
 * Focused tests: persistence of the last-confirmed room context
 * (semantic::RoomContextSnapshot, WallBounds, semantic::PassageContext) across the real
 * Atlas::CreateNewMap() tracking-loss/new-map lifecycle boundary.
 *
 * These tests exercise the genuine production exporter
 * (Atlas::exportRoomContextFromCurrentMap(), invoked internally from
 * Atlas::createNewMapWhileAtlasLocked()) and the genuine semantic::Room/geometric::Plane/semantic::Passage
 * getters it reads from -- they do not reconstruct the expected snapshot by
 * hand.
 */

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "Map.h"
#include "Semantic/Floor.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "Semantic/RoomContextSnapshot.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{

/** Builds a wall geometric::Plane with genuine, production-computed U/V bounds by
 * feeding a synthetic point cloud through the real geometric::Plane::updateSizeOfPlane()
 * path (the same function geometric::Plane::getGeometrySnapshot() reads from). */
void makeRefitWallPlane(geometric::Plane &wall_inout, int id_in, Map *p_map_in)
{
    wall_inout.setId(id_in);
    wall_inout.SetMap(p_map_in);
    wall_inout.setPlaneType(geometric::Plane::PlaneVariant::WALL);
    wall_inout.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
    wall_inout.setCentroid(Eigen::Vector3d(0.0, 1.0, 1.0));

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int yIndex = 0; yIndex <= 2; ++yIndex)
    {
        for (int zIndex = 0; zIndex <= 2; ++zIndex)
        {
            pcl::PointXYZRGBA point;
            point.x = 0.0F;
            point.y = static_cast<float>(yIndex);
            point.z = static_cast<float>(zIndex);
            cloud->push_back(point);
        }
    }
    wall_inout.setMapClouds(cloud);
    wall_inout.updateSizeOfPlane();
}

} // namespace

TEST(RoomContextPersist, NullCurrentMapExportIsNoOp)
{
    Atlas atlas(0);
    atlas.clearAtlas();

    atlas.exportRoomContextFromCurrentMap();

    const auto history = atlas.copyRoomContextHistory();
    EXPECT_TRUE(history.empty());
}

TEST(RoomContextPersist, EmptyRoomCollectionExportIsNoOp)
{
    Atlas                   atlas(0);
    Map                    *p_map = atlas.GetCurrentMap();
    const long unsigned int mapId = p_map->GetId();

    atlas.exportRoomContextFromCurrentMap();

    const auto history = atlas.copyRoomContextHistory();
    EXPECT_EQ(history.count(mapId), 0U);
}

TEST(RoomContextPersist, ExistingFieldsRetainNamesTypesAndValues)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    geometric::Plane wall;
    makeRefitWallPlane(wall, 1, p_map);
    p_map->AddMapPlane(&wall);

    semantic::Room room;
    room.setId(7);
    room.setMap(p_map);
    room.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0));
    room.setWalls(&wall);
    room.setRoomVariant(semantic::Room::RoomVariant::ROOM);
    room.setBoundaryStatus(semantic::Room::BoundaryStatus::INCOMPLETE);
    p_map->AddDetectedMapRoom(&room);

    const long unsigned int mapId = p_map->GetId();
    atlas.exportRoomContextFromCurrentMap();

    const auto history = atlas.copyRoomContextHistory();
    ASSERT_EQ(history.count(mapId), 1U);
    ASSERT_EQ(history.at(mapId).size(), 1U);

    const semantic::RoomContextSnapshot &snap = history.at(mapId).front();
    EXPECT_EQ(snap.roomId, 7);
    /* semantic::Room never given a semantic::Floor here: the exporter must not invent one. */
    EXPECT_EQ(snap.floorId, -1);
    EXPECT_TRUE(snap.centroid.isApprox(Eigen::Vector3d(1.0, 1.0, 1.0)));
    EXPECT_EQ(snap.roomTag, "room_7");
    EXPECT_TRUE(snap.wasConfirmedRoom);
    EXPECT_EQ(snap.boundaryStatus,
              static_cast<int>(semantic::Room::BoundaryStatus::INCOMPLETE));
    EXPECT_GT(snap.timestamp, 0.0);
    ASSERT_EQ(snap.wallNormals.size(), 1U);
    ASSERT_EQ(snap.wallCentroids.size(), 1U);
    ASSERT_EQ(snap.wallDistances.size(), 1U);
    EXPECT_TRUE(snap.wallNormals.front().allFinite());
    EXPECT_TRUE(snap.wallCentroids.front().isApprox(wall.getCentroid()));
    EXPECT_DOUBLE_EQ(snap.wallDistances.front(),
                     wall.getGlobalEquation().distance());

    /* The room-tag side effect on the live semantic::Room mirrors the exported field
     * (verifies the exporter uses the same production semantic::Room::setRoomTag()
     * path it always has). */
    EXPECT_EQ(room.getRoomTag(), "room_7");
}

TEST(RoomContextPersist, FloorIdCapturedWhenRoomHasFloorIdentity)
{
    /* A room is always floor-scoped (semantic::Room::getFloor()); the "last confirmed
     * room" context this snapshot exists to carry across a tracking-loss
     * boundary must carry that floor along too rather than dropping it. */
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    geometric::Plane wall;
    makeRefitWallPlane(wall, 1, p_map);
    p_map->AddMapPlane(&wall);

    semantic::Floor floor;
    floor.setId(42);
    floor.setMap(p_map);
    p_map->AddMapFloor(&floor);

    semantic::Room room;
    room.setId(8);
    room.setMap(p_map);
    room.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0));
    room.setWalls(&wall);
    room.setFloor(&floor);
    p_map->AddDetectedMapRoom(&room);

    const long unsigned int mapId = p_map->GetId();
    atlas.exportRoomContextFromCurrentMap();

    const auto history = atlas.copyRoomContextHistory();
    ASSERT_EQ(history.count(mapId), 1U);
    const semantic::RoomContextSnapshot &snap = history.at(mapId).front();
    EXPECT_EQ(snap.roomId, 8);
    EXPECT_EQ(snap.floorId, 42);
}

TEST(RoomContextPersist, WallBoundsIndexAlignedWithMixedValidity)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    geometric::Plane refitWall;
    makeRefitWallPlane(refitWall, 1, p_map);
    p_map->AddMapPlane(&refitWall);

    geometric::Plane unrefitWall; // Never assigned a cloud: bounds stay at the
                       // sentinel min>max default, so valid() is false.
    unrefitWall.setId(2);
    unrefitWall.SetMap(p_map);
    unrefitWall.setPlaneType(geometric::Plane::PlaneVariant::WALL);
    unrefitWall.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(0.0, 1.0, 0.0, 0.0)));
    p_map->AddMapPlane(&unrefitWall);

    geometric::Plane badWall;
    makeRefitWallPlane(badWall, 3, p_map);
    badWall.setBad();
    p_map->AddMapPlane(&badWall);

    semantic::Room room;
    room.setId(9);
    room.setMap(p_map);
    room.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0));
    room.setWalls(&refitWall);
    room.setWalls(&unrefitWall);
    room.setWalls(&badWall);
    p_map->AddDetectedMapRoom(&room);

    const long unsigned int mapId = p_map->GetId();
    atlas.exportRoomContextFromCurrentMap();

    const auto history = atlas.copyRoomContextHistory();
    ASSERT_EQ(history.count(mapId), 1U);
    const semantic::RoomContextSnapshot &snap = history.at(mapId).front();

    /* Exactly one WallBounds record per wallNormals entry, same order. */
    ASSERT_EQ(snap.wallBounds.size(), snap.wallNormals.size());
    ASSERT_EQ(snap.wallBounds.size(), 3U);

    EXPECT_TRUE(snap.wallBounds[0].valid);
    EXPECT_LT(snap.wallBounds[0].minU_m, snap.wallBounds[0].maxU_m);
    EXPECT_LT(snap.wallBounds[0].minV_m, snap.wallBounds[0].maxV_m);

    EXPECT_FALSE(snap.wallBounds[1].valid);

    EXPECT_FALSE(snap.wallBounds[2].valid);
    EXPECT_FALSE(std::isfinite(snap.wallNormals[2].x()));
}

TEST(RoomContextPersist, PassageContextIndexAlignedWithGenuineGetters)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    semantic::Room knownRoom;
    knownRoom.setId(20);
    knownRoom.setMap(p_map);
    p_map->AddDetectedMapRoom(&knownRoom);

    semantic::Room farRoom;
    farRoom.setId(21);
    farRoom.setMap(p_map);
    p_map->AddDetectedMapRoom(&farRoom);

    geometric::Plane associatedWall;
    makeRefitWallPlane(associatedWall, 30, p_map);
    p_map->AddMapPlane(&associatedWall);

    /* semantic::Passage A: fully specified -- valid aperture, known far-side room,
     * known-side direction, mixed traversal evidence, one associated wall. */
    semantic::Passage fullPassage;
    fullPassage.setId(40);
    fullPassage.setMap(p_map);
    fullPassage.setPassable(true);
    fullPassage.setWidth(1.2);
    fullPassage.setHeight(2.1);
    fullPassage.setCentroid(Eigen::Vector3d(0.0, 1.0, 1.0));
    fullPassage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
    fullPassage.setKnownSideRoom(&knownRoom);
    fullPassage.setKnownSideDirection(Eigen::Vector3d(-1.0, 0.0, 0.0));
    fullPassage.setProspectiveRoom(&farRoom);
    fullPassage.addAssociateWall(&associatedWall);
    fullPassage.addTraversalObservation(
        semantic::Passage::TraversalDirection::KNOWN_TO_FAR);
    fullPassage.addTraversalObservation(
        semantic::Passage::TraversalDirection::KNOWN_TO_FAR);
    fullPassage.addTraversalObservation(
        semantic::Passage::TraversalDirection::FAR_TO_KNOWN);
    fullPassage.addTraversalObservation(semantic::Passage::TraversalDirection::UNKNOWN);
    p_map->AddMapPassage(&fullPassage);
    knownRoom.setDoorways(&fullPassage);

    /* semantic::Passage B: no far-side room resolved yet, invalid (negative) aperture,
     * no known-side direction, no traversal evidence. */
    semantic::Passage sparsePassage;
    sparsePassage.setId(41);
    sparsePassage.setMap(p_map);
    sparsePassage.setPassable(false);
    sparsePassage.setWidth(-1.0);
    sparsePassage.setHeight(0.5);
    sparsePassage.setCentroid(Eigen::Vector3d(2.0, 1.0, 1.0));
    sparsePassage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, -2.0)));
    p_map->AddMapPassage(&sparsePassage);
    knownRoom.setDoorways(&sparsePassage);

    /* semantic::Passage C: non-finite (NaN) height and default (missing) width. */
    semantic::Passage nonFinitePassage;
    nonFinitePassage.setId(42);
    nonFinitePassage.setMap(p_map);
    nonFinitePassage.setHeight(std::numeric_limits<double>::quiet_NaN());
    nonFinitePassage.setCentroid(Eigen::Vector3d(3.0, 1.0, 1.0));
    p_map->AddMapPassage(&nonFinitePassage);
    knownRoom.setDoorways(&nonFinitePassage);

    const long unsigned int mapId = p_map->GetId();
    atlas.exportRoomContextFromCurrentMap();

    const auto history = atlas.copyRoomContextHistory();
    ASSERT_EQ(history.count(mapId), 1U);

    const std::vector<semantic::RoomContextSnapshot> &snapshots   = history.at(mapId);
    const semantic::RoomContextSnapshot              *p_knownSnap = nullptr;
    for (const semantic::RoomContextSnapshot &candidate : snapshots)
    {
        if (candidate.roomId == 20)
        {
            p_knownSnap = &candidate;
        }
    }
    ASSERT_NE(p_knownSnap, nullptr);

    /* Index alignment: one semantic::PassageContext per passageCentroids element. */
    ASSERT_EQ(p_knownSnap->passageContexts.size(),
              p_knownSnap->passageCentroids.size());
    ASSERT_EQ(p_knownSnap->passageContexts.size(), 3U);

    const semantic::PassageContext &full = p_knownSnap->passageContexts[0];
    EXPECT_EQ(full.id, 40);
    EXPECT_TRUE(full.passable);
    EXPECT_TRUE(full.hasFarSideRoom);
    EXPECT_EQ(full.secondaryRoomId, 21);
    EXPECT_TRUE(full.apertureValid);
    EXPECT_DOUBLE_EQ(full.width_m, 1.2);
    EXPECT_DOUBLE_EQ(full.height_m, 2.1);
    EXPECT_TRUE(full.hasKnownSideDirection);
    EXPECT_TRUE(full.knownSideDirection_World.isApprox(
        Eigen::Vector3d(-1.0, 0.0, 0.0)));
    EXPECT_TRUE(full.hasKnownSideRoom);
    EXPECT_EQ(full.knownSideRoomId, knownRoom.getId());
    EXPECT_EQ(full.traversalKnownToFarCount, 2U);
    EXPECT_EQ(full.traversalFarToKnownCount, 1U);
    EXPECT_EQ(full.traversalUnknownCount, 1U);
    EXPECT_EQ(full.associatedWallCount, 1U);
    EXPECT_TRUE(full.hasBidirectionalTraversalEvidence);

    const semantic::PassageContext &sparse = p_knownSnap->passageContexts[1];
    EXPECT_EQ(sparse.id, 41);
    EXPECT_FALSE(sparse.passable);
    EXPECT_FALSE(sparse.hasFarSideRoom); // absence, not a sentinel ID
    EXPECT_FALSE(sparse.apertureValid);  // negative width
    EXPECT_FALSE(sparse.hasKnownSideDirection);
    EXPECT_FALSE(sparse.hasKnownSideRoom);
    EXPECT_EQ(sparse.knownSideRoomId, -1);
    EXPECT_EQ(sparse.traversalKnownToFarCount, 0U);
    EXPECT_EQ(sparse.traversalFarToKnownCount, 0U);
    EXPECT_EQ(sparse.traversalUnknownCount, 0U);
    EXPECT_FALSE(sparse.hasBidirectionalTraversalEvidence);

    const semantic::PassageContext &nonFinite = p_knownSnap->passageContexts[2];
    EXPECT_EQ(nonFinite.id, 42);
    EXPECT_FALSE(nonFinite.apertureValid); // NaN height, default width
    EXPECT_FALSE(nonFinite.hasFarSideRoom);
}

TEST(RoomContextPersist, MissingAttributesCompleteWithoutCrash)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    geometric::Plane badWall;
    badWall.setId(1);
    badWall.SetMap(p_map);
    badWall.setBad();
    p_map->AddMapPlane(&badWall);

    semantic::Passage defaultPassage;
    defaultPassage.setId(50);
    defaultPassage.setMap(p_map);
    p_map->AddMapPassage(&defaultPassage);

    semantic::Room room;
    room.setId(30);
    room.setMap(p_map);
    room.setWalls(&badWall);
    room.setDoorways(&defaultPassage);
    p_map->AddDetectedMapRoom(&room);

    const long unsigned int mapId = p_map->GetId();
    EXPECT_NO_THROW(atlas.exportRoomContextFromCurrentMap());

    const auto history = atlas.copyRoomContextHistory();
    ASSERT_EQ(history.count(mapId), 1U);
    const semantic::RoomContextSnapshot &snap = history.at(mapId).front();
    ASSERT_EQ(snap.wallBounds.size(), 1U);
    EXPECT_FALSE(snap.wallBounds.front().valid);
    ASSERT_EQ(snap.passageContexts.size(), 1U);
    EXPECT_FALSE(snap.passageContexts.front().apertureValid);
    EXPECT_FALSE(snap.passageContexts.front().hasFarSideRoom);
}

TEST(RoomContextPersist, ClearMapBumpsRevisionGeneration)
{
    /* Same-map clears keep the map id, so the visualization/voxblox
     * revision token would not observe the reset and stale markers and
     * clouds would persist alongside the fresh map. The clear must bump
     * the generation the token is built from. */
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    const int changeIndexBefore = p_map->GetLastBigChangeIdx();
    atlas.clearMap();

    EXPECT_EQ(p_map->GetLastBigChangeIdx(), changeIndexBefore + 1);
}

TEST(RoomContextPersist, VisitedFlagRoundTripsThroughExport)
{
    Atlas atlas(0);
    Map  *p_oldMap = atlas.GetCurrentMap();

    semantic::Room visitedRoom;
    visitedRoom.setId(5);
    visitedRoom.setMap(p_oldMap);
    visitedRoom.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0));
    visitedRoom.setPreviouslyVisited(true);
    p_oldMap->AddDetectedMapRoom(&visitedRoom);

    semantic::Room freshRoom;
    freshRoom.setId(6);
    freshRoom.setMap(p_oldMap);
    freshRoom.setCentroid(Eigen::Vector3d(2.0, 2.0, 2.0));
    p_oldMap->AddDetectedMapRoom(&freshRoom);

    const long unsigned int oldMapId = p_oldMap->GetId();
    atlas.CreateNewMap();

    const auto history = atlas.copyRoomContextHistory();
    ASSERT_EQ(history.count(oldMapId), 1U);
    ASSERT_EQ(history.at(oldMapId).size(), 2U);

    bool sawVisited   = false;
    bool sawUnvisited = false;
    for (const auto &snapshot : history.at(oldMapId))
    {
        if (snapshot.roomId == 5)
        {
            EXPECT_TRUE(snapshot.wasPreviouslyVisited);
            sawVisited = true;
        }
        if (snapshot.roomId == 6)
        {
            EXPECT_FALSE(snapshot.wasPreviouslyVisited);
            sawUnvisited = true;
        }
    }
    EXPECT_TRUE(sawVisited);
    EXPECT_TRUE(sawUnvisited);
}

TEST(RoomContextPersist, SurvivesRealNewMapLifecycleBoundary)
{
    Atlas atlas(0);
    Map  *p_oldMap = atlas.GetCurrentMap();

    geometric::Plane wall;
    makeRefitWallPlane(wall, 1, p_oldMap);
    p_oldMap->AddMapPlane(&wall);

    semantic::Room room;
    room.setId(5);
    room.setMap(p_oldMap);
    room.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0));
    room.setWalls(&wall);
    p_oldMap->AddDetectedMapRoom(&room);

    const long unsigned int oldMapId = p_oldMap->GetId();

    /* Genuine tracking-loss/new-map boundary: Tracking.cc calls
     * Atlas::CreateNewMap() directly; it internally exports the outgoing
     * map's room context before installing the new current map. */
    atlas.CreateNewMap();
    Map *p_newMap = atlas.GetCurrentMap();
    ASSERT_NE(p_newMap, p_oldMap);

    const auto history = atlas.copyRoomContextHistory();
    ASSERT_EQ(history.count(oldMapId), 1U);
    ASSERT_EQ(history.at(oldMapId).size(), 1U);
    EXPECT_EQ(history.at(oldMapId).front().roomId, 5);
    EXPECT_EQ(history.at(oldMapId).front().roomTag, "room_5");
    ASSERT_EQ(history.at(oldMapId).front().wallBounds.size(), 1U);
    EXPECT_TRUE(history.at(oldMapId).front().wallBounds.front().valid);

    /* Map identity is carried by the history container key alone; no
     * separate snapshot-level mapId is introduced. */
    EXPECT_EQ(history.count(p_newMap->GetId()), 0U);
}

TEST(RoomContextPersist, HundredRepeatedExportResetChecksPass)
{
    unsigned int successCount = 0U;
    for (unsigned int iteration = 0U; iteration < 100U; ++iteration)
    {
        Atlas atlas(0);
        Map  *p_oldMap = atlas.GetCurrentMap();

        geometric::Plane wall;
        makeRefitWallPlane(wall, 1, p_oldMap);
        p_oldMap->AddMapPlane(&wall);

        semantic::Room room;
        room.setId(static_cast<int>(iteration));
        room.setMap(p_oldMap);
        room.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0));
        room.setWalls(&wall);
        p_oldMap->AddDetectedMapRoom(&room);

        const long unsigned int oldMapId = p_oldMap->GetId();
        atlas.CreateNewMap();

        const auto history = atlas.copyRoomContextHistory();
        const bool ok      = history.count(oldMapId) == 1U &&
                        history.at(oldMapId).size() == 1U &&
                        history.at(oldMapId).front().roomId ==
                            static_cast<int>(iteration) &&
                        history.at(oldMapId).front().wallBounds.size() == 1U &&
                        history.at(oldMapId).front().wallBounds.front().valid;
        if (ok)
        {
            ++successCount;
        }
    }
    EXPECT_EQ(successCount, 100U);
}

TEST(RoomContextPersist,
     TwoWorkerExportCopyCharacterizationHasNoObservableCorruption)
{
    Atlas atlas(0);

    /* Keep every historical room/wall alive for the whole test; Atlas retains
     * stored (retired-current) maps but the test owns the entities. */
    std::vector<std::unique_ptr<geometric::Plane>> walls;
    std::vector<std::unique_ptr<semantic::Room>>  rooms;

    std::atomic<bool> readerComplete{false};
    std::thread       reader(
        [&atlas, &readerComplete]()
        {
            for (unsigned int iteration = 0U; iteration < 100U; ++iteration)
            {
                const auto history = atlas.copyRoomContextHistory();
                for (const auto &entry : history)
                {
                    for (const semantic::RoomContextSnapshot &snap : entry.second)
                    {
                        /* Corruption characterization: index-alignment
                         * invariants must hold on every observed copy. */
                        if (snap.wallBounds.size() != snap.wallNormals.size() ||
                            snap.wallBounds.size() !=
                                snap.wallCentroids.size() ||
                            snap.wallBounds.size() !=
                                snap.wallDistances.size() ||
                            snap.passageContexts.size() !=
                                snap.passageCentroids.size())
                        {
                            ADD_FAILURE()
                                << "Observed misaligned snapshot vectors";
                        }
                    }
                }
            }
            readerComplete.store(true, std::memory_order_release);
        });

    for (unsigned int iteration = 0U; iteration < 100U; ++iteration)
    {
        Map *p_map = atlas.GetCurrentMap();

        auto p_wall = std::make_unique<geometric::Plane>();
        makeRefitWallPlane(*p_wall, static_cast<int>(iteration) + 1, p_map);
        p_map->AddMapPlane(p_wall.get());

        auto p_room = std::make_unique<semantic::Room>();
        p_room->setId(static_cast<int>(iteration));
        p_room->setMap(p_map);
        p_room->setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0));
        p_room->setWalls(p_wall.get());
        p_map->AddDetectedMapRoom(p_room.get());

        walls.push_back(std::move(p_wall));
        rooms.push_back(std::move(p_room));

        atlas.CreateNewMap();
    }

    reader.join();
    EXPECT_TRUE(readerComplete.load(std::memory_order_acquire));
}

} // namespace core
} // namespace vs_graphs
