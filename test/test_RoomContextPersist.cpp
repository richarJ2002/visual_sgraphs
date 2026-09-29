/*!
 * Focused tests: persistence of the last-confirmed room context
 * (semantic::RoomContextSnapshot, WallBounds, semantic::PassageContext) across
 * the real Atlas::CreateNewMap() tracking-loss/new-map lifecycle boundary.
 *
 * These tests exercise the genuine production exporter
 * (Atlas::exportRoomContextFromCurrentMap(), invoked internally from
 * Atlas::createNewMapWhileAtlasLocked()) and the genuine
 * semantic::Room/geometric::Plane/semantic::Passage getters it reads from --
 * they do not reconstruct the expected snapshot by hand.
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
#include <rclcpp/logging.hpp>
#include <thread>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{

/*! Builds a wall geometric::Plane with genuine, production-computed U/V bounds
 * by feeding a synthetic point cloud through the real
 * geometric::Plane::updateSizeOfPlane() path (the same function
 * geometric::Plane::getGeometrySnapshot() reads from). */
void makeRefitWallPlane(geometric::Plane &wall_inout, int id_in, Map *p_map_in)
{
    ASSERT_EQ((wall_inout.setId(id_in)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wall_inout.setMap(p_map_in)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wall_inout.setPlaneType(geometric::Plane::PlaneVariant::WALL)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wall_inout.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)))),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wall_inout.setCentroid(Eigen::Vector3d(0.0, 1.0, 1.0))),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);

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
    ASSERT_EQ((wall_inout.setMapClouds(cloud)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wall_inout.updateSizeOfPlane()),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
}

} // namespace

TEST(RoomContextPersist, NullCurrentMapExportIsNoOp)
{
    Atlas atlas(0);
    ASSERT_EQ((atlas.clearAtlas()), AtlasStatus::ATLAS_STATUS_SUCCESS);

    ASSERT_EQ((atlas.exportRoomContextFromCurrentMap()),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
        history{};
    ASSERT_EQ((atlas.copyRoomContextHistory(history)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_TRUE(history.empty());
}

TEST(RoomContextPersist, EmptyRoomCollectionExportIsNoOp)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    unsigned long mapIdValue{};
    ASSERT_EQ((p_map->getId(mapIdValue)), MapStatus::MAP_STATUS_SUCCESS);
    const long unsigned int mapId = static_cast<long unsigned int>(mapIdValue);

    ASSERT_EQ((atlas.exportRoomContextFromCurrentMap()),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
        history{};
    ASSERT_EQ((atlas.copyRoomContextHistory(history)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    EXPECT_EQ(history.count(mapId), 0U);
}

TEST(RoomContextPersist, ExistingFieldsRetainNamesTypesAndValues)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane wall;
    makeRefitWallPlane(wall, 1, p_map);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    semantic::Room room;
    ASSERT_EQ((room.setId(7)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(&wall)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryStatus(semantic::Room::BoundaryStatus::INCOMPLETE)),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    unsigned long mapIdValue{};
    ASSERT_EQ((p_map->getId(mapIdValue)), MapStatus::MAP_STATUS_SUCCESS);
    const long unsigned int mapId = static_cast<long unsigned int>(mapIdValue);
    ASSERT_EQ((atlas.exportRoomContextFromCurrentMap()),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
        history{};
    ASSERT_EQ((atlas.copyRoomContextHistory(history)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(history.count(mapId), 1U);
    ASSERT_EQ(history.at(mapId).size(), 1U);

    const semantic::RoomContextSnapshot &snap = history.at(mapId).front();
    EXPECT_EQ(snap.roomId, 7);
    /* semantic::Room never given a semantic::Floor here: the exporter must not
     * invent one. */
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
    Eigen::Vector3d getCentroid2{};
    ASSERT_EQ((wall.getCentroid(getCentroid2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_TRUE(snap.wallCentroids.front().isApprox(getCentroid2));
    g2o::Plane3D getGlobalEquation2{};
    ASSERT_EQ((wall.getGlobalEquation(getGlobalEquation2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(snap.wallDistances.front(), getGlobalEquation2.distance());

    /* The room-tag side effect on the live semantic::Room mirrors the exported
     * field (verifies the exporter uses the same production
     * semantic::Room::setRoomTag() path it always has). */
    std::string roomTag2{};
    ASSERT_EQ((room.getRoomTag(roomTag2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(roomTag2, "room_7");
}

TEST(RoomContextPersist, FloorIdCapturedWhenRoomHasFloorIdentity)
{
    /* A room is always floor-scoped (semantic::Room::getFloor()); the "last
     * confirmed room" context this snapshot exists to carry across a
     * tracking-loss boundary must carry that floor along too rather than
     * dropping it. */
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane wall;
    makeRefitWallPlane(wall, 1, p_map);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    semantic::Floor floor;
    ASSERT_EQ((floor.setId(42)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ((floor.setMap(p_map)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    semantic::Room room;
    ASSERT_EQ((room.setId(8)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(&wall)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setFloor(&floor)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    unsigned long mapIdValue{};
    ASSERT_EQ((p_map->getId(mapIdValue)), MapStatus::MAP_STATUS_SUCCESS);
    const long unsigned int mapId = static_cast<long unsigned int>(mapIdValue);
    ASSERT_EQ((atlas.exportRoomContextFromCurrentMap()),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
        history{};
    ASSERT_EQ((atlas.copyRoomContextHistory(history)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(history.count(mapId), 1U);
    const semantic::RoomContextSnapshot &snap = history.at(mapId).front();
    EXPECT_EQ(snap.roomId, 8);
    EXPECT_EQ(snap.floorId, 42);
}

TEST(RoomContextPersist, WallBoundsIndexAlignedWithMixedValidity)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane refitWall;
    makeRefitWallPlane(refitWall, 1, p_map);
    ASSERT_EQ((p_map->addMapPlane(&refitWall)), MapStatus::MAP_STATUS_SUCCESS);

    geometric::Plane
        unrefitWall; // Never assigned a cloud: bounds stay at the
                     // sentinel min>max default, so valid() is false.
    ASSERT_EQ((unrefitWall.setId(2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((unrefitWall.setMap(p_map)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((unrefitWall.setPlaneType(geometric::Plane::PlaneVariant::WALL)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((unrefitWall.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(0.0, 1.0, 0.0, 0.0)))),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&unrefitWall)),
              MapStatus::MAP_STATUS_SUCCESS);

    geometric::Plane badWall;
    makeRefitWallPlane(badWall, 3, p_map);
    ASSERT_EQ((badWall.setBad()), geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&badWall)), MapStatus::MAP_STATUS_SUCCESS);

    semantic::Room room;
    ASSERT_EQ((room.setId(9)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(&refitWall)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(&unrefitWall)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(&badWall)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    unsigned long mapIdValue{};
    ASSERT_EQ((p_map->getId(mapIdValue)), MapStatus::MAP_STATUS_SUCCESS);
    const long unsigned int mapId = static_cast<long unsigned int>(mapIdValue);
    ASSERT_EQ((atlas.exportRoomContextFromCurrentMap()),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
        history{};
    ASSERT_EQ((atlas.copyRoomContextHistory(history)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(history.count(mapId), 1U);
    const semantic::RoomContextSnapshot &snap = history.at(mapId).front();

    /* Exactly one WallBounds record per wallNormals entry, same order. */
    ASSERT_EQ(snap.wallBounds.size(), snap.wallNormals.size());
    ASSERT_EQ(snap.wallBounds.size(), 3U);

    EXPECT_TRUE(snap.wallBounds[0].isValid);
    EXPECT_LT(snap.wallBounds[0].minU_m, snap.wallBounds[0].maxU_m);
    EXPECT_LT(snap.wallBounds[0].minV_m, snap.wallBounds[0].maxV_m);

    EXPECT_FALSE(snap.wallBounds[1].isValid);

    EXPECT_FALSE(snap.wallBounds[2].isValid);
    EXPECT_FALSE(std::isfinite(snap.wallNormals[2].x()));
}

TEST(RoomContextPersist, PassageContextIndexAlignedWithGenuineGetters)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    semantic::Room knownRoom;
    ASSERT_EQ((knownRoom.setId(20)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&knownRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    semantic::Room farRoom;
    ASSERT_EQ((farRoom.setId(21)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((farRoom.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&farRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    geometric::Plane associatedWall;
    makeRefitWallPlane(associatedWall, 30, p_map);
    ASSERT_EQ((p_map->addMapPlane(&associatedWall)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* semantic::Passage A: fully specified -- valid aperture, known far-side
     * room, known-side direction, mixed traversal evidence, one associated
     * wall. */
    semantic::Passage fullPassage;
    ASSERT_EQ((fullPassage.setId(40)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.setWidth(1.2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.setHeight(2.1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.setCentroid(Eigen::Vector3d(0.0, 1.0, 1.0))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.setKnownSideRoom(&knownRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ(
        (fullPassage.setKnownSideDirection(Eigen::Vector3d(-1.0, 0.0, 0.0))),
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.setProspectiveRoom(&farRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.addAssociateWall(&associatedWall)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.addTraversalObservation(
                  semantic::Passage::TraversalDirection::KNOWN_TO_FAR)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.addTraversalObservation(
                  semantic::Passage::TraversalDirection::KNOWN_TO_FAR)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.addTraversalObservation(
                  semantic::Passage::TraversalDirection::FAR_TO_KNOWN)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((fullPassage.addTraversalObservation(
                  semantic::Passage::TraversalDirection::UNKNOWN)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPassage(&fullPassage)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setDoorways(&fullPassage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    /* semantic::Passage B: no far-side room resolved yet, invalid (negative)
     * aperture, no known-side direction, no traversal evidence. */
    semantic::Passage sparsePassage;
    ASSERT_EQ((sparsePassage.setId(41)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((sparsePassage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((sparsePassage.setPassable(false)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((sparsePassage.setWidth(-1.0)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((sparsePassage.setHeight(0.5)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((sparsePassage.setCentroid(Eigen::Vector3d(2.0, 1.0, 1.0))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((sparsePassage.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, -2.0)))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPassage(&sparsePassage)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setDoorways(&sparsePassage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    /* semantic::Passage C: non-finite (NaN) height and default (missing) width.
     */
    semantic::Passage nonFinitePassage;
    ASSERT_EQ((nonFinitePassage.setId(42)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((nonFinitePassage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ(
        (nonFinitePassage.setHeight(std::numeric_limits<double>::quiet_NaN())),
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((nonFinitePassage.setCentroid(Eigen::Vector3d(3.0, 1.0, 1.0))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPassage(&nonFinitePassage)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((knownRoom.setDoorways(&nonFinitePassage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    unsigned long mapIdValue{};
    ASSERT_EQ((p_map->getId(mapIdValue)), MapStatus::MAP_STATUS_SUCCESS);
    const long unsigned int mapId = static_cast<long unsigned int>(mapIdValue);
    ASSERT_EQ((atlas.exportRoomContextFromCurrentMap()),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
        history{};
    ASSERT_EQ((atlas.copyRoomContextHistory(history)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(history.count(mapId), 1U);

    const std::vector<semantic::RoomContextSnapshot> &snapshots =
        history.at(mapId);
    const semantic::RoomContextSnapshot *p_knownSnap = nullptr;
    for (const semantic::RoomContextSnapshot &candidate : snapshots)
    {
        if (candidate.roomId == 20)
        {
            p_knownSnap = &candidate;
        }
    }
    ASSERT_NE(p_knownSnap, nullptr);

    /* Index alignment: one semantic::PassageContext per passageCentroids
     * element. */
    ASSERT_EQ(p_knownSnap->passageContexts.size(),
              p_knownSnap->passageCentroids.size());
    ASSERT_EQ(p_knownSnap->passageContexts.size(), 3U);

    const semantic::PassageContext &full = p_knownSnap->passageContexts[0];
    EXPECT_EQ(full.id, 40);
    EXPECT_TRUE(full.isPassable);
    EXPECT_TRUE(full.hasFarSideRoom);
    EXPECT_EQ(full.secondaryRoomId, 21);
    EXPECT_TRUE(full.isApertureValid);
    EXPECT_DOUBLE_EQ(full.width_m, 1.2);
    EXPECT_DOUBLE_EQ(full.height_m, 2.1);
    EXPECT_TRUE(full.hasKnownSideDirection);
    EXPECT_TRUE(full.knownSideDirection_World.isApprox(
        Eigen::Vector3d(-1.0, 0.0, 0.0)));
    EXPECT_TRUE(full.hasKnownSideRoom);
    int id2{};
    ASSERT_EQ((knownRoom.getId(id2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(full.knownSideRoomId, id2);
    EXPECT_EQ(full.traversalKnownToFarCount, 2U);
    EXPECT_EQ(full.traversalFarToKnownCount, 1U);
    EXPECT_EQ(full.traversalUnknownCount, 1U);
    EXPECT_EQ(full.associatedWallCount, 1U);
    EXPECT_TRUE(full.hasBidirectionalTraversalEvidence);

    const semantic::PassageContext &sparse = p_knownSnap->passageContexts[1];
    EXPECT_EQ(sparse.id, 41);
    EXPECT_FALSE(sparse.isPassable);
    EXPECT_FALSE(sparse.hasFarSideRoom);  // absence, not a sentinel ID
    EXPECT_FALSE(sparse.isApertureValid); // negative width
    EXPECT_FALSE(sparse.hasKnownSideDirection);
    EXPECT_FALSE(sparse.hasKnownSideRoom);
    EXPECT_EQ(sparse.knownSideRoomId, -1);
    EXPECT_EQ(sparse.traversalKnownToFarCount, 0U);
    EXPECT_EQ(sparse.traversalFarToKnownCount, 0U);
    EXPECT_EQ(sparse.traversalUnknownCount, 0U);
    EXPECT_FALSE(sparse.hasBidirectionalTraversalEvidence);

    const semantic::PassageContext &nonFinite = p_knownSnap->passageContexts[2];
    EXPECT_EQ(nonFinite.id, 42);
    EXPECT_FALSE(nonFinite.isApertureValid); // NaN height, default width
    EXPECT_FALSE(nonFinite.hasFarSideRoom);
}

TEST(RoomContextPersist, MissingAttributesCompleteWithoutCrash)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane badWall;
    ASSERT_EQ((badWall.setId(1)), geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((badWall.setMap(p_map)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((badWall.setBad()), geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&badWall)), MapStatus::MAP_STATUS_SUCCESS);

    semantic::Passage defaultPassage;
    ASSERT_EQ((defaultPassage.setId(50)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((defaultPassage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPassage(&defaultPassage)),
              MapStatus::MAP_STATUS_SUCCESS);

    semantic::Room room;
    ASSERT_EQ((room.setId(30)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(&badWall)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setDoorways(&defaultPassage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    unsigned long mapIdValue{};
    ASSERT_EQ((p_map->getId(mapIdValue)), MapStatus::MAP_STATUS_SUCCESS);
    const long unsigned int mapId = static_cast<long unsigned int>(mapIdValue);
    AtlasStatus             exportStatus{};
    EXPECT_NO_THROW(exportStatus = atlas.exportRoomContextFromCurrentMap());
    EXPECT_EQ(exportStatus, AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
        history{};
    ASSERT_EQ((atlas.copyRoomContextHistory(history)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(history.count(mapId), 1U);
    const semantic::RoomContextSnapshot &snap = history.at(mapId).front();
    ASSERT_EQ(snap.wallBounds.size(), 1U);
    EXPECT_FALSE(snap.wallBounds.front().isValid);
    ASSERT_EQ(snap.passageContexts.size(), 1U);
    EXPECT_FALSE(snap.passageContexts.front().isApertureValid);
    EXPECT_FALSE(snap.passageContexts.front().hasFarSideRoom);
}

TEST(RoomContextPersist, ClearMapBumpsRevisionGeneration)
{
    /* Same-map clears keep the map id, so the visualization/voxblox
     * revision token would not observe the reset and stale markers and
     * clouds would persist alongside the fresh map. The clear must bump
     * the generation the token is built from. */
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    int changeIndexBefore{};
    ASSERT_EQ((p_map->getLastBigChangeIndex(changeIndexBefore)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((atlas.clearMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);

    int lastBigChangeIndex{};
    ASSERT_EQ((p_map->getLastBigChangeIndex(lastBigChangeIndex)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(lastBigChangeIndex, changeIndexBefore + 1);
}

TEST(RoomContextPersist, VisitedFlagRoundTripsThroughExport)
{
    Atlas atlas(0);
    Map  *p_oldMap = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_oldMap)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    semantic::Room visitedRoom;
    ASSERT_EQ((visitedRoom.setId(5)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((visitedRoom.setMap(p_oldMap)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((visitedRoom.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((visitedRoom.setPreviouslyVisited(true)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_oldMap->addDetectedMapRoom(&visitedRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    semantic::Room freshRoom;
    ASSERT_EQ((freshRoom.setId(6)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((freshRoom.setMap(p_oldMap)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((freshRoom.setCentroid(Eigen::Vector3d(2.0, 2.0, 2.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_oldMap->addDetectedMapRoom(&freshRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    unsigned long oldMapIdValue{};
    ASSERT_EQ((p_oldMap->getId(oldMapIdValue)), MapStatus::MAP_STATUS_SUCCESS);
    const long unsigned int oldMapId =
        static_cast<long unsigned int>(oldMapIdValue);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
        history{};
    ASSERT_EQ((atlas.copyRoomContextHistory(history)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
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
    Map  *p_oldMap = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_oldMap)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane wall;
    makeRefitWallPlane(wall, 1, p_oldMap);
    ASSERT_EQ((p_oldMap->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    semantic::Room room;
    ASSERT_EQ((room.setId(5)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_oldMap)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(&wall)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_oldMap->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    unsigned long oldMapIdValue{};
    ASSERT_EQ((p_oldMap->getId(oldMapIdValue)), MapStatus::MAP_STATUS_SUCCESS);
    const long unsigned int oldMapId =
        static_cast<long unsigned int>(oldMapIdValue);

    /* Genuine tracking-loss/new-map boundary: Tracking.cc calls
     * Atlas::CreateNewMap() directly; it internally exports the outgoing
     * map's room context before installing the new current map. */
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_newMap = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_newMap)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_NE(p_newMap, p_oldMap);

    std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
        history{};
    ASSERT_EQ((atlas.copyRoomContextHistory(history)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ(history.count(oldMapId), 1U);
    ASSERT_EQ(history.at(oldMapId).size(), 1U);
    EXPECT_EQ(history.at(oldMapId).front().roomId, 5);
    EXPECT_EQ(history.at(oldMapId).front().roomTag, "room_5");
    ASSERT_EQ(history.at(oldMapId).front().wallBounds.size(), 1U);
    EXPECT_TRUE(history.at(oldMapId).front().wallBounds.front().isValid);

    /* Map identity is carried by the history container key alone; no
     * separate snapshot-level mapId is introduced. */
    unsigned long id{};
    ASSERT_EQ((p_newMap->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(history.count(id), 0U);
}

TEST(RoomContextPersist, HundredRepeatedExportResetChecksPass)
{
    unsigned int successCount = 0U;
    for (unsigned int iteration = 0U; iteration < 100U; ++iteration)
    {
        Atlas atlas(0);
        Map  *p_oldMap = nullptr;
        ASSERT_EQ((atlas.getCurrentMap(p_oldMap)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);

        geometric::Plane wall;
        makeRefitWallPlane(wall, 1, p_oldMap);
        ASSERT_EQ((p_oldMap->addMapPlane(&wall)),
                  MapStatus::MAP_STATUS_SUCCESS);

        semantic::Room room;
        ASSERT_EQ((room.setId(static_cast<int>(iteration))),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ASSERT_EQ((room.setMap(p_oldMap)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ASSERT_EQ((room.setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0))),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ASSERT_EQ((room.setWalls(&wall)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ASSERT_EQ((p_oldMap->addDetectedMapRoom(&room)),
                  MapStatus::MAP_STATUS_SUCCESS);

        unsigned long oldMapIdValue{};
        ASSERT_EQ((p_oldMap->getId(oldMapIdValue)),
                  MapStatus::MAP_STATUS_SUCCESS);
        const long unsigned int oldMapId =
            static_cast<long unsigned int>(oldMapIdValue);
        ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);

        std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
            history{};
        ASSERT_EQ((atlas.copyRoomContextHistory(history)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);
        const bool ok = history.count(oldMapId) == 1U &&
                        history.at(oldMapId).size() == 1U &&
                        history.at(oldMapId).front().roomId ==
                            static_cast<int>(iteration) &&
                        history.at(oldMapId).front().wallBounds.size() == 1U &&
                        history.at(oldMapId).front().wallBounds.front().isValid;
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
    std::vector<std::unique_ptr<semantic::Room>>   rooms;

    std::atomic<bool> readerComplete{false};
    std::thread       reader(
        [&atlas, &readerComplete]()
        {
            for (unsigned int iteration = 0U; iteration < 100U; ++iteration)
            {
                std::map<unsigned long,
                               std::vector<semantic::RoomContextSnapshot>>
                    history{};
                if (atlas.copyRoomContextHistory(history) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: copyRoomContextHistory returned a failure status "
                              "although it cannot fail; continuing as before.",
                        __func__);
                }
                for (const auto &entry : history)
                {
                    for (const semantic::RoomContextSnapshot &snap :
                         entry.second)
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
        Map *p_map = nullptr;
        ASSERT_EQ((atlas.getCurrentMap(p_map)),
                  AtlasStatus::ATLAS_STATUS_SUCCESS);

        auto p_wall = std::make_unique<geometric::Plane>();
        makeRefitWallPlane(*p_wall, static_cast<int>(iteration) + 1, p_map);
        ASSERT_EQ((p_map->addMapPlane(p_wall.get())),
                  MapStatus::MAP_STATUS_SUCCESS);

        auto p_room = std::make_unique<semantic::Room>();
        ASSERT_EQ((p_room->setId(static_cast<int>(iteration))),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ASSERT_EQ((p_room->setMap(p_map)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ASSERT_EQ((p_room->setCentroid(Eigen::Vector3d(1.0, 1.0, 1.0))),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ASSERT_EQ((p_room->setWalls(p_wall.get())),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ASSERT_EQ((p_map->addDetectedMapRoom(p_room.get())),
                  MapStatus::MAP_STATUS_SUCCESS);

        walls.push_back(std::move(p_wall));
        rooms.push_back(std::move(p_room));

        ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    }

    reader.join();
    EXPECT_TRUE(readerComplete.load(std::memory_order_acquire));
}

} // namespace core
} // namespace vs_graphs
