/**
 * @file test_FloorFlatness.cpp
 * @brief Front-end floor-flatness coverage:
 *        SemanticsManager::reconcileRoomGroundPlanes() re-points a room
 *        whose own ground plane disagrees with the just-refreshed canonical
 *        semantic::Floor identity.
 */

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "Map.h"
#include "Semantic/Room.h"
#include "SemanticsManager.h"

#include <gtest/gtest.h>

#include <memory>

namespace vs_graphs
{
namespace core
{
namespace
{

/** Builds a GROUND geometric::Plane at height y = height_m_in with a genuine,
 * production-refit geometry snapshot (Map::GetBiggestGroundPlane() and
 * semantic::Floor::selectBestObservedFloor() both require cloudGeneration ==
 * successfulRefitGeneration and a finite support count, which only
 * geometric::Plane::completeMapCloudRefit() sets). */
std::unique_ptr<geometric::Plane>
    makeRefitGroundPlane(int id_in, Map *p_map_in, double height_m_in,
                        std::size_t pointCount_in)
{
    auto ground = std::make_unique<geometric::Plane>();
    ground->setId(id_in);
    ground->SetMap(p_map_in);
    ground->setPlaneType(geometric::Plane::planeVariant::GROUND);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (std::size_t index = 0; index < pointCount_in; ++index)
    {
        pcl::PointXYZRGBA point;
        point.x = static_cast<float>(index) * 0.1f;
        point.y = static_cast<float>(height_m_in);
        point.z = static_cast<float>(index) * 0.05f;
        cloud->push_back(point);
    }
    ground->replaceMapClouds(cloud);

    const auto snapshot = ground->beginMapCloudRefit();
    ground->completeMapCloudRefit(snapshot->cloudGeneration,
                                  Eigen::Vector3d(0.0, height_m_in, 0.0),
                                  g2o::Plane3D(Eigen::Vector4d(
                                      0.0, 1.0, 0.0, -height_m_in)),
                                  pointCount_in);
    return ground;
}

} // namespace

TEST(FloorFlatness, RepointsALessObservedRoomGroundPlaneToTheCanonicalOne)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);

    /* Canonical, strongly observed ground plane at y=0. */
    std::unique_ptr<geometric::Plane> canonicalGround =
        makeRefitGroundPlane(1, p_map, 0.0, 500U);
    atlas.AddMapPlane(canonicalGround.get());

    /* A weaker, disagreeing ground plane 0.5 m above -- well beyond
     * semantic::Floor::kMergeMaxPlaneOffset_m (0.35 m). */
    std::unique_ptr<geometric::Plane> roomGround =
        makeRefitGroundPlane(2, p_map, 0.5, 20U);
    atlas.AddMapPlane(roomGround.get());

    std::unique_ptr<semantic::Room> room = std::make_unique<semantic::Room>();
    room->setId(1);
    room->setMap(p_map);
    room->setRoomVariant(semantic::Room::RoomVariant::ROOM);
    room->setGroundPlane(roomGround.get());
    atlas.AddDetectedMapRoom(room.get());

    /* Establish the canonical semantic::Floor identity first, as Run() does. */
    manager.getUpdatedFloorsForTest();
    manager.reconcileRoomGroundPlanesForTest();

    EXPECT_EQ(room->getGroundPlane(), canonicalGround.get());
}

TEST(FloorFlatness, LeavesAnAgreeingRoomGroundPlaneUntouched)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> canonicalGround =
        makeRefitGroundPlane(1, p_map, 0.0, 500U);
    atlas.AddMapPlane(canonicalGround.get());

    /* Within tolerance: 0.05 m offset, well under 0.35 m. */
    std::unique_ptr<geometric::Plane> roomGround =
        makeRefitGroundPlane(2, p_map, 0.05, 20U);
    atlas.AddMapPlane(roomGround.get());

    std::unique_ptr<semantic::Room> room = std::make_unique<semantic::Room>();
    room->setId(1);
    room->setMap(p_map);
    room->setRoomVariant(semantic::Room::RoomVariant::ROOM);
    room->setGroundPlane(roomGround.get());
    atlas.AddDetectedMapRoom(room.get());

    manager.getUpdatedFloorsForTest();
    manager.reconcileRoomGroundPlanesForTest();

    EXPECT_EQ(room->getGroundPlane(), roomGround.get());
}

} // namespace core
} // namespace vs_graphs
