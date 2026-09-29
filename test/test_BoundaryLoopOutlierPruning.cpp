/*!
 * @file test_BoundaryLoopOutlierPruning.cpp
 * @brief User rule: once a room's true boundary closes into a valid loop,
 *        any wall the room still owns that is NOT part of that loop, and
 *        that no passage tied to the room explains, is invalid and must be
 *        detached -- rather than the presence of that one stray wall
 *        silently blocking the whole room from ever reaching COMPLETE.
 */

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "Map.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "SemanticsManager.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <memory>

namespace vs_graphs
{
namespace core
{
namespace
{

/*! Builds a GROUND geometric::Plane at z=0 with a genuine, production-refit
 * geometry snapshot (Map::GetBiggestGroundPlane() requires cloudGeneration ==
 * successfulRefitGeneration and a finite support count). */
std::unique_ptr<geometric::Plane> makeRefitGroundPlaneAtOrigin(int  id_in,
                                                               Map *p_map_in)
{
    auto ground = std::make_unique<geometric::Plane>();
    if (ground->setId(id_in) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // setId cannot fail; continue as before.
    }
    if (ground->setMap(p_map_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // setMap cannot fail; continue as before.
    }
    if (ground->setPlaneType(geometric::Plane::PlaneVariant::GROUND) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // setPlaneType cannot fail; continue as before.
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int index = 0; index < 50; ++index)
    {
        pcl::PointXYZRGBA point;
        point.x = static_cast<float>(index) * 0.1f - 2.5f;
        point.y = static_cast<float>(index) * 0.05f - 1.25f;
        point.z = 0.0f;
        cloud->push_back(point);
    }
    if (ground->replaceMapClouds(cloud) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // replaceMapClouds cannot fail; continue as before.
    }

    std::optional<geometric::Plane::GeometrySnapshot> snapshot{};
    if (ground->beginMapCloudRefit(snapshot) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // beginMapCloudRefit cannot fail; continue as before.
    }
    bool wasRefitPublished{};
    if (ground->completeMapCloudRefit(
            snapshot->cloudGeneration,
            Eigen::Vector3d::Zero(),
            g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0)),
            50U,
            wasRefitPublished) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // completeMapCloudRefit cannot fail; continue as before.
    }
    return ground;
}

/*! Builds an admissible WALL geometric::Plane on the plane {normal . p + d = 0}
 * passing through pointOnPlane_World_in, with a genuine on-plane point
 * cloud running along axisAlong_World_in (must be horizontal) for
 * [-halfLength_m_in, halfLength_m_in] and vertically for [zMin_m_in,
 * zMax_m_in]. */
std::unique_ptr<geometric::Plane>
    makeWallSegmentPlane(int                    id_in,
                         Map                   *p_map_in,
                         const Eigen::Vector3d &normal_World_in,
                         const Eigen::Vector3d &pointOnPlane_World_in,
                         const Eigen::Vector3d &axisAlong_World_in,
                         double                 halfLength_m_in,
                         double                 zMin_m_in,
                         double                 zMax_m_in)
{
    auto wall = std::make_unique<geometric::Plane>();
    if (wall->setId(id_in) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // setId cannot fail; continue as before.
    }
    if (wall->setMap(p_map_in) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // setMap cannot fail; continue as before.
    }
    if (wall->setPlaneType(geometric::Plane::PlaneVariant::WALL) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // setPlaneType cannot fail; continue as before.
    }
    if (wall->castWeightedVote(geometric::Plane::PlaneVariant::WALL, 1.0) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // castWeightedVote cannot fail; continue as before.
    }

    const double d = -normal_World_in.dot(pointOnPlane_World_in);
    if (wall->setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(
            normal_World_in.x(),
            normal_World_in.y(),
            normal_World_in.z(),
            d))) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // setGlobalEquation cannot fail; continue as before.
    }
    if (wall->setCentroid(pointOnPlane_World_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // setCentroid cannot fail; continue as before.
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    constexpr int steps = 12;
    for (int alongIndex = 0; alongIndex < steps; ++alongIndex)
    {
        const double t =
            -halfLength_m_in + 2.0 * halfLength_m_in *
                                   static_cast<double>(alongIndex) /
                                   static_cast<double>(steps - 1);
        for (int heightIndex = 0; heightIndex < steps; ++heightIndex)
        {
            const double z = zMin_m_in + (zMax_m_in - zMin_m_in) *
                                             static_cast<double>(heightIndex) /
                                             static_cast<double>(steps - 1);
            pcl::PointXYZRGBA point;
            point.x = static_cast<float>(pointOnPlane_World_in.x() +
                                         t * axisAlong_World_in.x());
            point.y = static_cast<float>(pointOnPlane_World_in.y() +
                                         t * axisAlong_World_in.y());
            point.z = static_cast<float>(z);
            cloud->push_back(point);
        }
    }
    if (wall->setMapClouds(cloud) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // setMapClouds cannot fail; continue as before.
    }
    return wall;
}

/*! Builds a 4-wall rectangular loop (x in [0,4], y in [0,3], z in [1,2])
 * plus a 5th wall far away and disconnected from it. */
struct RectangleWithOutlier
{
    std::unique_ptr<geometric::Plane> north, south, east, west, outlier;
};

RectangleWithOutlier makeRectangleWithOutlier(Map *p_map_in)
{
    RectangleWithOutlier walls;
    walls.north = makeWallSegmentPlane(1,
                                       p_map_in,
                                       Eigen::Vector3d(0.0, 1.0, 0.0),
                                       Eigen::Vector3d(2.0, 3.0, 1.5),
                                       Eigen::Vector3d(1.0, 0.0, 0.0),
                                       2.0,
                                       1.0,
                                       2.0);
    walls.south = makeWallSegmentPlane(2,
                                       p_map_in,
                                       Eigen::Vector3d(0.0, -1.0, 0.0),
                                       Eigen::Vector3d(2.0, 0.0, 1.5),
                                       Eigen::Vector3d(1.0, 0.0, 0.0),
                                       2.0,
                                       1.0,
                                       2.0);
    walls.east  = makeWallSegmentPlane(3,
                                      p_map_in,
                                      Eigen::Vector3d(1.0, 0.0, 0.0),
                                      Eigen::Vector3d(4.0, 1.5, 1.5),
                                      Eigen::Vector3d(0.0, 1.0, 0.0),
                                      1.5,
                                      1.0,
                                      2.0);
    walls.west  = makeWallSegmentPlane(4,
                                      p_map_in,
                                      Eigen::Vector3d(-1.0, 0.0, 0.0),
                                      Eigen::Vector3d(0.0, 1.5, 1.5),
                                      Eigen::Vector3d(0.0, 1.0, 0.0),
                                      1.5,
                                      1.0,
                                      2.0);
    /* Disconnected: same normal direction as `east`, but 16 m away, so it
     * cannot participate in closing the same loop. */
    walls.outlier = makeWallSegmentPlane(5,
                                         p_map_in,
                                         Eigen::Vector3d(1.0, 0.0, 0.0),
                                         Eigen::Vector3d(20.0, 21.5, 1.5),
                                         Eigen::Vector3d(0.0, 1.0, 0.0),
                                         1.5,
                                         1.0,
                                         2.0);
    return walls;
}

} // namespace

TEST(BoundaryLoopOutlierPruning, ClosesTheLoopAndDetachesAnUnexplainedOutlier)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> ground =
        makeRefitGroundPlaneAtOrigin(100, p_map);
    atlas.addMapPlane(ground.get());

    RectangleWithOutlier walls = makeRectangleWithOutlier(p_map);

    semantic::Room room;
    ASSERT_EQ((room.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(2.0, 1.5, 1.5))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(walls.north.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(walls.south.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(walls.east.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(walls.west.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(walls.outlier.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    atlas.addDetectedMapRoom(&room);

    manager.validateRoomBoundariesForTest();

    vs_graphs::core::semantic::Room::BoundaryStatus boundaryStatus{};
    ASSERT_EQ((room.getBoundaryStatus(boundaryStatus)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(boundaryStatus, semantic::Room::BoundaryStatus::COMPLETE);
    std::vector<Eigen::Vector3d> boundaryCorners_World_m{};
    ASSERT_EQ((room.getBoundaryCorners_World_m(boundaryCorners_World_m)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_GE(boundaryCorners_World_m.size(), 3U);

    std::vector<geometric::Plane *> remainingWalls{};
    ASSERT_EQ((room.getWalls(remainingWalls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(remainingWalls.size(), 4U);
    EXPECT_EQ(std::find(remainingWalls.begin(),
                        remainingWalls.end(),
                        walls.outlier.get()),
              remainingWalls.end());
    for (geometric::Plane *p_loopWall : {walls.north.get(),
                                         walls.south.get(),
                                         walls.east.get(),
                                         walls.west.get()})
    {
        EXPECT_NE(
            std::find(remainingWalls.begin(), remainingWalls.end(), p_loopWall),
            remainingWalls.end());
    }
}

TEST(BoundaryLoopOutlierPruning, KeepsAnOutlierExplainedByAPassage)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> ground =
        makeRefitGroundPlaneAtOrigin(100, p_map);
    atlas.addMapPlane(ground.get());

    RectangleWithOutlier walls = makeRectangleWithOutlier(p_map);

    semantic::Room room;
    ASSERT_EQ((room.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(2.0, 1.5, 1.5))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(walls.north.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(walls.south.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(walls.east.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(walls.west.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(walls.outlier.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    atlas.addDetectedMapRoom(&room);

    /* The outlier is a doorway wall framing a passage out of this room --
     * explained, so it must survive even though it is off the closed loop.
     */
    semantic::Passage passage;
    ASSERT_EQ((passage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.addAssociateWall(walls.outlier.get())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((room.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    manager.validateRoomBoundariesForTest();

    vs_graphs::core::semantic::Room::BoundaryStatus boundaryStatus{};
    ASSERT_EQ((room.getBoundaryStatus(boundaryStatus)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(boundaryStatus, semantic::Room::BoundaryStatus::COMPLETE);

    std::vector<geometric::Plane *> remainingWalls{};
    ASSERT_EQ((room.getWalls(remainingWalls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(remainingWalls.size(), 5U);
    EXPECT_NE(std::find(remainingWalls.begin(),
                        remainingWalls.end(),
                        walls.outlier.get()),
              remainingWalls.end());
}

} // namespace core
} // namespace vs_graphs
