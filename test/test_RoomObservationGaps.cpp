/*!
 * @file test_RoomObservationGaps.cpp
 * @brief User rule: manage incomplete rooms explicitly -- track which
 *        angular sectors around a room still have no wall evidence, as a
 *        situational-awareness signal (independent of, and available
 *        earlier than, semantic::BoundaryStatus::COMPLETE).
 */

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "Map.h"
#include "Semantic/Room.h"
#include "SemanticsManager.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>

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
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (ground->setMap(p_map_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (ground->setPlaneType(geometric::Plane::PlaneVariant::GROUND) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPlaneType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
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
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: replaceMapClouds returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    std::optional<geometric::Plane::GeometrySnapshot> snapshot{};
    if (ground->beginMapCloudRefit(snapshot) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: beginMapCloudRefit returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool wasRefitPublished{};
    if (ground->completeMapCloudRefit(
            snapshot->cloudGeneration,
            Eigen::Vector3d::Zero(),
            g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0)),
            50U,
            wasRefitPublished) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: completeMapCloudRefit returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    return ground;
}

/*! Builds an admissible WALL geometric::Plane on the plane {normal . p + d = 0}
 * passing through pointOnPlane_World_in, with a genuine on-plane point cloud
 * running along axisAlong_World_in (must be horizontal). */
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
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (wall->setMap(p_map_in) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (wall->setPlaneType(geometric::Plane::PlaneVariant::WALL) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPlaneType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (wall->castWeightedVote(geometric::Plane::PlaneVariant::WALL, 1.0) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: castWeightedVote returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    const double d = -normal_World_in.dot(pointOnPlane_World_in);
    if (wall->setGlobalEquation(g2o::Plane3D(Eigen::Vector4d(
            normal_World_in.x(),
            normal_World_in.y(),
            normal_World_in.z(),
            d))) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (wall->setCentroid(pointOnPlane_World_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
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
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMapClouds returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    return wall;
}

} // namespace

TEST(RoomObservationGaps, ReportsAFullCircleGapForARoomWithNoWalls)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> ground =
        makeRefitGroundPlaneAtOrigin(100, p_map);
    atlas.addMapPlane(ground.get());

    semantic::Room room;
    ASSERT_EQ((room.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(2.0, 1.5, 1.5))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    atlas.addDetectedMapRoom(&room);

    manager.validateRoomBoundariesForTest();

    std::vector<semantic::Room::ObservationGap> gaps{};
    ASSERT_EQ((room.getObservationGaps(gaps)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(gaps.size(), 1U);
    EXPECT_NEAR(gaps.front().spanAngle_rad, 2.0 * M_PI, 1e-6);
}

TEST(RoomObservationGaps, ReportsALargeGapForARoomWithOnlyOneWall)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> ground =
        makeRefitGroundPlaneAtOrigin(100, p_map);
    atlas.addMapPlane(ground.get());

    /* A single wall at y=3 (north side), facing the room centre. */
    std::unique_ptr<geometric::Plane> northWall =
        makeWallSegmentPlane(1,
                             p_map,
                             Eigen::Vector3d(0.0, 1.0, 0.0),
                             Eigen::Vector3d(2.0, 3.0, 1.5),
                             Eigen::Vector3d(1.0, 0.0, 0.0),
                             2.0,
                             1.0,
                             2.0);

    semantic::Room room;
    ASSERT_EQ((room.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(2.0, 1.5, 1.5))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(northWall.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    atlas.addDetectedMapRoom(&room);

    manager.validateRoomBoundariesForTest();

    /* One wall gives one point on the circle -- the gap wraps all the way
     * around back to that same point, i.e. one (nearly) full-circle gap. */
    std::vector<semantic::Room::ObservationGap> gaps{};
    ASSERT_EQ((room.getObservationGaps(gaps)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(gaps.size(), 1U);
    EXPECT_GT(gaps.front().spanAngle_rad, M_PI);
}

TEST(RoomObservationGaps, ReportsNoGapsForARoomWithACompleteBoundary)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> ground =
        makeRefitGroundPlaneAtOrigin(100, p_map);
    atlas.addMapPlane(ground.get());

    std::unique_ptr<geometric::Plane> north =
        makeWallSegmentPlane(1,
                             p_map,
                             Eigen::Vector3d(0.0, 1.0, 0.0),
                             Eigen::Vector3d(2.0, 3.0, 1.5),
                             Eigen::Vector3d(1.0, 0.0, 0.0),
                             2.0,
                             1.0,
                             2.0);
    std::unique_ptr<geometric::Plane> south =
        makeWallSegmentPlane(2,
                             p_map,
                             Eigen::Vector3d(0.0, -1.0, 0.0),
                             Eigen::Vector3d(2.0, 0.0, 1.5),
                             Eigen::Vector3d(1.0, 0.0, 0.0),
                             2.0,
                             1.0,
                             2.0);
    std::unique_ptr<geometric::Plane> east =
        makeWallSegmentPlane(3,
                             p_map,
                             Eigen::Vector3d(1.0, 0.0, 0.0),
                             Eigen::Vector3d(4.0, 1.5, 1.5),
                             Eigen::Vector3d(0.0, 1.0, 0.0),
                             1.5,
                             1.0,
                             2.0);
    std::unique_ptr<geometric::Plane> west =
        makeWallSegmentPlane(4,
                             p_map,
                             Eigen::Vector3d(-1.0, 0.0, 0.0),
                             Eigen::Vector3d(0.0, 1.5, 1.5),
                             Eigen::Vector3d(0.0, 1.0, 0.0),
                             1.5,
                             1.0,
                             2.0);

    semantic::Room room;
    ASSERT_EQ((room.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setCentroid(Eigen::Vector3d(2.0, 1.5, 1.5))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(north.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(south.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(east.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setWalls(west.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    atlas.addDetectedMapRoom(&room);

    manager.validateRoomBoundariesForTest();

    vs_graphs::core::semantic::Room::BoundaryStatus boundaryStatus{};
    ASSERT_EQ((room.getBoundaryStatus(boundaryStatus)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(boundaryStatus, semantic::Room::BoundaryStatus::COMPLETE);
    /* Four evenly-spaced walls around a rectangle are each exactly 90 deg
     * apart (wall midpoints sit on the centroid's principal axes) --
     * comfortably under the (100 deg) gap-reporting threshold. */
    std::vector<vs_graphs::core::semantic::Room::ObservationGap>
        observationGaps{};
    ASSERT_EQ((room.getObservationGaps(observationGaps)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(observationGaps.empty());
}

} // namespace core
} // namespace vs_graphs
