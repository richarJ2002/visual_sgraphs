/*!
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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace
{

/*! Builds a GROUND geometric::Plane at height y = height_m_in with a genuine,
 * production-refit geometry snapshot (Map::GetBiggestGroundPlane() and
 * semantic::Floor::selectBestObservedFloor() both require cloudGeneration ==
 * successfulRefitGeneration and a finite support count, which only
 * geometric::Plane::completeMapCloudRefit() sets). */
std::unique_ptr<geometric::Plane>
    makeRefitGroundPlane(int         id_in,
                         Map        *p_map_in,
                         double      height_m_in,
                         std::size_t pointCount_in)
{
    std::unique_ptr<geometric::Plane> ground =
        std::make_unique<geometric::Plane>();
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
    for (std::size_t index = 0; index < pointCount_in; ++index)
    {
        pcl::PointXYZRGBA point;
        point.x = static_cast<float>(index) * 0.1f;
        point.y = static_cast<float>(height_m_in);
        point.z = static_cast<float>(index) * 0.05f;
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
            Eigen::Vector3d(0.0, height_m_in, 0.0),
            g2o::Plane3D(Eigen::Vector4d(0.0, 1.0, 0.0, -height_m_in)),
            pointCount_in,
            wasRefitPublished) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: completeMapCloudRefit returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    return ground;
}

} // namespace

TEST(FloorFlatness, RepointsALessObservedRoomGroundPlaneToTheCanonicalOne)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    /* Canonical, strongly observed ground plane at y=0. */
    std::unique_ptr<geometric::Plane> canonicalGround =
        makeRefitGroundPlane(1, p_map, 0.0, 500U);
    ASSERT_EQ((atlas.addMapPlane(canonicalGround.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    /* A weaker, disagreeing ground plane 0.5 m above -- well beyond
     * semantic::Floor::kMergeMaxPlaneOffset_m (0.35 m). */
    std::unique_ptr<geometric::Plane> roomGround =
        makeRefitGroundPlane(2, p_map, 0.5, 20U);
    ASSERT_EQ((atlas.addMapPlane(roomGround.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::unique_ptr<semantic::Room> room = std::make_unique<semantic::Room>();
    ASSERT_EQ((room->setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room->setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room->setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room->setGroundPlane(roomGround.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addDetectedMapRoom(room.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    /* Establish the canonical semantic::Floor identity first, as Run() does. */
    ASSERT_EQ((manager.getUpdatedFloorsForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.reconcileRoomGroundPlanesForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    vs_graphs::core::geometric::Plane *p_groundPlane = nullptr;
    ASSERT_EQ((room->getGroundPlane(p_groundPlane)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(p_groundPlane, canonicalGround.get());
}

TEST(FloorFlatness, LeavesAnAgreeingRoomGroundPlaneUntouched)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticsManager manager(&atlas);

    std::unique_ptr<geometric::Plane> canonicalGround =
        makeRefitGroundPlane(1, p_map, 0.0, 500U);
    ASSERT_EQ((atlas.addMapPlane(canonicalGround.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    /* Within tolerance: 0.05 m offset, well under 0.35 m. */
    std::unique_ptr<geometric::Plane> roomGround =
        makeRefitGroundPlane(2, p_map, 0.05, 20U);
    ASSERT_EQ((atlas.addMapPlane(roomGround.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    std::unique_ptr<semantic::Room> room = std::make_unique<semantic::Room>();
    ASSERT_EQ((room->setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room->setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room->setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room->setGroundPlane(roomGround.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addDetectedMapRoom(room.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    ASSERT_EQ((manager.getUpdatedFloorsForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);
    ASSERT_EQ((manager.reconcileRoomGroundPlanesForTest()),
              SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS);

    vs_graphs::core::geometric::Plane *p_groundPlane = nullptr;
    ASSERT_EQ((room->getGroundPlane(p_groundPlane)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(p_groundPlane, roomGround.get());
}

} // namespace core
} // namespace vs_graphs
