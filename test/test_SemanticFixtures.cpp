/*!
 * @file test_SemanticFixtures.cpp
 * @brief Self-test for the deterministic semantic-fixture builders
 *. Each case exercises one
 *        builder and asserts the object it produced is what later
 *        semantic-axiom-plan phases will assume: correctly wired, and
 *        deterministic across repeated construction.
 */

#include "SemanticFixtures.h"

#include <gtest/gtest.h>

#include <algorithm>

namespace vs_graphs
{
namespace core
{
namespace test
{
namespace
{

TEST(SemanticFixtures, GridCloudIsDeterministicAndOnPlane)
{
    const Eigen::Vector3d centroid(1.0, 2.0, 3.0);
    const Eigen::Vector3d axisU(0.0, 1.0, 0.0);
    const Eigen::Vector3d axisV(0.0, 0.0, 1.0);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA =
        makeGridCloud(centroid, axisU, axisV, 0.5, 0.75, 6);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB =
        makeGridCloud(centroid, axisU, axisV, 0.5, 0.75, 6);

    ASSERT_EQ(cloudA->size(), cloudB->size());
    ASSERT_EQ(cloudA->size(), static_cast<std::size_t>(6 * 6));
    for (std::size_t index = 0; index < cloudA->size(); ++index)
    {
        EXPECT_FLOAT_EQ(cloudA->points[index].x, cloudB->points[index].x);
        EXPECT_FLOAT_EQ(cloudA->points[index].y, cloudB->points[index].y);
        EXPECT_FLOAT_EQ(cloudA->points[index].z, cloudB->points[index].z);
        /* Every point lies on the X = 1.0 plane (normal along world X). */
        EXPECT_NEAR(cloudA->points[index].x, 1.0F, 1e-5F);
    }
}

TEST(SemanticFixtures, MakeWallPlaneProducesAdmissibleGeometry)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane wall;
    makeWallPlane(wall,
                  7,
                  p_map,
                  Eigen::Vector4d(1.0, 0.0, 0.0, -2.0),
                  Eigen::Vector3d(0.0, 1.0, 0.0),
                  Eigen::Vector3d(0.0, 0.0, 1.0),
                  1.0,
                  1.1,
                  Eigen::Vector3d(2.0, 0.0, 1.0));

    int getId2{};
    ASSERT_EQ((wall.getId(getId2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(getId2, 7);
    geometric::Plane::PlaneVariant planeType{};
    ASSERT_EQ((wall.getPlaneType(planeType)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(planeType, geometric::Plane::PlaneVariant::WALL);
    Eigen::Vector3d getCentroid2{};
    ASSERT_EQ((wall.getCentroid(getCentroid2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_TRUE(getCentroid2.isApprox(Eigen::Vector3d(2.0, 0.0, 1.0)));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mapClouds{};
    ASSERT_EQ((wall.getMapClouds(mapClouds)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_GT(mapClouds->size(), 0U);
    /* The equation's normal survives normalization; only its sign is
     * arbitrary until oriented toward a room. */
    g2o::Plane3D getGlobalEquation2{};
    ASSERT_EQ((wall.getGlobalEquation(getGlobalEquation2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_NEAR(std::abs(getGlobalEquation2.coeffs().head<3>().dot(
                    Eigen::Vector3d::UnitX())),
                1.0,
                1e-6);
}

TEST(SemanticFixtures, MakeGroundPlaneRefitsSuccessfully)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane ground;
    const bool       refitOk = makeGroundPlane(ground, 1, p_map);

    EXPECT_TRUE(refitOk);
    geometric::Plane::PlaneVariant planeType{};
    ASSERT_EQ((ground.getPlaneType(planeType)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_EQ(planeType, geometric::Plane::PlaneVariant::GROUND);
}

TEST(SemanticFixtures, MakeRoomAttachesWallAndCentroid)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane wall;
    makeWallPlane(wall,
                  1,
                  p_map,
                  Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                  Eigen::Vector3d(0.0, 1.0, 0.0),
                  Eigen::Vector3d(0.0, 0.0, 1.0),
                  1.0,
                  1.0);

    semantic::Room room;
    makeRoom(room, 5, p_map, &wall, Eigen::Vector3d(3.0, 4.0, 0.0));

    int id{};
    ASSERT_EQ((room.getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(id, 5);
    vs_graphs::core::semantic::Room::RoomVariant roomVariant{};
    ASSERT_EQ((room.getRoomVariant(roomVariant)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(roomVariant, semantic::Room::RoomVariant::ROOM);
    Eigen::Vector3d centroid{};
    ASSERT_EQ((room.getCentroid(centroid)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(centroid.isApprox(Eigen::Vector3d(3.0, 4.0, 0.0)));
    std::vector<vs_graphs::core::geometric::Plane *> walls{};
    ASSERT_EQ((room.getWalls(walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(walls.size(), 1U);
    EXPECT_EQ(walls.front(), &wall);
}

TEST(SemanticFixtures, MakePassageWiresKnownAndFarSide)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    semantic::Room knownRoom;
    makeRoom(knownRoom, 1, p_map, nullptr, Eigen::Vector3d(-1.0, 0.0, 0.0));
    semantic::Room farRoom;
    makeRoom(farRoom, 2, p_map, nullptr, Eigen::Vector3d(1.0, 0.0, 0.0));

    semantic::Passage passage;
    makePassage(passage,
                9,
                p_map,
                Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                Eigen::Vector3d::Zero(),
                &knownRoom,
                Eigen::Vector3d(-1.0, 0.0, 0.0),
                &farRoom);

    int id{};
    ASSERT_EQ((passage.getId(id)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(id, 9);
    /* makePassage never claims traversal evidence on the caller's behalf. */
    bool traversalEvidence{};
    ASSERT_EQ((passage.getTraversalEvidence(traversalEvidence)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_FALSE(traversalEvidence);
    vs_graphs::core::semantic::Passage::KnownSideProvenance
        knownSideProvenance{};
    ASSERT_EQ((passage.getKnownSideProvenance(knownSideProvenance)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(knownSideProvenance.p_room, &knownRoom);
    vs_graphs::core::semantic::Passage::KnownSideProvenance
        knownSideProvenance2{};
    ASSERT_EQ((passage.getKnownSideProvenance(knownSideProvenance2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(knownSideProvenance2.knownSideDirection_world.isApprox(
        Eigen::Vector3d(-1.0, 0.0, 0.0)));
    vs_graphs::core::semantic::Room *p_prospectiveRoom = nullptr;
    ASSERT_EQ((passage.getProspectiveRoom(p_prospectiveRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(p_prospectiveRoom, &farRoom);
}

TEST(SemanticFixtures, MakeFloorOwnsGivenRooms)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    semantic::Room roomA;
    makeRoom(roomA, 1, p_map, nullptr);
    semantic::Room roomB;
    makeRoom(roomB, 2, p_map, nullptr);

    semantic::Floor floor;
    makeFloor(floor, 3, p_map, {&roomA, &roomB}, 0.0);

    std::vector<vs_graphs::core::semantic::Room *> rooms{};
    ASSERT_EQ((floor.getRooms(rooms)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ(rooms.size(), 2U);
    EXPECT_NE(std::find(rooms.begin(), rooms.end(), &roomA), rooms.end());
    EXPECT_NE(std::find(rooms.begin(), rooms.end(), &roomB), rooms.end());
}

TEST(SemanticFixtures, MakeKeyFrameAtRegistersWithMapAndCameraCenter)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    KeyFrame keyFrame;
    makeKeyFrameAt(keyFrame, 42U, p_map, Eigen::Vector3f(1.0F, 2.0F, 3.0F));

    EXPECT_EQ(keyFrame.id, 42U);
    Eigen::Vector3f cameraCenter{};
    ASSERT_EQ((keyFrame.getCameraCenter(cameraCenter)),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    EXPECT_TRUE(cameraCenter.isApprox(Eigen::Vector3f(1.0F, 2.0F, 3.0F)));
    std::vector<KeyFrame *> allKeyFrames{};
    ASSERT_EQ((p_map->getAllKeyFrames(allKeyFrames)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_NE(std::find(allKeyFrames.begin(), allKeyFrames.end(), &keyFrame),
              allKeyFrames.end());
}

TEST(SemanticFixtures, MakeKeyFrameAtSkipsRegistrationWhenMapIsNull)
{
    KeyFrame keyFrame;
    makeKeyFrameAt(keyFrame, 1U, nullptr, Eigen::Vector3f::Zero());
    EXPECT_EQ(keyFrame.id, 1U);
}

TEST(SemanticFixtures, NonTrivialSim3IsInvertibleAndNonIdentity)
{
    const g2o::Sim3 transform = makeNonTrivialSim3();

    EXPECT_NE(transform.scale(), 1.0);
    EXPECT_FALSE(transform.rotation().isApprox(Eigen::Quaterniond::Identity()));

    const Eigen::Vector3d probe(0.3, -1.2, 2.5);
    const Eigen::Vector3d forwardThenBack =
        transform.inverse().map(transform.map(probe));
    EXPECT_TRUE(forwardThenBack.isApprox(probe, 1e-9));
}

TEST(SemanticFixtures, NonTrivialSim3IsDeterministic)
{
    const g2o::Sim3 first  = makeNonTrivialSim3();
    const g2o::Sim3 second = makeNonTrivialSim3();

    const Eigen::Vector3d probe(1.0, 1.0, 1.0);
    EXPECT_TRUE(first.map(probe).isApprox(second.map(probe)));
}

} // namespace
} // namespace test
} // namespace core
} // namespace vs_graphs
