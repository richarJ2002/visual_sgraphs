/**
 * @file test_SemanticFixtures.cpp
 * @brief Self-test for the deterministic semantic-fixture builders
 *        (semantic-axiom-reliability-plan.md, P0.4). Each case exercises one
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

    auto cloudA = makeGridCloud(centroid, axisU, axisV, 0.5, 0.75, 6);
    auto cloudB = makeGridCloud(centroid, axisU, axisV, 0.5, 0.75, 6);

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
    Map  *p_map = atlas.GetCurrentMap();

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

    EXPECT_EQ(wall.getId(), 7);
    EXPECT_EQ(wall.getPlaneType(), geometric::Plane::planeVariant::WALL);
    EXPECT_TRUE(wall.getCentroid().isApprox(Eigen::Vector3d(2.0, 0.0, 1.0)));
    EXPECT_GT(wall.getMapClouds()->size(), 0U);
    /* The equation's normal survives normalization; only its sign is
     * arbitrary until oriented toward a room. */
    EXPECT_NEAR(std::abs(wall.getGlobalEquation().coeffs().head<3>().dot(
                    Eigen::Vector3d::UnitX())),
                1.0,
                1e-6);
}

TEST(SemanticFixtures, MakeGroundPlaneRefitsSuccessfully)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    geometric::Plane      ground;
    const bool refitOk = makeGroundPlane(ground, 1, p_map);

    EXPECT_TRUE(refitOk);
    EXPECT_EQ(ground.getPlaneType(), geometric::Plane::planeVariant::GROUND);
}

TEST(SemanticFixtures, MakeRoomAttachesWallAndCentroid)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

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

    EXPECT_EQ(room.getId(), 5);
    EXPECT_EQ(room.getRoomVariant(), semantic::Room::roomVariant::ROOM);
    EXPECT_TRUE(room.getCentroid().isApprox(Eigen::Vector3d(3.0, 4.0, 0.0)));
    const auto walls = room.getWalls();
    ASSERT_EQ(walls.size(), 1U);
    EXPECT_EQ(walls.front(), &wall);
}

TEST(SemanticFixtures, MakePassageWiresKnownAndFarSide)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

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

    EXPECT_EQ(passage.getId(), 9);
    /* makePassage never claims traversal evidence on the caller's behalf. */
    EXPECT_FALSE(passage.getTraversalEvidence());
    EXPECT_EQ(passage.getKnownSideProvenance().pRoom, &knownRoom);
    EXPECT_TRUE(passage.getKnownSideProvenance().direction_World.isApprox(
        Eigen::Vector3d(-1.0, 0.0, 0.0)));
    EXPECT_EQ(passage.getProspectiveRoom(), &farRoom);
}

TEST(SemanticFixtures, MakeFloorOwnsGivenRooms)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    semantic::Room roomA;
    makeRoom(roomA, 1, p_map, nullptr);
    semantic::Room roomB;
    makeRoom(roomB, 2, p_map, nullptr);

    semantic::Floor floor;
    makeFloor(floor, 3, p_map, {&roomA, &roomB}, 0.0);

    const auto rooms = floor.getRooms();
    ASSERT_EQ(rooms.size(), 2U);
    EXPECT_NE(std::find(rooms.begin(), rooms.end(), &roomA), rooms.end());
    EXPECT_NE(std::find(rooms.begin(), rooms.end(), &roomB), rooms.end());
}

TEST(SemanticFixtures, MakeKeyFrameAtRegistersWithMapAndCameraCenter)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    KeyFrame keyFrame;
    makeKeyFrameAt(keyFrame, 42U, p_map, Eigen::Vector3f(1.0F, 2.0F, 3.0F));

    EXPECT_EQ(keyFrame.mnId, 42U);
    EXPECT_TRUE(
        keyFrame.GetCameraCenter().isApprox(Eigen::Vector3f(1.0F, 2.0F, 3.0F)));
    const auto allKeyFrames = p_map->GetAllKeyFrames();
    EXPECT_NE(std::find(allKeyFrames.begin(), allKeyFrames.end(), &keyFrame),
              allKeyFrames.end());
}

TEST(SemanticFixtures, MakeKeyFrameAtSkipsRegistrationWhenMapIsNull)
{
    KeyFrame keyFrame;
    makeKeyFrameAt(keyFrame, 1U, nullptr, Eigen::Vector3f::Zero());
    EXPECT_EQ(keyFrame.mnId, 1U);
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
