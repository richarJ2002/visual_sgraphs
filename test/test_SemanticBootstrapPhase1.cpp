/*!
 * @file test_SemanticBootstrapPhase1.cpp
 * @brief Focused Phase-1 semantic bootstrap and wall-lifecycle tests.
 */

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "Geometric/Plane.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Semantic/Floor.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "SemanticsManager.h"

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace
{
std::unique_ptr<geometric::Plane> makeAdmissibleWall(const int    id_in,
                                                     Map         *p_map_in,
                                                     const double x_m_in = 0.0)
{
    std::unique_ptr<geometric::Plane> p_wall =
        std::make_unique<geometric::Plane>();
    if (p_wall->setId(id_in) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_wall->setMap(p_map_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_wall->setPlaneType(geometric::Plane::PlaneVariant::WALL) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPlaneType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_wall->castWeightedVote(geometric::Plane::PlaneVariant::WALL, 1.0) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: castWeightedVote returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_wall->setGlobalEquation(
            g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, -x_m_in))) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_wall->setCentroid(Eigen::Vector3d(x_m_in, 0.0, 1.0)) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    for (int widthIndex = 0; widthIndex < 20; ++widthIndex)
    {
        for (int heightIndex = 0; heightIndex < 20; ++heightIndex)
        {
            pcl::PointXYZRGBA point;
            point.x = static_cast<float>(x_m_in);
            point.y = -1.0F + 2.0F * static_cast<float>(widthIndex) / 19.0F;
            point.z = 2.0F * static_cast<float>(heightIndex) / 19.0F;
            p_cloud->push_back(point);
        }
    }
    if (p_wall->setMapClouds(p_cloud) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMapClouds returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_wall->updateSizeOfPlane() !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateSizeOfPlane returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    return p_wall;
}

semantic::Room *bootstrap(SemanticsManager &manager_inout, Atlas &atlas_inout)
{
    EXPECT_GE(manager_inout.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d(0.0, 0.0, 1.0)),
              0);
    std::vector<semantic::Room *> rooms{};
    if (atlas_inout.getCurrentMap()->getAllRooms(rooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    EXPECT_EQ(rooms.size(), 1U);
    return rooms.empty() ? nullptr : rooms.front();
}
} // namespace

TEST(SemanticBootstrapPhase1, BootstrapInitializationIsIdempotent)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_firstRoom = bootstrap(manager, atlas);
    ASSERT_NE(p_firstRoom, nullptr);

    manager.ensureActiveMapBootstrapHierarchyForTest(
        Eigen::Vector3d(9.0, 9.0, 9.0));

    std::vector<semantic::Room *> allRooms{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllRooms(allRooms)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ(allRooms.size(), 1U);
    std::vector<semantic::Room *> allRooms2{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllRooms(allRooms2)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(allRooms2.front(), p_firstRoom);
    std::vector<semantic::Floor *> allFloors{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllFloors(allFloors)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(allFloors.size(), 1U);
    int id{};
    ASSERT_EQ((p_firstRoom->getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(id, 0);
    std::string name{};
    ASSERT_EQ((p_firstRoom->getName(name)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(name, "semantic::Room#0");
    int                            id2{};
    std::vector<semantic::Floor *> allFloors2{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllFloors(allFloors2)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((allFloors2.front()->getId(id2)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    EXPECT_EQ(id2, 0);
    std::string                    name2{};
    std::vector<semantic::Floor *> allFloors3{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllFloors(allFloors3)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((allFloors3.front()->getName(name2)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    EXPECT_EQ(name2, "semantic::Floor#0");
}

TEST(SemanticBootstrapPhase1,
     NewMapRestoresLastRoomFloorAndPassageWithoutHistoricalWalls)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    Map             *p_departedMap  = atlas.getCurrentMap();
    semantic::Room  *p_departedRoom = bootstrap(manager, atlas);
    ASSERT_NE(p_departedRoom, nullptr);

    std::unique_ptr<geometric::Plane> p_historicalWall =
        makeAdmissibleWall(4, p_departedMap);
    atlas.addMapPlane(p_historicalWall.get());
    ASSERT_EQ((p_departedRoom->setWalls(p_historicalWall.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    semantic::Passage departedPassage;
    ASSERT_EQ((departedPassage.setId(atlas.reservePassageIdentity())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.setMap(p_departedMap)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.setKnownSideRoom(p_departedRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.addTraversalObservation(
                  semantic::Passage::TraversalDirection::KNOWN_TO_FAR)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    atlas.addMapPassage(&departedPassage);
    ASSERT_EQ((p_departedRoom->setDoorways(&departedPassage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    atlas.createNewMap();
    Map *p_recoveryMap = atlas.getCurrentMap();
    ASSERT_NE(p_recoveryMap, p_departedMap);

    const Eigen::Vector3d recoveredCameraPosition_World_m(8.0, 2.0, 1.0);
    EXPECT_GE(manager.ensureActiveMapBootstrapHierarchyForTest(
                  recoveredCameraPosition_World_m),
              0);

    std::vector<semantic::Room *> recoveredRooms{};
    ASSERT_EQ((p_recoveryMap->getAllRooms(recoveredRooms)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ(recoveredRooms.size(), 1U);
    semantic::Room *p_recoveredRoom = recoveredRooms.front();
    int             id{};
    ASSERT_EQ((p_recoveredRoom->getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(id, 0);
    std::string name{};
    ASSERT_EQ((p_recoveredRoom->getName(name)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(name, "semantic::Room#0");
    bool isRecoveryProxy2{};
    ASSERT_EQ((p_recoveredRoom->isRecoveryProxy(isRecoveryProxy2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(isRecoveryProxy2);
    Eigen::Vector3d centroid{};
    ASSERT_EQ((p_recoveredRoom->getCentroid(centroid)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(centroid.isApprox(recoveredCameraPosition_World_m));
    std::vector<vs_graphs::core::geometric::Plane *> walls{};
    ASSERT_EQ((p_recoveredRoom->getWalls(walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(walls.empty());
    EXPECT_EQ(manager.getCurrentRoomId(), 0);

    vs_graphs::core::semantic::Floor *p_floor = nullptr;
    ASSERT_EQ((p_recoveredRoom->getFloor(p_floor)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_NE(p_floor, nullptr);
    vs_graphs::core::semantic::Floor *p_floor2 = nullptr;
    ASSERT_EQ((p_recoveredRoom->getFloor(p_floor2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    int id2{};
    ASSERT_EQ((p_floor2->getId(id2)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    EXPECT_EQ(id2, 0);
    vs_graphs::core::semantic::Floor *p_floor3 = nullptr;
    ASSERT_EQ((p_recoveredRoom->getFloor(p_floor3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    std::vector<vs_graphs::core::semantic::Room *> rooms{};
    ASSERT_EQ((p_floor3->getRooms(rooms)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ(rooms.size(), 1U);
    vs_graphs::core::semantic::Floor *p_floor4 = nullptr;
    ASSERT_EQ((p_recoveredRoom->getFloor(p_floor4)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    std::vector<vs_graphs::core::semantic::Room *> rooms2{};
    ASSERT_EQ((p_floor4->getRooms(rooms2)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    EXPECT_EQ(rooms2.front(), p_recoveredRoom);

    std::vector<semantic::Passage *> recoveredPassages{};
    ASSERT_EQ((p_recoveryMap->getAllPassages(recoveredPassages)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ(recoveredPassages.size(), 1U);
    int id3{};
    ASSERT_EQ((recoveredPassages.front()->getId(id3)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    int id4{};
    ASSERT_EQ((departedPassage.getId(id4)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(id3, id4);
    bool isRecoveryProxy3{};
    ASSERT_EQ((recoveredPassages.front()->isRecoveryProxy(isRecoveryProxy3)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(isRecoveryProxy3);
    vs_graphs::core::semantic::Passage::KnownSideProvenance
        knownSideProvenance{};
    ASSERT_EQ((recoveredPassages.front()->getKnownSideProvenance(
                  knownSideProvenance)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(knownSideProvenance.p_room, p_recoveredRoom);
    std::vector<vs_graphs::core::semantic::Passage *> passages{};
    ASSERT_EQ((p_recoveredRoom->getPassages(passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(passages.size(), 1U);
    std::vector<vs_graphs::core::semantic::Passage *> passages2{};
    ASSERT_EQ((p_recoveredRoom->getPassages(passages2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(passages2.front(), recoveredPassages.front());
}

TEST(SemanticBootstrapPhase1,
     RepeatedNoPoseMapRecoveryPreservesCurrentRoomFloorIdentity)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);

    semantic::Room roomOne;
    ASSERT_EQ((roomOne.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomOne.setMap(atlas.getCurrentMap())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomOne.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomOne.setName("semantic::Room#1")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomOne.setCentroid(Eigen::Vector3d(4.0, 0.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    atlas.addDetectedMapRoom(&roomOne);
    int roomOneId{};
    ASSERT_EQ((roomOne.getId(roomOneId)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    manager.setCurrentRoomIdForTest(roomOneId);
    manager.ensureActiveMapBootstrapHierarchyForTest(
        Eigen::Vector3d(4.0, 0.0, 1.0));

    vs_graphs::core::semantic::Floor *p_floor = nullptr;
    ASSERT_EQ((roomOne.getFloor(p_floor)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_NE(p_floor, nullptr);
    vs_graphs::core::semantic::Floor *p_floor2 = nullptr;
    ASSERT_EQ((roomOne.getFloor(p_floor2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    int id{};
    ASSERT_EQ((p_floor2->getId(id)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ(id, 0);
    vs_graphs::core::semantic::Floor *p_floor3 = nullptr;
    ASSERT_EQ((roomOne.getFloor(p_floor3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_TRUE((p_floor3->setPlaneIdentity(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0),
                                            100U,
                                            4U) ==
                 vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));

    semantic::Floor *p_previousMapFloor = nullptr;
    ASSERT_EQ((roomOne.getFloor(p_previousMapFloor)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    for (int resetIndex = 0; resetIndex < 2; ++resetIndex)
    {
        atlas.createNewMap();
        Map *p_recoveryMap = atlas.getCurrentMap();
        ASSERT_NE(p_recoveryMap, nullptr);

        /* Match the live ordering: bootstrap has no pose, then floor refresh
         * runs before the first usable camera pose arrives. */
        manager.ensureActiveMapBootstrapHierarchyForTest(
            Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN()));
        manager.getUpdatedFloorsForTest();

        std::vector<semantic::Floor *> earlyFloors{};
        ASSERT_EQ((p_recoveryMap->getAllFloors(earlyFloors)),
                  MapStatus::MAP_STATUS_SUCCESS);
        ASSERT_EQ(earlyFloors.size(), 1U);
        int id2{};
        ASSERT_EQ((earlyFloors.front()->getId(id2)),
                  vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
        EXPECT_EQ(id2, 0);
        EXPECT_NE(earlyFloors.front(), p_previousMapFloor);
        bool hasPlaneIdentity2{};
        ASSERT_EQ((earlyFloors.front()->hasPlaneIdentity(hasPlaneIdentity2)),
                  vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
        EXPECT_FALSE(hasPlaneIdentity2);

        const Eigen::Vector3d recoveredCameraPosition_World_m(
            8.0 + static_cast<double>(resetIndex),
            2.0,
            1.0);
        EXPECT_GE(manager.ensureActiveMapBootstrapHierarchyForTest(
                      recoveredCameraPosition_World_m),
                  0);

        std::vector<semantic::Room *> recoveredRooms{};
        ASSERT_EQ((p_recoveryMap->getAllRooms(recoveredRooms)),
                  MapStatus::MAP_STATUS_SUCCESS);
        ASSERT_EQ(recoveredRooms.size(), 1U);
        semantic::Room *p_recoveredRoom = recoveredRooms.front();
        int             id3{};
        ASSERT_EQ((p_recoveredRoom->getId(id3)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ(id3, 1);
        vs_graphs::core::semantic::Floor *p_floor4 = nullptr;
        ASSERT_EQ((p_recoveredRoom->getFloor(p_floor4)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ASSERT_NE(p_floor4, nullptr);
        vs_graphs::core::semantic::Floor *p_floor5 = nullptr;
        ASSERT_EQ((p_recoveredRoom->getFloor(p_floor5)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        int id4{};
        ASSERT_EQ((p_floor5->getId(id4)),
                  vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
        EXPECT_EQ(id4, 0);
        vs_graphs::core::semantic::Floor *p_floor6 = nullptr;
        ASSERT_EQ((p_recoveredRoom->getFloor(p_floor6)),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        EXPECT_EQ(p_floor6, earlyFloors.front());
        p_previousMapFloor = earlyFloors.front();
    }
}

TEST(SemanticBootstrapPhase1,
     SameMapResetRecreatesSameRoomFloorWithoutCameraPose)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);
    int id{};
    ASSERT_EQ((p_roomZero->getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(id, 0);

    /* Same-map reset (Tracking::ResetActiveMap): Map::clear() wipes the live
     * sets. Atlas::clearMap() must have exported the hierarchy first. */
    atlas.clearMap();
    std::vector<semantic::Room *> allRooms{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllRooms(allRooms)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_TRUE(allRooms.empty());
    std::vector<semantic::Floor *> allFloors{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllFloors(allFloors)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_TRUE(allFloors.empty());

    /* No keyframes exist after the clear, so no usable camera pose. Recovery
     * must still recreate semantic::Room#0/semantic::Floor#0 from the snapshot
     * centroid. */
    EXPECT_EQ(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d::Constant(
                      std::numeric_limits<double>::quiet_NaN())),
              1);

    std::vector<semantic::Room *> recoveredRooms{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllRooms(recoveredRooms)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ(recoveredRooms.size(), 1U);
    int id2{};
    ASSERT_EQ((recoveredRooms.front()->getId(id2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(id2, 0);
    bool isRecoveryProxy2{};
    ASSERT_EQ((recoveredRooms.front()->isRecoveryProxy(isRecoveryProxy2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(isRecoveryProxy2);
    vs_graphs::core::semantic::Floor *p_floor = nullptr;
    ASSERT_EQ((recoveredRooms.front()->getFloor(p_floor)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_NE(p_floor, nullptr);
    vs_graphs::core::semantic::Floor *p_floor2 = nullptr;
    ASSERT_EQ((recoveredRooms.front()->getFloor(p_floor2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    int id3{};
    ASSERT_EQ((p_floor2->getId(id3)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    EXPECT_EQ(id3, 0);
    EXPECT_EQ(manager.getCurrentRoomId(), 0);
}

TEST(SemanticBootstrapPhase1, BootstrapIgnoresSpuriousRoomWhenRecoveryPending)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);

    atlas.clearMap();

    /* Spurious free-space SE# classified as ROOM during the reset transient
     * (live failure 20260910-222334 cycles 135-138: SE#4 hijacked current). */
    semantic::Room spuriousRoom;
    ASSERT_EQ((spuriousRoom.setId(99)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((spuriousRoom.setMap(atlas.getCurrentMap())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((spuriousRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((spuriousRoom.setName("semantic::Room#99")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((spuriousRoom.setCentroid(Eigen::Vector3d(5.0, 5.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    atlas.addDetectedMapRoom(&spuriousRoom);

    EXPECT_EQ(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d(1.0, 0.0, 1.0)),
              1);

    /* Recovery identity wins: current returns to semantic::Room#0, not the
     * spurious lowest-ID fallback. Duplicate fusion owns the spurious room
     * later. */
    EXPECT_EQ(manager.getCurrentRoomId(), 0);
    EXPECT_EQ(atlas.getCurrentSemanticRoomIdentity(), 0);
}

TEST(SemanticBootstrapPhase1,
     RoomIdentityOrderingContinuesAfterRecoveryMapBootstrap)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);

    semantic::Passage passage;
    ASSERT_EQ((passage.setId(atlas.reservePassageIdentity())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(atlas.getCurrentMap())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setKnownSideRoom(p_roomZero)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    atlas.addMapPassage(&passage);
    ASSERT_EQ((p_roomZero->setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    atlas.createNewMap();
    manager.ensureActiveMapBootstrapHierarchyForTest(
        Eigen::Vector3d(1.0, 0.0, 1.0));

    vs_graphs::core::semantic::Room *p_blankRoomCandidate = nullptr;
    ASSERT_EQ((GeoSemHelpers::createBlankRoomCandidate(
                  &atlas,
                  p_blankRoomCandidate,
                  Eigen::Vector3d(2.0, 0.0, 1.0))),
              GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> p_roomOne(p_blankRoomCandidate);
    ASSERT_NE(p_roomOne, nullptr);
    int id{};
    ASSERT_EQ((p_roomOne->getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(id, 1);
}

TEST(SemanticBootstrapPhase1,
     SameIdentityObservedPassageReplacesRecoveryProxyGeometry)
{
    semantic::Passage canonicalPassage;
    ASSERT_EQ((canonicalPassage.setId(4)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((canonicalPassage.setRecoveryProxy(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((canonicalPassage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);

    semantic::Passage observedPassage;
    ASSERT_EQ((observedPassage.setId(4)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((observedPassage.setRecoveryProxy(false)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((observedPassage.setPassable(false)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((observedPassage.setPassageType(
                  semantic::Passage::PassageVariant::DOORWAY)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((observedPassage.setCentroid(Eigen::Vector3d(3.0, 2.0, 1.0))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((observedPassage.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, -3.0)))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((observedPassage.setWidth(1.2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((observedPassage.setHeight(2.1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_TRUE(
        (observedPassage.setKnownSideDirection(Eigen::Vector3d::UnitX()) ==
         vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS));

    bool wasGeometryReplaced{};
    ASSERT_EQ((canonicalPassage.mergeFromDuplicate(&observedPassage,
                                                   wasGeometryReplaced)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(wasGeometryReplaced);
    bool isRecoveryProxy2{};
    ASSERT_EQ((canonicalPassage.isRecoveryProxy(isRecoveryProxy2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_FALSE(isRecoveryProxy2);
    bool isPassable2{};
    ASSERT_EQ((canonicalPassage.isPassable(isPassable2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_FALSE(isPassable2);
    vs_graphs::core::semantic::Passage::PassageVariant passageType{};
    ASSERT_EQ((canonicalPassage.getPassageType(passageType)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(passageType, semantic::Passage::PassageVariant::DOORWAY);
    Eigen::Vector3d centroid{};
    ASSERT_EQ((canonicalPassage.getCentroid(centroid)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(centroid.isApprox(Eigen::Vector3d(3.0, 2.0, 1.0)));
    g2o::Plane3D globalEquation{};
    ASSERT_EQ((canonicalPassage.getGlobalEquation(globalEquation)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(
        globalEquation.coeffs().isApprox(Eigen::Vector4d(1.0, 0.0, 0.0, -3.0)));
    double width{};
    ASSERT_EQ((canonicalPassage.getWidth(width)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(width, 1.2);
    double height{};
    ASSERT_EQ((canonicalPassage.getHeight(height)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(height, 2.1);
    vs_graphs::core::semantic::Passage::KnownSideProvenance
        knownSideProvenance{};
    ASSERT_EQ((canonicalPassage.getKnownSideProvenance(knownSideProvenance)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(
        knownSideProvenance.direction_World.isApprox(Eigen::Vector3d::UnitX()));
}

TEST(SemanticBootstrapPhase1,
     SameIdentityPassageMergePreservesUniqueTopologyAndMaximumCounters)
{
    semantic::Passage canonicalPassage;
    ASSERT_EQ((canonicalPassage.setId(5)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((canonicalPassage.setRecoveryProxy(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((canonicalPassage.addTraversalObservation(
                  semantic::Passage::TraversalDirection::KNOWN_TO_FAR)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);

    semantic::Passage duplicatePassage;
    ASSERT_EQ((duplicatePassage.setId(5)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((duplicatePassage.addTraversalObservation(
                  semantic::Passage::TraversalDirection::KNOWN_TO_FAR)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((duplicatePassage.addTraversalObservation(
                  semantic::Passage::TraversalDirection::KNOWN_TO_FAR)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((duplicatePassage.addTraversalObservation(
                  semantic::Passage::TraversalDirection::FAR_TO_KNOWN)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_TRUE(
        (duplicatePassage.setKnownSideDirection(Eigen::Vector3d::UnitY()) ==
         vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS));

    geometric::Plane supportingWall;
    ASSERT_EQ((duplicatePassage.addAssociateWall(&supportingWall)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((duplicatePassage.addAssociateWall(&supportingWall)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);

    bool wasGeometryReplaced{};
    ASSERT_EQ((canonicalPassage.mergeFromDuplicate(&duplicatePassage,
                                                   wasGeometryReplaced)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_FALSE(wasGeometryReplaced);
    std::size_t traversalKnownToFarCount{};
    ASSERT_EQ((canonicalPassage.getTraversalKnownToFarCount(
                  traversalKnownToFarCount)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalKnownToFarCount, 2U);
    std::size_t traversalFarToKnownCount{};
    ASSERT_EQ((canonicalPassage.getTraversalFarToKnownCount(
                  traversalFarToKnownCount)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalFarToKnownCount, 1U);
    std::size_t traversalUnknownCount{};
    ASSERT_EQ(
        (canonicalPassage.getTraversalUnknownCount(traversalUnknownCount)),
        vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalUnknownCount, 0U);
    vs_graphs::core::semantic::Passage::KnownSideProvenance
        knownSideProvenance{};
    ASSERT_EQ((canonicalPassage.getKnownSideProvenance(knownSideProvenance)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(
        knownSideProvenance.direction_World.isApprox(Eigen::Vector3d::UnitY()));
    std::vector<vs_graphs::core::geometric::Plane *> associateWalls{};
    ASSERT_EQ((canonicalPassage.getAssociateWalls(associateWalls)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ(associateWalls.size(), 1U);
    std::vector<vs_graphs::core::geometric::Plane *> associateWalls2{};
    ASSERT_EQ((canonicalPassage.getAssociateWalls(associateWalls2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(associateWalls2.front(), &supportingWall);
    bool isRecoveryProxy2{};
    ASSERT_EQ((canonicalPassage.isRecoveryProxy(isRecoveryProxy2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(isRecoveryProxy2);
}

TEST(SemanticBootstrapPhase1, RoomAndFloorOwnershipIsReciprocal)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);
    std::vector<semantic::Floor *> allFloors{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllFloors(allFloors)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ(allFloors.size(), 1U);
    std::vector<semantic::Floor *> allFloors2{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllFloors(allFloors2)),
              MapStatus::MAP_STATUS_SUCCESS);
    semantic::Floor                  *p_floor  = allFloors2.front();
    vs_graphs::core::semantic::Floor *p_floor2 = nullptr;
    ASSERT_EQ((p_room->getFloor(p_floor2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(p_floor2, p_floor);
    std::vector<vs_graphs::core::semantic::Room *> rooms{};
    ASSERT_EQ((p_floor->getRooms(rooms)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_EQ(rooms.size(), 1U);
    std::vector<vs_graphs::core::semantic::Room *> rooms2{};
    ASSERT_EQ((p_floor->getRooms(rooms2)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    EXPECT_EQ(rooms2.front(), p_room);
}

TEST(SemanticBootstrapPhase1, BootstrapSeedsCurrentRoom)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);
    int id{};
    ASSERT_EQ((p_room->getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(manager.getCurrentRoomId(), id);
}

TEST(SemanticBootstrapPhase1, IdempotentBootstrapPreservesTraversedCurrentRoom)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);

    semantic::Room roomOne;
    ASSERT_EQ((roomOne.setId(1)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomOne.setMap(atlas.getCurrentMap())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomOne.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomOne.setName("semantic::Room#1")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomOne.setCentroid(Eigen::Vector3d(4.0, 0.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    atlas.addDetectedMapRoom(&roomOne);

    int roomOneId{};
    ASSERT_EQ((roomOne.getId(roomOneId)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    manager.setCurrentRoomIdForTest(roomOneId);
    manager.ensureActiveMapBootstrapHierarchyForTest(
        Eigen::Vector3d(4.0, 0.0, 1.0));

    int id{};
    ASSERT_EQ((roomOne.getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(manager.getCurrentRoomId(), id);
    int id2{};
    ASSERT_EQ((roomOne.getId(id2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(atlas.getCurrentSemanticRoomIdentity(), id2);
    vs_graphs::core::semantic::Floor *p_floor = nullptr;
    ASSERT_EQ((roomOne.getFloor(p_floor)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_NE(p_floor, nullptr);
    vs_graphs::core::semantic::Floor *p_floor2 = nullptr;
    ASSERT_EQ((roomOne.getFloor(p_floor2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    int id3{};
    ASSERT_EQ((p_floor2->getId(id3)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    EXPECT_EQ(id3, 0);

    std::unique_ptr<geometric::Plane> p_wall =
        makeAdmissibleWall(7, atlas.getCurrentMap(), 5.0);
    atlas.addMapPlane(p_wall.get());
    manager.associateAllWallsToRoomsForTest();
    std::vector<vs_graphs::core::geometric::Plane *> walls{};
    ASSERT_EQ((p_roomZero->getWalls(walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(walls.empty());
    std::vector<vs_graphs::core::geometric::Plane *> walls2{};
    ASSERT_EQ((roomOne.getWalls(walls2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(walls2.size(), 1U);
    std::vector<vs_graphs::core::geometric::Plane *> walls3{};
    ASSERT_EQ((roomOne.getWalls(walls3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(walls3.front(), p_wall.get());
}

TEST(SemanticBootstrapPhase1, OrdinaryAdmissibleWallBelongsToCurrentRoom)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);
    semantic::Room  *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);
    std::unique_ptr<geometric::Plane> p_wall = makeAdmissibleWall(1, p_map);
    atlas.addMapPlane(p_wall.get());

    manager.associateAllWallsToRoomsForTest();

    std::vector<vs_graphs::core::geometric::Plane *> walls{};
    ASSERT_EQ((p_room->getWalls(walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(walls.size(), 1U);
    std::vector<vs_graphs::core::geometric::Plane *> walls2{};
    ASSERT_EQ((p_room->getWalls(walls2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(walls2.front(), p_wall.get());
    EXPECT_EQ(atlas.getRoomWallPlaneById(1), p_wall.get());
}

TEST(SemanticBootstrapPhase1, PassageFarSideRoutingPrecedesCurrentRoomFallback)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);
    semantic::Room  *p_nearRoom = bootstrap(manager, atlas);
    ASSERT_NE(p_nearRoom, nullptr);
    ASSERT_EQ((p_nearRoom->setCentroid(Eigen::Vector3d(-1.0, 0.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    semantic::Room farRoom;
    ASSERT_EQ((farRoom.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((farRoom.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((farRoom.setCentroid(Eigen::Vector3d(1.0, 0.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((farRoom.setRoomVariant(semantic::Room::RoomVariant::UNDEFINED)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    atlas.addCandidateMapRoom(&farRoom);

    semantic::Passage passage;
    ASSERT_EQ((passage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setWidth(4.0)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setHeight(4.0)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setProspectiveRoom(&farRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    atlas.addMapPassage(&passage);

    std::unique_ptr<geometric::Plane> p_wall =
        makeAdmissibleWall(3, p_map, 2.0);
    atlas.addMapPlane(p_wall.get());
    manager.associateAllWallsToRoomsForTest();

    std::vector<vs_graphs::core::geometric::Plane *> walls{};
    ASSERT_EQ((p_nearRoom->getWalls(walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(walls.empty());
    std::vector<vs_graphs::core::geometric::Plane *> walls2{};
    ASSERT_EQ((farRoom.getWalls(walls2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(walls2.size(), 1U);
    std::vector<vs_graphs::core::geometric::Plane *> walls3{};
    ASSERT_EQ((farRoom.getWalls(walls3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(walls3.front(), p_wall.get());
}

TEST(SemanticBootstrapPhase1,
     ExactSupportingWallKeepsPassageLinkedWhenRoomCentroidIsCoplanar)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);
    semantic::Room  *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);

    std::unique_ptr<geometric::Plane> p_wall =
        makeAdmissibleWall(11, p_map, 0.0);
    atlas.addMapPlane(p_wall.get());
    ASSERT_EQ((p_room->setWalls(p_wall.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    semantic::Passage passage;
    ASSERT_EQ((passage.setId(atlas.reservePassageIdentity())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setWidth(1.2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setHeight(2.0)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 1.0))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.addAssociateWall(p_wall.get())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setKnownSideRoom(p_room)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_TRUE(
        (passage.setKnownSideDirection(Eigen::Vector3d::UnitX()) ==
         vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS));
    atlas.addMapPassage(&passage);

    for (int cycle = 0; cycle < 7; ++cycle)
    {
        manager.associatePassagesToRoomsForTest();
    }

    bool isBad2{};
    ASSERT_EQ((passage.isBad(isBad2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_FALSE(isBad2);
    std::vector<vs_graphs::core::semantic::Passage *> passages{};
    ASSERT_EQ((p_room->getPassages(passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(passages.size(), 1U);
    std::vector<vs_graphs::core::semantic::Passage *> passages2{};
    ASSERT_EQ((p_room->getPassages(passages2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(passages2.front(), &passage);
}

TEST(SemanticBootstrapPhase1,
     SparseRoomWithoutExactWallCannotWinPassageByProximity)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);
    semantic::Room  *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);

    /* The room owns exactly one wall at x=0 -- deliberately NOT the
     * passage's supporting wall -- and sits clearly on the negative side
     * of the passage plane, so it would win that side by centroid distance
     * alone without the proximity-association guard. */
    std::unique_ptr<geometric::Plane> p_roomWall =
        makeAdmissibleWall(11, p_map, 0.0);
    atlas.addMapPlane(p_roomWall.get());
    ASSERT_EQ((p_room->setWalls(p_roomWall.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_room->setCentroid(Eigen::Vector3d(-2.0, 0.0, 1.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    std::vector<vs_graphs::core::geometric::Plane *> walls{};
    ASSERT_EQ((p_room->getWalls(walls)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(walls.size(), 1U);

    /* A different but nearby parallel wall (0.5 m away, inside the
     * supporting-plane gate) supports the passage. */
    std::unique_ptr<geometric::Plane> p_supportWall =
        makeAdmissibleWall(12, p_map, 0.5);
    atlas.addMapPlane(p_supportWall.get());

    semantic::Passage passage;
    ASSERT_EQ((passage.setId(atlas.reservePassageIdentity())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setWidth(1.2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setHeight(2.0)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 1.0))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.addAssociateWall(p_supportWall.get())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    atlas.addMapPassage(&passage);

    for (int cycle = 0; cycle < 3; ++cycle)
    {
        manager.associatePassagesToRoomsForTest();
    }

    std::vector<vs_graphs::core::semantic::Passage *> passages{};
    ASSERT_EQ((p_room->getPassages(passages)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(passages.empty());
}

TEST(SemanticBootstrapPhase1, PendingWallHasFiveCycleGraceAndGrowthReset)
{
    Atlas                             atlas(0);
    Map                              *p_map = atlas.getCurrentMap();
    SemanticsManager                  manager(&atlas);
    std::unique_ptr<geometric::Plane> p_wall = makeAdmissibleWall(1, p_map);
    atlas.addMapPlane(p_wall.get());
    manager.associateAllWallsToRoomsForTest();

    manager.suppressUndefendedWallsForTest();
    manager.suppressUndefendedWallsForTest();
    manager.suppressUndefendedWallsForTest();
    EXPECT_EQ(manager.getPendingWallAgeForTest(1), 2);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud{};
    ASSERT_EQ((p_wall->getMapClouds(p_cloud)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    p_cloud->push_back(p_cloud->back());
    ASSERT_EQ((p_wall->setMapClouds(p_cloud)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    manager.suppressUndefendedWallsForTest();
    EXPECT_EQ(manager.getPendingWallAgeForTest(1), 0);

    for (int cycle = 0; cycle < 4; ++cycle)
    {
        manager.suppressUndefendedWallsForTest();
    }
    bool isBad2{};
    ASSERT_EQ((p_wall->isBad(isBad2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_FALSE(isBad2);
    manager.suppressUndefendedWallsForTest();
    bool isBad3{};
    ASSERT_EQ((p_wall->isBad(isBad3)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_TRUE(isBad3);
}

TEST(SemanticBootstrapPhase1, OwnedAndPassageWallsCannotRetireUndefended)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    SemanticsManager manager(&atlas);
    semantic::Room  *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);
    std::unique_ptr<geometric::Plane> p_ownedWall =
        makeAdmissibleWall(1, p_map);
    std::unique_ptr<geometric::Plane> p_passageWall =
        makeAdmissibleWall(2, p_map, 3.0);
    atlas.addMapPlane(p_ownedWall.get());
    atlas.addMapPlane(p_passageWall.get());
    ASSERT_EQ((p_room->setWalls(p_ownedWall.get())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    semantic::Passage passage;
    ASSERT_EQ((passage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.addAssociateWall(p_passageWall.get())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    atlas.addMapPassage(&passage);

    for (int cycle = 0; cycle < 8; ++cycle)
    {
        manager.suppressUndefendedWallsForTest();
    }
    bool isBad2{};
    ASSERT_EQ((p_ownedWall->isBad(isBad2)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_FALSE(isBad2);
    bool isBad3{};
    ASSERT_EQ((p_passageWall->isBad(isBad3)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    EXPECT_FALSE(isBad3);
}

TEST(SemanticBootstrapPhase1, NewRoomIsUnvisitedByDefault)
{
    semantic::Room room;
    bool           hasPreviouslyVisited2{};
    ASSERT_EQ((room.hasPreviouslyVisited(hasPreviouslyVisited2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(hasPreviouslyVisited2);
}

TEST(SemanticBootstrapPhase1, BootstrapRoomIsVisitedAtBirth)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);
    /* The UAV starts inside the bootstrap room: presence evidences entry. */
    bool hasPreviouslyVisited2{};
    ASSERT_EQ((p_room->hasPreviouslyVisited(hasPreviouslyVisited2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(hasPreviouslyVisited2);
}

TEST(SemanticBootstrapPhase1, ResetRestoresVisitedFlag)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);
    bool hasPreviouslyVisited2{};
    ASSERT_EQ((p_roomZero->hasPreviouslyVisited(hasPreviouslyVisited2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_TRUE(hasPreviouslyVisited2);

    atlas.clearMap();

    EXPECT_EQ(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d::Constant(
                      std::numeric_limits<double>::quiet_NaN())),
              1);

    std::vector<semantic::Room *> recoveredRooms{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllRooms(recoveredRooms)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ(recoveredRooms.size(), 1U);
    int id{};
    ASSERT_EQ((recoveredRooms.front()->getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(id, 0);
    bool hasPreviouslyVisited3{};
    ASSERT_EQ(
        (recoveredRooms.front()->hasPreviouslyVisited(hasPreviouslyVisited3)),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(hasPreviouslyVisited3);
    EXPECT_EQ(manager.getCurrentRoomId(), 0);
}

TEST(SemanticBootstrapPhase1, ResetRestoresPassageIdentityWithoutGeometry)
{
    /* Restored passages are new objects with stable IDs: frame-free state
     * (passable, traversal history) carries over, but position, orientation,
     * and aperture dimensions are never copied across a map break. */
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    Map             *p_departedMap  = atlas.getCurrentMap();
    semantic::Room  *p_departedRoom = bootstrap(manager, atlas);
    ASSERT_NE(p_departedRoom, nullptr);

    semantic::Passage departedPassage;
    ASSERT_EQ((departedPassage.setId(atlas.reservePassageIdentity())),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.setMap(p_departedMap)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.setCentroid(Eigen::Vector3d(1.0, 2.0, 3.0))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.setGlobalEquation(
                  g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, -3.0)))),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.setWidth(1.25)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.setHeight(2.0)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.setKnownSideRoom(p_departedRoom)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((departedPassage.addTraversalObservation(
                  semantic::Passage::TraversalDirection::KNOWN_TO_FAR)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    atlas.addMapPassage(&departedPassage);
    ASSERT_EQ((p_departedRoom->setDoorways(&departedPassage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    atlas.createNewMap();
    Map *p_recoveryMap = atlas.getCurrentMap();
    ASSERT_NE(p_recoveryMap, p_departedMap);

    EXPECT_GE(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d(8.0, 2.0, 1.0)),
              0);

    std::vector<semantic::Passage *> recoveredPassages{};
    ASSERT_EQ((p_recoveryMap->getAllPassages(recoveredPassages)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ(recoveredPassages.size(), 1U);
    int id{};
    ASSERT_EQ((recoveredPassages.front()->getId(id)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    int id2{};
    ASSERT_EQ((departedPassage.getId(id2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(id, id2);
    bool isRecoveryProxy2{};
    ASSERT_EQ((recoveredPassages.front()->isRecoveryProxy(isRecoveryProxy2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(isRecoveryProxy2);
    bool isPassable2{};
    ASSERT_EQ((recoveredPassages.front()->isPassable(isPassable2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(isPassable2);
    Eigen::Vector3d centroid{};
    ASSERT_EQ((recoveredPassages.front()->getCentroid(centroid)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_TRUE(centroid.isApprox(Eigen::Vector3d::Zero()));
    double width{};
    ASSERT_EQ((recoveredPassages.front()->getWidth(width)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(width, 0.0);
    double height{};
    ASSERT_EQ((recoveredPassages.front()->getHeight(height)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_DOUBLE_EQ(height, 0.0);
    std::size_t traversalKnownToFarCount{};
    ASSERT_EQ((recoveredPassages.front()->getTraversalKnownToFarCount(
                  traversalKnownToFarCount)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    EXPECT_EQ(traversalKnownToFarCount, 1U);
}

TEST(SemanticBootstrapPhase1, MapChainLinksStartingFinalAndFollowingRooms)
{
    /* Mission-chain trace: each map records its entry room, its departure
     * room, and its successor. Same-map clears link nothing. */
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    Map             *p_mapZero  = atlas.getCurrentMap();
    semantic::Room  *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);
    semantic::Room *p_startingRoom = nullptr;
    ASSERT_EQ((p_mapZero->getStartingRoom(p_startingRoom)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_startingRoom, p_roomZero);
    Map *p_followingMap = nullptr;
    ASSERT_EQ((p_mapZero->getFollowingMap(p_followingMap)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_followingMap, nullptr);

    atlas.createNewMap();
    Map *p_mapOne = atlas.getCurrentMap();
    ASSERT_NE(p_mapOne, p_mapZero);
    Map *p_followingMap2 = nullptr;
    ASSERT_EQ((p_mapZero->getFollowingMap(p_followingMap2)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_followingMap2, p_mapOne);
    semantic::Room *p_finalRoom = nullptr;
    ASSERT_EQ((p_mapZero->getFinalRoom(p_finalRoom)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_finalRoom, p_roomZero);

    EXPECT_GE(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d(8.0, 2.0, 1.0)),
              0);
    std::vector<semantic::Room *> recoveredRooms{};
    ASSERT_EQ((p_mapOne->getAllRooms(recoveredRooms)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ(recoveredRooms.size(), 1U);
    semantic::Room *p_startingRoom2 = nullptr;
    ASSERT_EQ((p_mapOne->getStartingRoom(p_startingRoom2)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_startingRoom2, recoveredRooms.front());

    atlas.clearMap();
    Map *p_followingMap3 = nullptr;
    ASSERT_EQ((p_mapOne->getFollowingMap(p_followingMap3)),
              MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_followingMap3, nullptr);
}

TEST(SemanticBootstrapPhase1, ZeroPoseKeyFrameFallsBackToSnapshotCentroid)
{
    /* Live failure: a brand-new map's first keyframes carry identity poses,
     * whose exactly-zero centers passed the finiteness check and planted the
     * bootstrap room at the origin. Zero is rejected; the snapshot centroid
     * is used instead. */
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room  *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);

    semantic::Room roomTwo;
    ASSERT_EQ((roomTwo.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomTwo.setMap(atlas.getCurrentMap())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomTwo.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomTwo.setName("semantic::Room#2")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomTwo.setCentroid(Eigen::Vector3d(2.0, 3.0, 4.0))),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    atlas.addDetectedMapRoom(&roomTwo);
    manager.setCurrentRoomIdForTest(2);

    atlas.clearMap();

    /* Uninitialized first-frame pose: finite but exactly zero. */
    KeyFrame zeroPoseKeyFrame;
    ASSERT_EQ(
        (zeroPoseKeyFrame.setPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                               Eigen::Vector3f::Zero()))),
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    ASSERT_EQ((atlas.getCurrentMap()->addKeyFrame(&zeroPoseKeyFrame)),
              MapStatus::MAP_STATUS_SUCCESS);

    EXPECT_EQ(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d::Constant(
                      std::numeric_limits<double>::quiet_NaN())),
              1);

    std::vector<semantic::Room *> recoveredRooms{};
    ASSERT_EQ((atlas.getCurrentMap()->getAllRooms(recoveredRooms)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ(recoveredRooms.size(), 1U);
    int id{};
    ASSERT_EQ((recoveredRooms.front()->getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(id, 2);
    Eigen::Vector3d centroid2{};
    ASSERT_EQ((recoveredRooms.front()->getCentroid(centroid2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(centroid2.isApprox(Eigen::Vector3d(2.0, 3.0, 4.0)));
    Eigen::Vector3d centroid3{};
    ASSERT_EQ((recoveredRooms.front()->getCentroid(centroid3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(centroid3.isZero());
}
} // namespace core
} // namespace vs_graphs
