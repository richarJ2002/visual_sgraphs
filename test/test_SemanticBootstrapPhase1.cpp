/**
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
    std::unique_ptr<geometric::Plane> p_wall = std::make_unique<geometric::Plane>();
    p_wall->setId(id_in);
    p_wall->setMap(p_map_in);
    p_wall->setPlaneType(geometric::Plane::PlaneVariant::WALL);
    p_wall->castWeightedVote(geometric::Plane::PlaneVariant::WALL, 1.0);
    p_wall->setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, -x_m_in)));
    p_wall->setCentroid(Eigen::Vector3d(x_m_in, 0.0, 1.0));

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
    p_wall->setMapClouds(p_cloud);
    p_wall->updateSizeOfPlane();
    return p_wall;
}

semantic::Room *bootstrap(SemanticsManager &manager_inout, Atlas &atlas_inout)
{
    EXPECT_GE(manager_inout.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d(0.0, 0.0, 1.0)),
              0);
    const std::vector<semantic::Room *> rooms =
        atlas_inout.GetCurrentMap()->GetAllRooms();
    EXPECT_EQ(rooms.size(), 1U);
    return rooms.empty() ? nullptr : rooms.front();
}
} // namespace

TEST(SemanticBootstrapPhase1, BootstrapInitializationIsIdempotent)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_firstRoom = bootstrap(manager, atlas);
    ASSERT_NE(p_firstRoom, nullptr);

    manager.ensureActiveMapBootstrapHierarchyForTest(
        Eigen::Vector3d(9.0, 9.0, 9.0));

    ASSERT_EQ(atlas.GetCurrentMap()->GetAllRooms().size(), 1U);
    EXPECT_EQ(atlas.GetCurrentMap()->GetAllRooms().front(), p_firstRoom);
    EXPECT_EQ(atlas.GetCurrentMap()->GetAllFloors().size(), 1U);
    EXPECT_EQ(p_firstRoom->getId(), 0);
    EXPECT_EQ(p_firstRoom->getName(), "semantic::Room#0");
    EXPECT_EQ(atlas.GetCurrentMap()->GetAllFloors().front()->getId(), 0);
    EXPECT_EQ(atlas.GetCurrentMap()->GetAllFloors().front()->getName(),
              "semantic::Floor#0");
}

TEST(SemanticBootstrapPhase1,
     NewMapRestoresLastRoomFloorAndPassageWithoutHistoricalWalls)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    Map             *p_departedMap  = atlas.GetCurrentMap();
    semantic::Room            *p_departedRoom = bootstrap(manager, atlas);
    ASSERT_NE(p_departedRoom, nullptr);

    std::unique_ptr<geometric::Plane> p_historicalWall =
        makeAdmissibleWall(4, p_departedMap);
    atlas.AddMapPlane(p_historicalWall.get());
    p_departedRoom->setWalls(p_historicalWall.get());

    semantic::Passage departedPassage;
    departedPassage.setId(atlas.reservePassageIdentity());
    departedPassage.setMap(p_departedMap);
    departedPassage.setPassable(true);
    departedPassage.setKnownSideRoom(p_departedRoom);
    departedPassage.addTraversalObservation(
        semantic::Passage::TraversalDirection::KNOWN_TO_FAR);
    atlas.AddMapPassage(&departedPassage);
    p_departedRoom->setDoorways(&departedPassage);

    atlas.CreateNewMap();
    Map *p_recoveryMap = atlas.GetCurrentMap();
    ASSERT_NE(p_recoveryMap, p_departedMap);

    const Eigen::Vector3d recoveredCameraPosition_World_m(8.0, 2.0, 1.0);
    EXPECT_GE(manager.ensureActiveMapBootstrapHierarchyForTest(
                  recoveredCameraPosition_World_m),
              0);

    const std::vector<semantic::Room *> recoveredRooms = p_recoveryMap->GetAllRooms();
    ASSERT_EQ(recoveredRooms.size(), 1U);
    semantic::Room *p_recoveredRoom = recoveredRooms.front();
    EXPECT_EQ(p_recoveredRoom->getId(), 0);
    EXPECT_EQ(p_recoveredRoom->getName(), "semantic::Room#0");
    EXPECT_TRUE(p_recoveredRoom->isRecoveryProxy());
    EXPECT_TRUE(p_recoveredRoom->getCentroid().isApprox(
        recoveredCameraPosition_World_m));
    EXPECT_TRUE(p_recoveredRoom->getWalls().empty());
    EXPECT_EQ(manager.getCurrentRoomId(), 0);

    ASSERT_NE(p_recoveredRoom->getFloor(), nullptr);
    EXPECT_EQ(p_recoveredRoom->getFloor()->getId(), 0);
    ASSERT_EQ(p_recoveredRoom->getFloor()->getRooms().size(), 1U);
    EXPECT_EQ(p_recoveredRoom->getFloor()->getRooms().front(), p_recoveredRoom);

    const std::vector<semantic::Passage *> recoveredPassages =
        p_recoveryMap->GetAllPassages();
    ASSERT_EQ(recoveredPassages.size(), 1U);
    EXPECT_EQ(recoveredPassages.front()->getId(), departedPassage.getId());
    EXPECT_TRUE(recoveredPassages.front()->isRecoveryProxy());
    EXPECT_EQ(recoveredPassages.front()->getKnownSideProvenance().pRoom,
              p_recoveredRoom);
    ASSERT_EQ(p_recoveredRoom->getPassages().size(), 1U);
    EXPECT_EQ(p_recoveredRoom->getPassages().front(),
              recoveredPassages.front());
}

TEST(SemanticBootstrapPhase1,
     RepeatedNoPoseMapRecoveryPreservesCurrentRoomFloorIdentity)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);

    semantic::Room roomOne;
    roomOne.setId(1);
    roomOne.setMap(atlas.GetCurrentMap());
    roomOne.setRoomVariant(semantic::Room::RoomVariant::ROOM);
    roomOne.setName("semantic::Room#1");
    roomOne.setCentroid(Eigen::Vector3d(4.0, 0.0, 1.0));
    atlas.AddDetectedMapRoom(&roomOne);
    manager.setCurrentRoomIdForTest(roomOne.getId());
    manager.ensureActiveMapBootstrapHierarchyForTest(
        Eigen::Vector3d(4.0, 0.0, 1.0));

    ASSERT_NE(roomOne.getFloor(), nullptr);
    ASSERT_EQ(roomOne.getFloor()->getId(), 0);
    ASSERT_TRUE(roomOne.getFloor()->setPlaneIdentity(
        Eigen::Vector4d(0.0, 0.0, 1.0, 0.0),
        100U,
        4U));

    semantic::Floor *p_previousMapFloor = roomOne.getFloor();
    for (int resetIndex = 0; resetIndex < 2; ++resetIndex)
    {
        atlas.CreateNewMap();
        Map *p_recoveryMap = atlas.GetCurrentMap();
        ASSERT_NE(p_recoveryMap, nullptr);

        /* Match the live ordering: bootstrap has no pose, then floor refresh
         * runs before the first usable camera pose arrives. */
        manager.ensureActiveMapBootstrapHierarchyForTest(
            Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN()));
        manager.getUpdatedFloorsForTest();

        const std::vector<semantic::Floor *> earlyFloors = p_recoveryMap->GetAllFloors();
        ASSERT_EQ(earlyFloors.size(), 1U);
        EXPECT_EQ(earlyFloors.front()->getId(), 0);
        EXPECT_NE(earlyFloors.front(), p_previousMapFloor);
        EXPECT_FALSE(earlyFloors.front()->hasPlaneIdentity());

        const Eigen::Vector3d recoveredCameraPosition_World_m(
            8.0 + static_cast<double>(resetIndex),
            2.0,
            1.0);
        EXPECT_GE(manager.ensureActiveMapBootstrapHierarchyForTest(
                      recoveredCameraPosition_World_m),
                  0);

        const std::vector<semantic::Room *> recoveredRooms = p_recoveryMap->GetAllRooms();
        ASSERT_EQ(recoveredRooms.size(), 1U);
        semantic::Room *p_recoveredRoom = recoveredRooms.front();
        EXPECT_EQ(p_recoveredRoom->getId(), 1);
        ASSERT_NE(p_recoveredRoom->getFloor(), nullptr);
        EXPECT_EQ(p_recoveredRoom->getFloor()->getId(), 0);
        EXPECT_EQ(p_recoveredRoom->getFloor(), earlyFloors.front());
        p_previousMapFloor = earlyFloors.front();
    }
}

TEST(SemanticBootstrapPhase1,
     SameMapResetRecreatesSameRoomFloorWithoutCameraPose)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);
    ASSERT_EQ(p_roomZero->getId(), 0);

    /* Same-map reset (Tracking::ResetActiveMap): Map::clear() wipes the live
     * sets. Atlas::clearMap() must have exported the hierarchy first. */
    atlas.clearMap();
    EXPECT_TRUE(atlas.GetCurrentMap()->GetAllRooms().empty());
    EXPECT_TRUE(atlas.GetCurrentMap()->GetAllFloors().empty());

    /* No keyframes exist after the clear, so no usable camera pose. Recovery
     * must still recreate semantic::Room#0/semantic::Floor#0 from the snapshot centroid. */
    EXPECT_EQ(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d::Constant(
                      std::numeric_limits<double>::quiet_NaN())),
              1);

    const std::vector<semantic::Room *> recoveredRooms =
        atlas.GetCurrentMap()->GetAllRooms();
    ASSERT_EQ(recoveredRooms.size(), 1U);
    EXPECT_EQ(recoveredRooms.front()->getId(), 0);
    EXPECT_TRUE(recoveredRooms.front()->isRecoveryProxy());
    ASSERT_NE(recoveredRooms.front()->getFloor(), nullptr);
    EXPECT_EQ(recoveredRooms.front()->getFloor()->getId(), 0);
    EXPECT_EQ(manager.getCurrentRoomId(), 0);
}

TEST(SemanticBootstrapPhase1, BootstrapIgnoresSpuriousRoomWhenRecoveryPending)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);

    atlas.clearMap();

    /* Spurious free-space SE# classified as ROOM during the reset transient
     * (live failure 20260910-222334 cycles 135-138: SE#4 hijacked current). */
    semantic::Room spuriousRoom;
    spuriousRoom.setId(99);
    spuriousRoom.setMap(atlas.GetCurrentMap());
    spuriousRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM);
    spuriousRoom.setName("semantic::Room#99");
    spuriousRoom.setCentroid(Eigen::Vector3d(5.0, 5.0, 1.0));
    atlas.AddDetectedMapRoom(&spuriousRoom);

    EXPECT_EQ(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d(1.0, 0.0, 1.0)),
              1);

    /* Recovery identity wins: current returns to semantic::Room#0, not the spurious
     * lowest-ID fallback. Duplicate fusion owns the spurious room later. */
    EXPECT_EQ(manager.getCurrentRoomId(), 0);
    EXPECT_EQ(atlas.getCurrentSemanticRoomIdentity(), 0);
}

TEST(SemanticBootstrapPhase1,
     RoomIdentityOrderingContinuesAfterRecoveryMapBootstrap)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);

    semantic::Passage passage;
    passage.setId(atlas.reservePassageIdentity());
    passage.setMap(atlas.GetCurrentMap());
    passage.setPassable(true);
    passage.setKnownSideRoom(p_roomZero);
    atlas.AddMapPassage(&passage);
    p_roomZero->setDoorways(&passage);

    atlas.CreateNewMap();
    manager.ensureActiveMapBootstrapHierarchyForTest(
        Eigen::Vector3d(1.0, 0.0, 1.0));

    std::unique_ptr<semantic::Room> p_roomOne(GeoSemHelpers::createBlankRoomCandidate(
        &atlas,
        Eigen::Vector3d(2.0, 0.0, 1.0)));
    ASSERT_NE(p_roomOne, nullptr);
    EXPECT_EQ(p_roomOne->getId(), 1);
}

TEST(SemanticBootstrapPhase1,
     SameIdentityObservedPassageReplacesRecoveryProxyGeometry)
{
    semantic::Passage canonicalPassage;
    canonicalPassage.setId(4);
    canonicalPassage.setRecoveryProxy(true);
    canonicalPassage.setPassable(true);

    semantic::Passage observedPassage;
    observedPassage.setId(4);
    observedPassage.setRecoveryProxy(false);
    observedPassage.setPassable(false);
    observedPassage.setPassageType(semantic::Passage::PassageVariant::DOORWAY);
    observedPassage.setCentroid(Eigen::Vector3d(3.0, 2.0, 1.0));
    observedPassage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, -3.0)));
    observedPassage.setWidth(1.2);
    observedPassage.setHeight(2.1);
    ASSERT_TRUE(
        observedPassage.setKnownSideDirection(Eigen::Vector3d::UnitX()));

    EXPECT_TRUE(canonicalPassage.mergeFromDuplicate(&observedPassage));
    EXPECT_FALSE(canonicalPassage.isRecoveryProxy());
    EXPECT_FALSE(canonicalPassage.isPassable());
    EXPECT_EQ(canonicalPassage.getPassageType(),
              semantic::Passage::PassageVariant::DOORWAY);
    EXPECT_TRUE(canonicalPassage.getCentroid().isApprox(
        Eigen::Vector3d(3.0, 2.0, 1.0)));
    EXPECT_TRUE(canonicalPassage.getGlobalEquation().coeffs().isApprox(
        Eigen::Vector4d(1.0, 0.0, 0.0, -3.0)));
    EXPECT_DOUBLE_EQ(canonicalPassage.getWidth(), 1.2);
    EXPECT_DOUBLE_EQ(canonicalPassage.getHeight(), 2.1);
    EXPECT_TRUE(
        canonicalPassage.getKnownSideProvenance().direction_World.isApprox(
            Eigen::Vector3d::UnitX()));
}

TEST(SemanticBootstrapPhase1,
     SameIdentityPassageMergePreservesUniqueTopologyAndMaximumCounters)
{
    semantic::Passage canonicalPassage;
    canonicalPassage.setId(5);
    canonicalPassage.setRecoveryProxy(true);
    canonicalPassage.addTraversalObservation(
        semantic::Passage::TraversalDirection::KNOWN_TO_FAR);

    semantic::Passage duplicatePassage;
    duplicatePassage.setId(5);
    duplicatePassage.addTraversalObservation(
        semantic::Passage::TraversalDirection::KNOWN_TO_FAR);
    duplicatePassage.addTraversalObservation(
        semantic::Passage::TraversalDirection::KNOWN_TO_FAR);
    duplicatePassage.addTraversalObservation(
        semantic::Passage::TraversalDirection::FAR_TO_KNOWN);
    ASSERT_TRUE(
        duplicatePassage.setKnownSideDirection(Eigen::Vector3d::UnitY()));

    geometric::Plane supportingWall;
    duplicatePassage.addAssociateWall(&supportingWall);
    duplicatePassage.addAssociateWall(&supportingWall);

    EXPECT_FALSE(canonicalPassage.mergeFromDuplicate(&duplicatePassage));
    EXPECT_EQ(canonicalPassage.getTraversalKnownToFarCount(), 2U);
    EXPECT_EQ(canonicalPassage.getTraversalFarToKnownCount(), 1U);
    EXPECT_EQ(canonicalPassage.getTraversalUnknownCount(), 0U);
    EXPECT_TRUE(
        canonicalPassage.getKnownSideProvenance().direction_World.isApprox(
            Eigen::Vector3d::UnitY()));
    ASSERT_EQ(canonicalPassage.getAssociateWalls().size(), 1U);
    EXPECT_EQ(canonicalPassage.getAssociateWalls().front(), &supportingWall);
    EXPECT_TRUE(canonicalPassage.isRecoveryProxy());
}

TEST(SemanticBootstrapPhase1, RoomAndFloorOwnershipIsReciprocal)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);
    ASSERT_EQ(atlas.GetCurrentMap()->GetAllFloors().size(), 1U);
    semantic::Floor *p_floor = atlas.GetCurrentMap()->GetAllFloors().front();
    EXPECT_EQ(p_room->getFloor(), p_floor);
    ASSERT_EQ(p_floor->getRooms().size(), 1U);
    EXPECT_EQ(p_floor->getRooms().front(), p_room);
}

TEST(SemanticBootstrapPhase1, BootstrapSeedsCurrentRoom)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);
    EXPECT_EQ(manager.getCurrentRoomId(), p_room->getId());
}

TEST(SemanticBootstrapPhase1, IdempotentBootstrapPreservesTraversedCurrentRoom)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);

    semantic::Room roomOne;
    roomOne.setId(1);
    roomOne.setMap(atlas.GetCurrentMap());
    roomOne.setRoomVariant(semantic::Room::RoomVariant::ROOM);
    roomOne.setName("semantic::Room#1");
    roomOne.setCentroid(Eigen::Vector3d(4.0, 0.0, 1.0));
    atlas.AddDetectedMapRoom(&roomOne);

    manager.setCurrentRoomIdForTest(roomOne.getId());
    manager.ensureActiveMapBootstrapHierarchyForTest(
        Eigen::Vector3d(4.0, 0.0, 1.0));

    EXPECT_EQ(manager.getCurrentRoomId(), roomOne.getId());
    EXPECT_EQ(atlas.getCurrentSemanticRoomIdentity(), roomOne.getId());
    ASSERT_NE(roomOne.getFloor(), nullptr);
    EXPECT_EQ(roomOne.getFloor()->getId(), 0);

    std::unique_ptr<geometric::Plane> p_wall =
        makeAdmissibleWall(7, atlas.GetCurrentMap(), 5.0);
    atlas.AddMapPlane(p_wall.get());
    manager.associateAllWallsToRoomsForTest();
    EXPECT_TRUE(p_roomZero->getWalls().empty());
    ASSERT_EQ(roomOne.getWalls().size(), 1U);
    EXPECT_EQ(roomOne.getWalls().front(), p_wall.get());
}

TEST(SemanticBootstrapPhase1, OrdinaryAdmissibleWallBelongsToCurrentRoom)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);
    semantic::Room            *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);
    std::unique_ptr<geometric::Plane> p_wall = makeAdmissibleWall(1, p_map);
    atlas.AddMapPlane(p_wall.get());

    manager.associateAllWallsToRoomsForTest();

    ASSERT_EQ(p_room->getWalls().size(), 1U);
    EXPECT_EQ(p_room->getWalls().front(), p_wall.get());
    EXPECT_EQ(atlas.GetRoomWallPlaneById(1), p_wall.get());
}

TEST(SemanticBootstrapPhase1, PassageFarSideRoutingPrecedesCurrentRoomFallback)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);
    semantic::Room            *p_nearRoom = bootstrap(manager, atlas);
    ASSERT_NE(p_nearRoom, nullptr);
    p_nearRoom->setCentroid(Eigen::Vector3d(-1.0, 0.0, 1.0));

    semantic::Room farRoom;
    farRoom.setId(2);
    farRoom.setMap(p_map);
    farRoom.setCentroid(Eigen::Vector3d(1.0, 0.0, 1.0));
    farRoom.setRoomVariant(semantic::Room::RoomVariant::UNDEFINED);
    atlas.AddCandidateMapRoom(&farRoom);

    semantic::Passage passage;
    passage.setId(1);
    passage.setMap(p_map);
    passage.setPassable(true);
    passage.setCentroid(Eigen::Vector3d::Zero());
    passage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
    passage.setWidth(4.0);
    passage.setHeight(4.0);
    passage.setProspectiveRoom(&farRoom);
    atlas.AddMapPassage(&passage);

    std::unique_ptr<geometric::Plane> p_wall = makeAdmissibleWall(3, p_map, 2.0);
    atlas.AddMapPlane(p_wall.get());
    manager.associateAllWallsToRoomsForTest();

    EXPECT_TRUE(p_nearRoom->getWalls().empty());
    ASSERT_EQ(farRoom.getWalls().size(), 1U);
    EXPECT_EQ(farRoom.getWalls().front(), p_wall.get());
}

TEST(SemanticBootstrapPhase1,
     ExactSupportingWallKeepsPassageLinkedWhenRoomCentroidIsCoplanar)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);
    semantic::Room            *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);

    std::unique_ptr<geometric::Plane> p_wall = makeAdmissibleWall(11, p_map, 0.0);
    atlas.AddMapPlane(p_wall.get());
    p_room->setWalls(p_wall.get());

    semantic::Passage passage;
    passage.setId(atlas.reservePassageIdentity());
    passage.setMap(p_map);
    passage.setPassable(true);
    passage.setWidth(1.2);
    passage.setHeight(2.0);
    passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 1.0));
    passage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
    passage.addAssociateWall(p_wall.get());
    passage.setKnownSideRoom(p_room);
    ASSERT_TRUE(passage.setKnownSideDirection(Eigen::Vector3d::UnitX()));
    atlas.AddMapPassage(&passage);

    for (int cycle = 0; cycle < 7; ++cycle)
    {
        manager.associatePassagesToRoomsForTest();
    }

    EXPECT_FALSE(passage.isBad());
    ASSERT_EQ(p_room->getPassages().size(), 1U);
    EXPECT_EQ(p_room->getPassages().front(), &passage);
}

TEST(SemanticBootstrapPhase1,
     SparseRoomWithoutExactWallCannotWinPassageByProximity)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);
    semantic::Room            *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);

    /* The room owns exactly one wall at x=0 -- deliberately NOT the
     * passage's supporting wall -- and sits clearly on the negative side
     * of the passage plane, so it would win that side by centroid distance
     * alone without the proximity-association guard. */
    std::unique_ptr<geometric::Plane> p_roomWall = makeAdmissibleWall(11, p_map, 0.0);
    atlas.AddMapPlane(p_roomWall.get());
    p_room->setWalls(p_roomWall.get());
    p_room->setCentroid(Eigen::Vector3d(-2.0, 0.0, 1.0));
    ASSERT_EQ(p_room->getWalls().size(), 1U);

    /* A different but nearby parallel wall (0.5 m away, inside the
     * supporting-plane gate) supports the passage. */
    std::unique_ptr<geometric::Plane> p_supportWall = makeAdmissibleWall(12, p_map, 0.5);
    atlas.AddMapPlane(p_supportWall.get());

    semantic::Passage passage;
    passage.setId(atlas.reservePassageIdentity());
    passage.setMap(p_map);
    passage.setPassable(true);
    passage.setWidth(1.2);
    passage.setHeight(2.0);
    passage.setCentroid(Eigen::Vector3d(0.0, 0.0, 1.0));
    passage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0)));
    passage.addAssociateWall(p_supportWall.get());
    atlas.AddMapPassage(&passage);

    for (int cycle = 0; cycle < 3; ++cycle)
    {
        manager.associatePassagesToRoomsForTest();
    }

    EXPECT_TRUE(p_room->getPassages().empty());
}

TEST(SemanticBootstrapPhase1, PendingWallHasFiveCycleGraceAndGrowthReset)
{
    Atlas                  atlas(0);
    Map                   *p_map = atlas.GetCurrentMap();
    SemanticsManager       manager(&atlas);
    std::unique_ptr<geometric::Plane> p_wall = makeAdmissibleWall(1, p_map);
    atlas.AddMapPlane(p_wall.get());
    manager.associateAllWallsToRoomsForTest();

    manager.suppressUndefendedWallsForTest();
    manager.suppressUndefendedWallsForTest();
    manager.suppressUndefendedWallsForTest();
    EXPECT_EQ(manager.getPendingWallAgeForTest(1), 2);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud = p_wall->getMapClouds();
    p_cloud->push_back(p_cloud->back());
    p_wall->setMapClouds(p_cloud);
    manager.suppressUndefendedWallsForTest();
    EXPECT_EQ(manager.getPendingWallAgeForTest(1), 0);

    for (int cycle = 0; cycle < 4; ++cycle)
    {
        manager.suppressUndefendedWallsForTest();
    }
    EXPECT_FALSE(p_wall->isBad());
    manager.suppressUndefendedWallsForTest();
    EXPECT_TRUE(p_wall->isBad());
}

TEST(SemanticBootstrapPhase1, OwnedAndPassageWallsCannotRetireUndefended)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.GetCurrentMap();
    SemanticsManager manager(&atlas);
    semantic::Room            *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);
    std::unique_ptr<geometric::Plane> p_ownedWall   = makeAdmissibleWall(1, p_map);
    std::unique_ptr<geometric::Plane> p_passageWall = makeAdmissibleWall(2, p_map, 3.0);
    atlas.AddMapPlane(p_ownedWall.get());
    atlas.AddMapPlane(p_passageWall.get());
    p_room->setWalls(p_ownedWall.get());

    semantic::Passage passage;
    passage.setId(1);
    passage.setMap(p_map);
    passage.setPassable(true);
    passage.addAssociateWall(p_passageWall.get());
    atlas.AddMapPassage(&passage);

    for (int cycle = 0; cycle < 8; ++cycle)
    {
        manager.suppressUndefendedWallsForTest();
    }
    EXPECT_FALSE(p_ownedWall->isBad());
    EXPECT_FALSE(p_passageWall->isBad());
}

TEST(SemanticBootstrapPhase1, NewRoomIsUnvisitedByDefault)
{
    semantic::Room room;
    EXPECT_FALSE(room.hasPreviouslyVisited());
}

TEST(SemanticBootstrapPhase1, BootstrapRoomIsVisitedAtBirth)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_room = bootstrap(manager, atlas);
    ASSERT_NE(p_room, nullptr);
    /* The UAV starts inside the bootstrap room: presence evidences entry. */
    EXPECT_TRUE(p_room->hasPreviouslyVisited());
}

TEST(SemanticBootstrapPhase1, ResetRestoresVisitedFlag)
{
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);
    ASSERT_TRUE(p_roomZero->hasPreviouslyVisited());

    atlas.clearMap();

    EXPECT_EQ(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d::Constant(
                      std::numeric_limits<double>::quiet_NaN())),
              1);

    const std::vector<semantic::Room *> recoveredRooms =
        atlas.GetCurrentMap()->GetAllRooms();
    ASSERT_EQ(recoveredRooms.size(), 1U);
    EXPECT_EQ(recoveredRooms.front()->getId(), 0);
    EXPECT_TRUE(recoveredRooms.front()->hasPreviouslyVisited());
    EXPECT_EQ(manager.getCurrentRoomId(), 0);
}

TEST(SemanticBootstrapPhase1, ResetRestoresPassageIdentityWithoutGeometry)
{
    /* Restored passages are new objects with stable IDs: frame-free state
     * (passable, traversal history) carries over, but position, orientation,
     * and aperture dimensions are never copied across a map break. */
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    Map             *p_departedMap  = atlas.GetCurrentMap();
    semantic::Room            *p_departedRoom = bootstrap(manager, atlas);
    ASSERT_NE(p_departedRoom, nullptr);

    semantic::Passage departedPassage;
    departedPassage.setId(atlas.reservePassageIdentity());
    departedPassage.setMap(p_departedMap);
    departedPassage.setPassable(true);
    departedPassage.setCentroid(Eigen::Vector3d(1.0, 2.0, 3.0));
    departedPassage.setGlobalEquation(
        g2o::Plane3D(Eigen::Vector4d(0.0, 0.0, 1.0, -3.0)));
    departedPassage.setWidth(1.25);
    departedPassage.setHeight(2.0);
    departedPassage.setKnownSideRoom(p_departedRoom);
    departedPassage.addTraversalObservation(
        semantic::Passage::TraversalDirection::KNOWN_TO_FAR);
    atlas.AddMapPassage(&departedPassage);
    p_departedRoom->setDoorways(&departedPassage);

    atlas.CreateNewMap();
    Map *p_recoveryMap = atlas.GetCurrentMap();
    ASSERT_NE(p_recoveryMap, p_departedMap);

    EXPECT_GE(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d(8.0, 2.0, 1.0)),
              0);

    const std::vector<semantic::Passage *> recoveredPassages =
        p_recoveryMap->GetAllPassages();
    ASSERT_EQ(recoveredPassages.size(), 1U);
    EXPECT_EQ(recoveredPassages.front()->getId(), departedPassage.getId());
    EXPECT_TRUE(recoveredPassages.front()->isRecoveryProxy());
    EXPECT_TRUE(recoveredPassages.front()->isPassable());
    EXPECT_TRUE(recoveredPassages.front()->getCentroid().isApprox(
        Eigen::Vector3d::Zero()));
    EXPECT_DOUBLE_EQ(recoveredPassages.front()->getWidth(), 0.0);
    EXPECT_DOUBLE_EQ(recoveredPassages.front()->getHeight(), 0.0);
    EXPECT_EQ(recoveredPassages.front()->getTraversalKnownToFarCount(), 1U);
}

TEST(SemanticBootstrapPhase1, MapChainLinksStartingFinalAndFollowingRooms)
{
    /* Mission-chain trace: each map records its entry room, its departure
     * room, and its successor. Same-map clears link nothing. */
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    Map             *p_mapZero  = atlas.GetCurrentMap();
    semantic::Room            *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);
    EXPECT_EQ(p_mapZero->getStartingRoom(), p_roomZero);
    EXPECT_EQ(p_mapZero->getFollowingMap(), nullptr);

    atlas.CreateNewMap();
    Map *p_mapOne = atlas.GetCurrentMap();
    ASSERT_NE(p_mapOne, p_mapZero);
    EXPECT_EQ(p_mapZero->getFollowingMap(), p_mapOne);
    EXPECT_EQ(p_mapZero->getFinalRoom(), p_roomZero);

    EXPECT_GE(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d(8.0, 2.0, 1.0)),
              0);
    const std::vector<semantic::Room *> recoveredRooms = p_mapOne->GetAllRooms();
    ASSERT_EQ(recoveredRooms.size(), 1U);
    EXPECT_EQ(p_mapOne->getStartingRoom(), recoveredRooms.front());

    atlas.clearMap();
    EXPECT_EQ(p_mapOne->getFollowingMap(), nullptr);
}

TEST(SemanticBootstrapPhase1, ZeroPoseKeyFrameFallsBackToSnapshotCentroid)
{
    /* Live failure: a brand-new map's first keyframes carry identity poses,
     * whose exactly-zero centers passed the finiteness check and planted the
     * bootstrap room at the origin. Zero is rejected; the snapshot centroid
     * is used instead. */
    Atlas            atlas(0);
    SemanticsManager manager(&atlas);
    semantic::Room            *p_roomZero = bootstrap(manager, atlas);
    ASSERT_NE(p_roomZero, nullptr);

    semantic::Room roomTwo;
    roomTwo.setId(2);
    roomTwo.setMap(atlas.GetCurrentMap());
    roomTwo.setRoomVariant(semantic::Room::RoomVariant::ROOM);
    roomTwo.setName("semantic::Room#2");
    roomTwo.setCentroid(Eigen::Vector3d(2.0, 3.0, 4.0));
    atlas.AddDetectedMapRoom(&roomTwo);
    manager.setCurrentRoomIdForTest(2);

    atlas.clearMap();

    /* Uninitialized first-frame pose: finite but exactly zero. */
    KeyFrame zeroPoseKeyFrame;
    zeroPoseKeyFrame.SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(),
                                          Eigen::Vector3f::Zero()));
    atlas.GetCurrentMap()->AddKeyFrame(&zeroPoseKeyFrame);

    EXPECT_EQ(manager.ensureActiveMapBootstrapHierarchyForTest(
                  Eigen::Vector3d::Constant(
                      std::numeric_limits<double>::quiet_NaN())),
              1);

    const std::vector<semantic::Room *> recoveredRooms =
        atlas.GetCurrentMap()->GetAllRooms();
    ASSERT_EQ(recoveredRooms.size(), 1U);
    EXPECT_EQ(recoveredRooms.front()->getId(), 2);
    EXPECT_TRUE(recoveredRooms.front()->getCentroid().isApprox(
        Eigen::Vector3d(2.0, 3.0, 4.0)));
    EXPECT_FALSE(recoveredRooms.front()->getCentroid().isZero());
}
} // namespace core
} // namespace vs_graphs
