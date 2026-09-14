/**
 * @file test_RoomCreationBudget.cpp
 * @brief User-mandated invariant: a map may hold at most one more room than
 *        it has PASSABLE passages. GeoSemHelpers::createBlankRoomCandidate()
 *        is the sole choke point that ever constructs a new Room, so the
 *        cap is enforced there, regardless of which call site (free-space
 *        bootstrap, or a passage's prospective-room creation) is asking.
 *
 *        A "blocked" Passage (isPassable() == false) must not count toward
 *        the budget -- a Passage can be created purely from a classified
 *        door plane sitting near a wall (SemanticsManager::
 *        detectDoorsAndDoorways()), with zero free-space evidence. Only a
 *        Passage actually observed passable (real Voxblox-skeleton-crosses-
 *        wall evidence) may unlock a new room.
 */

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "Map.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"

#include <gtest/gtest.h>

#include <memory>

namespace ORB_SLAM3
{

TEST(RoomCreationBudget, AllowsTheFirstRoomWithZeroPassages)
{
    Atlas atlas(0);

    std::unique_ptr<Room> room(
        GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                Eigen::Vector3d::Zero()));
    ASSERT_NE(room, nullptr);
    EXPECT_EQ(room->getId(), 0);
}

TEST(RoomCreationBudget, RefusesASecondRoomWithoutAPassablePassage)
{
    Atlas atlas(0);

    std::unique_ptr<Room> firstRoom(
        GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                Eigen::Vector3d::Zero()));
    ASSERT_NE(firstRoom, nullptr);
    atlas.AddCandidateMapRoom(firstRoom.get());

    std::unique_ptr<Room> secondRoom(GeoSemHelpers::createBlankRoomCandidate(
        &atlas,
        Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_EQ(secondRoom, nullptr);
}

TEST(RoomCreationBudget, AllowsASecondRoomOnceAPassablePassageExists)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    std::unique_ptr<Room> firstRoom(
        GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                Eigen::Vector3d::Zero()));
    ASSERT_NE(firstRoom, nullptr);
    atlas.AddCandidateMapRoom(firstRoom.get());

    Passage passage;
    passage.setId(1);
    passage.setMap(p_map);
    passage.setPassable(true);
    atlas.AddMapPassage(&passage);

    std::unique_ptr<Room> secondRoom(GeoSemHelpers::createBlankRoomCandidate(
        &atlas,
        Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_NE(secondRoom, nullptr);
}

TEST(RoomCreationBudget, ABlockedPassageDoesNotUnlockASecondRoom)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    std::unique_ptr<Room> firstRoom(
        GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                Eigen::Vector3d::Zero()));
    ASSERT_NE(firstRoom, nullptr);
    atlas.AddCandidateMapRoom(firstRoom.get());

    /* Represents a Passage created purely from a classified door plane near
     * a wall -- no free-space evidence yet, so it must not spend the
     * room-creation budget. */
    Passage blockedPassage;
    blockedPassage.setId(1);
    blockedPassage.setMap(p_map);
    blockedPassage.setPassable(false);
    atlas.AddMapPassage(&blockedPassage);

    std::unique_ptr<Room> secondRoom(GeoSemHelpers::createBlankRoomCandidate(
        &atlas,
        Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_EQ(secondRoom, nullptr);
}

TEST(RoomCreationBudget, ThirdRoomRequiresATwoPassablePassages)
{
    Atlas atlas(0);
    Map  *p_map = atlas.GetCurrentMap();

    std::unique_ptr<Room> firstRoom(
        GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                Eigen::Vector3d::Zero()));
    ASSERT_NE(firstRoom, nullptr);
    atlas.AddCandidateMapRoom(firstRoom.get());

    Passage passage;
    passage.setId(1);
    passage.setMap(p_map);
    passage.setPassable(true);
    atlas.AddMapPassage(&passage);

    std::unique_ptr<Room> secondRoom(GeoSemHelpers::createBlankRoomCandidate(
        &atlas,
        Eigen::Vector3d(1.0, 0.0, 0.0)));
    ASSERT_NE(secondRoom, nullptr);
    atlas.AddCandidateMapRoom(secondRoom.get());

    /* Still only one passable passage -- a third room must be refused. */
    std::unique_ptr<Room> thirdRoom(GeoSemHelpers::createBlankRoomCandidate(
        &atlas,
        Eigen::Vector3d(2.0, 0.0, 0.0)));
    EXPECT_EQ(thirdRoom, nullptr);

    Passage secondPassage;
    secondPassage.setId(2);
    secondPassage.setMap(p_map);
    secondPassage.setPassable(true);
    atlas.AddMapPassage(&secondPassage);

    std::unique_ptr<Room> thirdRoomRetry(
        GeoSemHelpers::createBlankRoomCandidate(
            &atlas,
            Eigen::Vector3d(2.0, 0.0, 0.0)));
    EXPECT_NE(thirdRoomRetry, nullptr);
}

} // namespace ORB_SLAM3
