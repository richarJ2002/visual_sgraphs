/*!
 * @file test_RoomCreationBudget.cpp
 * @brief User-mandated invariant: a map may hold at most one more room than
 *        it has PASSABLE passages. GeoSemHelpers::createBlankRoomCandidate()
 *        is the sole choke point that ever constructs a new semantic::Room, so
 * the cap is enforced there, regardless of which call site (free-space
 *        bootstrap, or a passage's prospective-room creation) is asking.
 *
 *        A "blocked" semantic::Passage (isPassable() == false) must not count
 * toward the budget -- a semantic::Passage can be created purely from a
 * classified door plane sitting near a wall (SemanticsManager::
 *        detectDoorsAndDoorways()), with zero free-space evidence. Only a
 *        semantic::Passage actually observed passable (real
 * Voxblox-skeleton-crosses- wall evidence) may unlock a new room.
 */

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "Map.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"

#include <gtest/gtest.h>

#include <memory>

namespace vs_graphs
{
namespace core
{

TEST(RoomCreationBudget, AllowsTheFirstRoomWithZeroPassages)
{
    Atlas atlas(0);

    std::unique_ptr<semantic::Room> room(
        GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                Eigen::Vector3d::Zero()));
    ASSERT_NE(room, nullptr);
    int id{};
    ASSERT_EQ((room->getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(id, 0);
}

TEST(RoomCreationBudget, RefusesASecondRoomWithoutAPassablePassage)
{
    Atlas atlas(0);

    std::unique_ptr<semantic::Room> firstRoom(
        GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                Eigen::Vector3d::Zero()));
    ASSERT_NE(firstRoom, nullptr);
    atlas.addCandidateMapRoom(firstRoom.get());

    std::unique_ptr<semantic::Room> secondRoom(
        GeoSemHelpers::createBlankRoomCandidate(
            &atlas,
            Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_EQ(secondRoom, nullptr);
}

TEST(RoomCreationBudget, AllowsASecondRoomOnceAPassablePassageExists)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    std::unique_ptr<semantic::Room> firstRoom(
        GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                Eigen::Vector3d::Zero()));
    ASSERT_NE(firstRoom, nullptr);
    atlas.addCandidateMapRoom(firstRoom.get());

    semantic::Passage passage;
    ASSERT_EQ((passage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    atlas.addMapPassage(&passage);

    std::unique_ptr<semantic::Room> secondRoom(
        GeoSemHelpers::createBlankRoomCandidate(
            &atlas,
            Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_NE(secondRoom, nullptr);
}

TEST(RoomCreationBudget, ABlockedPassageDoesNotUnlockASecondRoom)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    std::unique_ptr<semantic::Room> firstRoom(
        GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                Eigen::Vector3d::Zero()));
    ASSERT_NE(firstRoom, nullptr);
    atlas.addCandidateMapRoom(firstRoom.get());

    /* Represents a semantic::Passage created purely from a classified door
     * plane near a wall -- no free-space evidence yet, so it must not spend the
     * room-creation budget. */
    semantic::Passage blockedPassage;
    ASSERT_EQ((blockedPassage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((blockedPassage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((blockedPassage.setPassable(false)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    atlas.addMapPassage(&blockedPassage);

    std::unique_ptr<semantic::Room> secondRoom(
        GeoSemHelpers::createBlankRoomCandidate(
            &atlas,
            Eigen::Vector3d(1.0, 0.0, 0.0)));
    EXPECT_EQ(secondRoom, nullptr);
}

TEST(RoomCreationBudget, ThirdRoomRequiresATwoPassablePassages)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    std::unique_ptr<semantic::Room> firstRoom(
        GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                Eigen::Vector3d::Zero()));
    ASSERT_NE(firstRoom, nullptr);
    atlas.addCandidateMapRoom(firstRoom.get());

    semantic::Passage passage;
    ASSERT_EQ((passage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    atlas.addMapPassage(&passage);

    std::unique_ptr<semantic::Room> secondRoom(
        GeoSemHelpers::createBlankRoomCandidate(
            &atlas,
            Eigen::Vector3d(1.0, 0.0, 0.0)));
    ASSERT_NE(secondRoom, nullptr);
    atlas.addCandidateMapRoom(secondRoom.get());

    /* Still only one passable passage -- a third room must be refused. */
    std::unique_ptr<semantic::Room> thirdRoom(
        GeoSemHelpers::createBlankRoomCandidate(
            &atlas,
            Eigen::Vector3d(2.0, 0.0, 0.0)));
    EXPECT_EQ(thirdRoom, nullptr);

    semantic::Passage secondPassage;
    ASSERT_EQ((secondPassage.setId(2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((secondPassage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((secondPassage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    atlas.addMapPassage(&secondPassage);

    std::unique_ptr<semantic::Room> thirdRoomRetry(
        GeoSemHelpers::createBlankRoomCandidate(
            &atlas,
            Eigen::Vector3d(2.0, 0.0, 0.0)));
    EXPECT_NE(thirdRoomRetry, nullptr);
}

} // namespace core
} // namespace vs_graphs
