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

    vs_graphs::core::semantic::Room *p_blankRoomCandidate = nullptr;
    ASSERT_EQ(
        (GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                 p_blankRoomCandidate,
                                                 Eigen::Vector3d::Zero())),
        GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> room(p_blankRoomCandidate);
    ASSERT_NE(room, nullptr);
    int id{};
    ASSERT_EQ((room->getId(id)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_EQ(id, 0);
}

TEST(RoomCreationBudget, RefusesASecondRoomWithoutAPassablePassage)
{
    Atlas atlas(0);

    vs_graphs::core::semantic::Room *p_blankRoomCandidate = nullptr;
    ASSERT_EQ(
        (GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                 p_blankRoomCandidate,
                                                 Eigen::Vector3d::Zero())),
        GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> firstRoom(p_blankRoomCandidate);
    ASSERT_NE(firstRoom, nullptr);
    ASSERT_EQ((atlas.addCandidateMapRoom(firstRoom.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    vs_graphs::core::semantic::Room *p_blankRoomCandidate2 = nullptr;
    ASSERT_EQ((GeoSemHelpers::createBlankRoomCandidate(
                  &atlas,
                  p_blankRoomCandidate2,
                  Eigen::Vector3d(1.0, 0.0, 0.0))),
              GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> secondRoom(p_blankRoomCandidate2);
    EXPECT_EQ(secondRoom, nullptr);
}

TEST(RoomCreationBudget, AllowsASecondRoomOnceAPassablePassageExists)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    vs_graphs::core::semantic::Room *p_blankRoomCandidate = nullptr;
    ASSERT_EQ(
        (GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                 p_blankRoomCandidate,
                                                 Eigen::Vector3d::Zero())),
        GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> firstRoom(p_blankRoomCandidate);
    ASSERT_NE(firstRoom, nullptr);
    ASSERT_EQ((atlas.addCandidateMapRoom(firstRoom.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    semantic::Passage passage;
    ASSERT_EQ((passage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addMapPassage(&passage)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    vs_graphs::core::semantic::Room *p_blankRoomCandidate2 = nullptr;
    ASSERT_EQ((GeoSemHelpers::createBlankRoomCandidate(
                  &atlas,
                  p_blankRoomCandidate2,
                  Eigen::Vector3d(1.0, 0.0, 0.0))),
              GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> secondRoom(p_blankRoomCandidate2);
    EXPECT_NE(secondRoom, nullptr);
}

TEST(RoomCreationBudget, ABlockedPassageDoesNotUnlockASecondRoom)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    vs_graphs::core::semantic::Room *p_blankRoomCandidate = nullptr;
    ASSERT_EQ(
        (GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                 p_blankRoomCandidate,
                                                 Eigen::Vector3d::Zero())),
        GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> firstRoom(p_blankRoomCandidate);
    ASSERT_NE(firstRoom, nullptr);
    ASSERT_EQ((atlas.addCandidateMapRoom(firstRoom.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

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
    ASSERT_EQ((atlas.addMapPassage(&blockedPassage)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    vs_graphs::core::semantic::Room *p_blankRoomCandidate2 = nullptr;
    ASSERT_EQ((GeoSemHelpers::createBlankRoomCandidate(
                  &atlas,
                  p_blankRoomCandidate2,
                  Eigen::Vector3d(1.0, 0.0, 0.0))),
              GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> secondRoom(p_blankRoomCandidate2);
    EXPECT_EQ(secondRoom, nullptr);
}

TEST(RoomCreationBudget, ThirdRoomRequiresATwoPassablePassages)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    vs_graphs::core::semantic::Room *p_blankRoomCandidate = nullptr;
    ASSERT_EQ(
        (GeoSemHelpers::createBlankRoomCandidate(&atlas,
                                                 p_blankRoomCandidate,
                                                 Eigen::Vector3d::Zero())),
        GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> firstRoom(p_blankRoomCandidate);
    ASSERT_NE(firstRoom, nullptr);
    ASSERT_EQ((atlas.addCandidateMapRoom(firstRoom.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    semantic::Passage passage;
    ASSERT_EQ((passage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((passage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addMapPassage(&passage)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    vs_graphs::core::semantic::Room *p_blankRoomCandidate2 = nullptr;
    ASSERT_EQ((GeoSemHelpers::createBlankRoomCandidate(
                  &atlas,
                  p_blankRoomCandidate2,
                  Eigen::Vector3d(1.0, 0.0, 0.0))),
              GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> secondRoom(p_blankRoomCandidate2);
    ASSERT_NE(secondRoom, nullptr);
    ASSERT_EQ((atlas.addCandidateMapRoom(secondRoom.get())),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    /* Still only one passable passage -- a third room must be refused. */
    vs_graphs::core::semantic::Room *p_blankRoomCandidate3 = nullptr;
    ASSERT_EQ((GeoSemHelpers::createBlankRoomCandidate(
                  &atlas,
                  p_blankRoomCandidate3,
                  Eigen::Vector3d(2.0, 0.0, 0.0))),
              GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> thirdRoom(p_blankRoomCandidate3);
    EXPECT_EQ(thirdRoom, nullptr);

    semantic::Passage secondPassage;
    ASSERT_EQ((secondPassage.setId(2)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((secondPassage.setMap(p_map)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((secondPassage.setPassable(true)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((atlas.addMapPassage(&secondPassage)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);

    vs_graphs::core::semantic::Room *p_blankRoomCandidate4 = nullptr;
    ASSERT_EQ((GeoSemHelpers::createBlankRoomCandidate(
                  &atlas,
                  p_blankRoomCandidate4,
                  Eigen::Vector3d(2.0, 0.0, 0.0))),
              GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS);
    std::unique_ptr<semantic::Room> thirdRoomRetry(p_blankRoomCandidate4);
    EXPECT_NE(thirdRoomRetry, nullptr);
}

} // namespace core
} // namespace vs_graphs
