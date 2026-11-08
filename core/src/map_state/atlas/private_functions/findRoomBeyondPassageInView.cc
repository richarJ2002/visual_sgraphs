/*!
 * @file            findRoomBeyondPassageInView.cc
 *
 * @brief           Implements findRoomBeyondPassageInView(), declared in
 *                  private_functions.h.
 */

#include "private_functions.h"

#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

#include <algorithm>
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

semantic::Room *
    findRoomBeyondPassageInView(const std::vector<semantic::Room *> &rooms_in,
                                semantic::Room        *p_finalRoom_in,
                                keyframe::KeyFrame    *p_keyFrame_in,
                                const Eigen::Vector3d &upDirection_world_in,
                                const std::pair<float, float> &depthRange_m_in,
                                semantic::Passage *&p_passageInView_out)
{
    p_passageInView_out = nullptr;
    if (p_finalRoom_in == nullptr || p_keyFrame_in == nullptr)
    {
        return nullptr;
    }
    std::vector<semantic::Passage *> finalRoomPassages{};
    if (p_finalRoom_in->getPassages(finalRoomPassages) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* The nearest passage in view with a known room beyond it wins. */
    semantic::Room *p_roomBeyondPassage = nullptr;
    double          nearestDepth_m      = std::numeric_limits<double>::max();
    for (semantic::Passage *p_passage : finalRoomPassages)
    {
        bool passageIsBad = true;
        if (p_passage != nullptr &&
            p_passage->isBad(passageIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        double openingDepth_m = 0.0;
        if (p_passage == nullptr || passageIsBad ||
            !isPassageOpeningInView(p_passage,
                                    p_keyFrame_in,
                                    upDirection_world_in,
                                    depthRange_m_in,
                                    openingDepth_m) ||
            openingDepth_m >= nearestDepth_m)
        {
            continue;
        }

        /* The room beyond is the other room that lists this passage, or else
         * the passage's far-side room, when it is one of this map's rooms. */
        semantic::Room *p_candidateRoom = nullptr;
        for (semantic::Room *p_room : rooms_in)
        {
            bool                             roomIsBad = true;
            std::vector<semantic::Passage *> roomPassages{};
            if (p_room != nullptr && p_room != p_finalRoom_in &&
                (p_room->isBad(roomIsBad) !=
                     semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
                 p_room->getPassages(roomPassages) !=
                     semantic::RoomStatus::ROOM_STATUS_SUCCESS))
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad/getPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!roomIsBad && std::find(roomPassages.begin(),
                                        roomPassages.end(),
                                        p_passage) != roomPassages.end())
            {
                p_candidateRoom = p_room;
                break;
            }
        }
        if (p_candidateRoom == nullptr)
        {
            semantic::Room *p_prospectiveRoom = nullptr;
            if (p_passage->getProspectiveRoom(p_prospectiveRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
            bool prospectiveRoomIsBad = true;
            if (p_prospectiveRoom != nullptr &&
                p_prospectiveRoom != p_finalRoom_in &&
                std::find(rooms_in.begin(),
                          rooms_in.end(),
                          p_prospectiveRoom) != rooms_in.end() &&
                p_prospectiveRoom->isBad(prospectiveRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!prospectiveRoomIsBad)
            {
                p_candidateRoom = p_prospectiveRoom;
            }
        }
        if (p_candidateRoom != nullptr)
        {
            p_roomBeyondPassage = p_candidateRoom;
            p_passageInView_out = p_passage;
            nearestDepth_m      = openingDepth_m;
        }
    }
    return p_roomBeyondPassage;
}

} /* namespace core */
} /* namespace vs_graphs */
