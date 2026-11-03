/*!
 * @file            collectRoomEvidence.cc
 *
 * @brief           Implements collectRoomEvidence(), declared in
 *                  private_functions.h.
 */

#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void collectRoomEvidence(const std::vector<Room *>  &rooms_in,
                         const SemanticVerifyConfig &configuration_in,
                         std::vector<Room *>        &admittedRooms_out,
                         std::vector<SemanticMergeRoomEvidence> &evidence_out)
{
    admittedRooms_out.clear();
    evidence_out.clear();
    for (Room *p_room : rooms_in)
    {
        if (p_room == nullptr)
        {
            continue;
        }

        bool roomIsBad{};
        if (p_room->isBad(roomIsBad) != RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Room::RoomVariant roomVariant{};
        if (!roomIsBad && p_room->getRoomVariant(roomVariant) !=
                              RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (roomIsBad || roomVariant != Room::RoomVariant::ROOM)
        {
            continue;
        }

        SemanticMergeRoomEvidence evidence{};
        if (copyMergeRoomEvidence(p_room, configuration_in, evidence) !=
            SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: copyMergeRoomEvidence returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        admittedRooms_out.push_back(p_room);
        evidence_out.push_back(evidence);
    }
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
