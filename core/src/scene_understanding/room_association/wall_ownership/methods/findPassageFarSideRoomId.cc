/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            findPassageFarSideRoomId.cc
 *
 * @brief           Implements SemanticsManager::findPassageFarSideRoomId(),
 *                  declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <vector>

#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::findPassageFarSideRoomId(
    semantic::Passage                   *p_passage_in,
    int                                  nearRoomId_in,
    const Eigen::Vector3d               &farPoint_world_m_in,
    const std::vector<semantic::Room *> &rooms_in,
    int                                 &farSideRoomId_out) const
{
    farSideRoomId_out = -1;

    /* The passage's rooms: its known-side and prospective rooms and every
     * room that lists it. */
    semantic::Passage::KnownSideProvenance knownSide{};
    semantic::Room                        *p_prospectiveRoom = nullptr;
    g2o::Plane3D                           passagePlane{};
    if (p_passage_in->getKnownSideProvenance(knownSide) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_in->getProspectiveRoom(p_prospectiveRoom) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage_in->getGlobalEquation(passagePlane) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKnownSideProvenance/getProspectiveRoom/"
                     "getGlobalEquation returned a failure status although "
                     "they cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> passageRooms{knownSide.p_room,
                                               p_prospectiveRoom};
    for (semantic::Room *p_room : rooms_in)
    {
        std::vector<semantic::Passage *> roomPassages{};
        if (p_room != nullptr &&
            p_room->getPassages(roomPassages) ==
                semantic::RoomStatus::ROOM_STATUS_SUCCESS &&
            std::find(roomPassages.begin(), roomPassages.end(), p_passage_in) !=
                roomPassages.end())
        {
            passageRooms.push_back(p_room);
        }
    }

    /* Live rooms other than the near room, each once. */
    std::vector<semantic::Room *> farRoomCandidates{};
    for (semantic::Room *p_room : passageRooms)
    {
        bool isRoomBad{true};
        int  roomId{-1};
        if (p_room == nullptr ||
            p_room->isBad(isRoomBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            isRoomBad ||
            p_room->getId(roomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            roomId == nearRoomId_in ||
            std::find(farRoomCandidates.begin(),
                      farRoomCandidates.end(),
                      p_room) != farRoomCandidates.end())
        {
            continue;
        }
        farRoomCandidates.push_back(p_room);
    }

    /* One other room is the far side. With more, the far side is the one
     * whose centroid lies on the far point's side of the passage plane. */
    const Eigen::Vector4d passageEquation = passagePlane.coeffs();
    const double          farPointSide_m =
        passageEquation.head<3>().dot(farPoint_world_m_in) + passageEquation(3);
    for (semantic::Room *p_candidateRoom : farRoomCandidates)
    {
        Eigen::Vector3d candidateCentroid_world_m{};
        if (p_candidateRoom->getCentroid(candidateCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const double candidateSide_m =
            passageEquation.head<3>().dot(candidateCentroid_world_m) +
            passageEquation(3);
        if (farRoomCandidates.size() == 1U ||
            candidateSide_m * farPointSide_m > 0.0)
        {
            if (p_candidateRoom->getId(farSideRoomId_out) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            break;
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
