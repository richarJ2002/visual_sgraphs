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
 * @file            unlinkRetiredRooms.cc
 *
 * @brief           Implements unlinkRetiredRooms(), declared in
 *                  semantics_manager/graph/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/graph/private_functions/private_functions.h"

#include <memory>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{

void unlinkRetiredRooms(
    const std::vector<std::unique_ptr<semantic::Room>> &retiredRooms_in,
    const std::vector<semantic::Floor *>               &liveFloors_in,
    const std::vector<semantic::Passage *>             &livePassages_in)
{
    for (const std::unique_ptr<semantic::Room> &p_retiredRoom : retiredRooms_in)
    {
        /* The room's planes and key frames belong to the SLAM map and are
         * not freed; the room only lets go of them, and its key frames are
         * left without a room. */
        if (p_retiredRoom->clearWalls() !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            p_retiredRoom->setGroundPlane(nullptr) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            p_retiredRoom->transferKeyFrames(nullptr) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: clearWalls/setGroundPlane/transferKeyFrames "
                         "returned a failure status although they cannot "
                         "fail; continuing as before.",
                         __func__);
        }

        for (semantic::Floor *p_floor : liveFloors_in)
        {
            if (p_floor->detachRoom(p_retiredRoom.get()) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: detachRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        for (semantic::Passage *p_passage : livePassages_in)
        {
            semantic::Passage::KnownSideProvenance knownSideProvenance{};
            semantic::Room                        *p_prospectiveRoom = nullptr;
            if (p_passage->getKnownSideProvenance(knownSideProvenance) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
                p_passage->getProspectiveRoom(p_prospectiveRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getKnownSideProvenance/getProspectiveRoom "
                             "returned a failure status although they cannot "
                             "fail; continuing as before.",
                             __func__);
            }
            if (knownSideProvenance.p_room == p_retiredRoom.get() &&
                p_passage->setKnownSideRoom(nullptr) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setKnownSideRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_prospectiveRoom == p_retiredRoom.get() &&
                p_passage->setProspectiveRoom(nullptr) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
