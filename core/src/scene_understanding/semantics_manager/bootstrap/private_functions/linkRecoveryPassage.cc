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
 * @file            linkRecoveryPassage.cc
 *
 * @brief           Implements linkRecoveryPassage(), declared in
 *                  bootstrap/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/bootstrap/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void linkRecoveryPassage(const semantic::PassageContext &passageContext_in,
                         int                recoveredRoomPreviousId_in,
                         semantic::Passage *p_passage_inout,
                         semantic::Room    *p_room_inout)
{
    /* Context room ids are the recovered room's previous identity. */
    if (passageContext_in.hasKnownSideRoom &&
        passageContext_in.knownSideRoomId == recoveredRoomPreviousId_in)
    {
        if (p_passage_inout->setKnownSideRoom(p_room_inout) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setKnownSideRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    if (passageContext_in.hasFarSideRoom &&
        passageContext_in.secondaryRoomId == recoveredRoomPreviousId_in)
    {
        if (p_passage_inout->setProspectiveRoom(p_room_inout) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* A passage that still names no room gets the recovered room as its
     * known side. */
    semantic::Passage::KnownSideProvenance knownSideProvenance{};
    semantic::Room                        *p_prospectiveRoom = nullptr;
    if (p_passage_inout->getKnownSideProvenance(knownSideProvenance) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKnownSideProvenance returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if ((knownSideProvenance.p_room == nullptr) &&
        p_passage_inout->getProspectiveRoom(p_prospectiveRoom) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getProspectiveRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (knownSideProvenance.p_room == nullptr && p_prospectiveRoom == nullptr)
    {
        if (p_passage_inout->setKnownSideRoom(p_room_inout) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setKnownSideRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    if (p_room_inout->addPassageAssociation(p_passage_inout) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: addPassageAssociation returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
