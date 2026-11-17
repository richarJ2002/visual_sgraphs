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
 * @file            linkRoomToPassage.cc
 *
 * @brief           Implements linkRoomToPassage(), declared in
 *                  semantics_manager/private_functions/private_functions.h.
 */

#include "private_functions.h"

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus linkRoomToPassage(
    semantic::Room    *p_room_inout,
    semantic::Passage *p_passage_in,
    const std::unordered_map<semantic::Room *, std::unordered_set<int>>
        &previousPassageIdsByRoom_in)
{
    /* Skip invalid rooms */
    if (p_room_inout == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract the passages already assigned to the room */
    std::vector<vs_graphs::core::semantic::Passage *> roomPassages{};
    if (p_room_inout->getPassages(roomPassages) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassages returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* Check whether the relationship already exists */
    bool alreadyAssociated = false;
    for (vs_graphs::core::semantic::Passage *p_existingPassage : roomPassages)
    {
        int existingPassageId{};
        if ((p_existingPassage != nullptr) &&
            p_existingPassage->getId(existingPassageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int passageId{};
        if ((p_existingPassage != nullptr) &&
            p_passage_in->getId(passageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_existingPassage != nullptr && existingPassageId == passageId)
        {
            alreadyAssociated = true;
            break;
        }
    }

    /* Add the relationship if required */
    if (!alreadyAssociated)
    {
        if (p_room_inout->addPassageAssociation(p_passage_in) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addPassageAssociation returned a failure status "
                         "although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        const std::unordered_map<semantic::Room *,
                                 std::unordered_set<int>>::const_iterator
            previousPassagesIterator =
                previousPassageIdsByRoom_in.find(p_room_inout);

        int passageId{};
        if ((previousPassagesIterator != previousPassageIdsByRoom_in.end()) &&
            p_passage_in->getId(passageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const bool relationshipAlreadyExisted =
            previousPassagesIterator != previousPassageIdsByRoom_in.end() &&
            previousPassagesIterator->second.count(passageId) > 0U;

        if (!relationshipAlreadyExisted)
        {
            int passageId2{};
            if (p_passage_in->getId(passageId2) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int room_inoutId{};
            if (p_room_inout->getId(room_inoutId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Associated Passage#" << passageId2
                      << " with Room#" << room_inoutId << "." << std::endl;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
