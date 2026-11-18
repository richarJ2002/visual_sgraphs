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
 * @file            enforcePassageTopologyInvariant.cc
 *
 * @brief           Implements
 * SemanticsManager::enforcePassageTopologyInvariant(), declared in
 * SemanticsManager.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::enforcePassageTopologyInvariant(void)
{
    std::vector<semantic::Passage *> allPassages{};
    std::vector<semantic::Room *>    allRooms{};
    if (getAllPassagesInIdOrder(allPassages) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        getAllRoomsInIdOrder(allRooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassagesInIdOrder/getAllRoomsInIdOrder "
                     "returned a failure status; continuing as before.",
                     __func__);
    }

    for (semantic::Passage *p_passage : allPassages)
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
        if (p_passage == nullptr || passageIsBad)
        {
            continue;
        }

        /* Owning room of each valid wall, host (first) wall first. */
        std::vector<geometric::Plane *> passageWalls{};
        if (p_passage->getAssociateWalls(passageWalls) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getAssociateWalls returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        std::vector<semantic::Room *> wallRooms{};
        for (geometric::Plane *p_wall : passageWalls)
        {
            bool wallIsBad = true;
            if (p_wall != nullptr &&
                p_wall->isBad(wallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_wall == nullptr || wallIsBad)
            {
                continue;
            }
            semantic::Room *p_wallRoom = nullptr;
            if (findWallRoom(allRooms, p_wall, p_wallRoom) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: findWallRoom returned a failure status although it "
                    "cannot fail; continuing as before.",
                    __func__);
            }
            wallRooms.push_back(p_wallRoom);
        }

        std::size_t linkedRoomCount = 0U;
        for (semantic::Room *p_room : allRooms)
        {
            bool                             roomIsBad = true;
            std::vector<semantic::Passage *> roomPassages{};
            if (p_room != nullptr &&
                (p_room->isBad(roomIsBad) !=
                     semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
                 p_room->getPassages(roomPassages) !=
                     semantic::RoomStatus::ROOM_STATUS_SUCCESS))
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad/getPassages returned a failure status "
                             "although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room != nullptr && !roomIsBad &&
                std::find(roomPassages.begin(),
                          roomPassages.end(),
                          p_passage) != roomPassages.end())
            {
                linkedRoomCount++;
            }
        }

        semantic::Room *p_prospectiveRoom = nullptr;
        if (p_passage->getProspectiveRoom(p_prospectiveRoom) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getProspectiveRoom returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }

        bool        isTopologyValid{};
        std::string violationReason{};
        if (isPassageTopologyValid(wallRooms,
                                   linkedRoomCount,
                                   p_prospectiveRoom,
                                   isTopologyValid,
                                   violationReason) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isPassageTopologyValid returned a failure status "
                         "although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (isTopologyValid)
        {
            continue;
        }

        int passageId{};
        if (p_passage->setBad() !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_passage->getId(passageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: setBad/getId returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        std::cout
            << "SG_PIPELINE {\"event\":\"passage_invalid\",\"passage_id\":"
            << passageId << ",\"reason\":\"" << violationReason << "\"}"
            << std::endl;
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
