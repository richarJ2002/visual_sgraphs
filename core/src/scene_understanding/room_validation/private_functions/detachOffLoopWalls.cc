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
 * @file            detachOffLoopWalls.cc
 *
 * @brief           Implements detachOffLoopWalls(), declared in
 *                  room_validation/private_functions/private_functions.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/room_validation/private_functions/private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    detachOffLoopWalls(semantic::Room                         *p_room_inout,
                       const std::vector<FiniteWallSegment2d> &wallSegments_in)
{
    std::vector<geometric::Plane *> loopWalls;
    loopWalls.reserve(wallSegments_in.size());
    for (const FiniteWallSegment2d &segment : wallSegments_in)
    {
        if (segment.p_wall != nullptr)
        {
            loopWalls.push_back(segment.p_wall);
        }
    }

    std::vector<semantic::Passage *> roomPassages{};
    if (p_room_inout->getPassages(roomPassages) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassages returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<geometric::Plane *> roomWalls4{};
    if (p_room_inout->getWalls(roomWalls4) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_ownedWall : roomWalls4)
    {
        if (p_ownedWall == nullptr)
        {
            continue;
        }

        if (std::find(loopWalls.begin(), loopWalls.end(), p_ownedWall) !=
            loopWalls.end())
        {
            continue;
        }

        /* Explained when one of the room's passages has this wall among its
         * framing walls; the first such passage decides. */
        bool explainedByPassage = false;
        for (semantic::Passage *p_passage : roomPassages)
        {
            if (p_passage == nullptr)
            {
                continue;
            }
            std::vector<geometric::Plane *> supportingWalls{};
            if (p_passage->getAssociateWalls(supportingWalls) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                /*!
                 * getAssociateWalls cannot fail; continue as
                 * before.
                 */
            }
            if (std::find(supportingWalls.begin(),
                          supportingWalls.end(),
                          p_ownedWall) != supportingWalls.end())
            {
                explainedByPassage = true;
                break;
            }
        }

        if (explainedByPassage)
        {
            continue;
        }

        bool roomWasWallRemoved2{};
        if (p_room_inout->removeWall(p_ownedWall, roomWasWallRemoved2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            roomWasWallRemoved2 = false;
            RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                        "%s: removeWall rejected its input; continuing "
                        "as before.",
                        __func__);
        }
        if (roomWasWallRemoved2)
        {
            int roomId5{};
            if (p_room_inout->getId(roomId5) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int ownedWallGetId{};
            if (p_ownedWall->getId(ownedWallGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Room#" << roomId5
                      << "'s boundary is COMPLETE; detached Wall#"
                      << ownedWallGetId
                      << ", which is neither part of the closed wall "
                         "loop nor explained by any of this room's "
                         "passages."
                      << std::endl;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
