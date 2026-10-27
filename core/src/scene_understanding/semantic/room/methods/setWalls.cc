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
 * @file            setWalls.cc
 *
 * @brief           Implements Room::setWalls(), declared in
 * semantic/room/objects/RoomClass.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "scene_understanding/semantic/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantic/passage/passage.h"
#include "scene_understanding/semantic/room/objects/RoomClass.h"
#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::setWalls(geometric::Plane *p_wall_in)
{
    /* Confirm that input wall is valid */
    if (p_wall_in == nullptr)
    {
        return RoomStatus::ROOM_STATUS_SUCCESS;
    }

    if (setRecoveryProxy(false) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setRecoveryProxy returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /*
     * A passage is framed by the two faces of one wall, which bound two
     * different rooms: never take a face of a linked passage whose other
     * face this room already owns.
     */
    std::vector<Passage *>          linkedPassages{};
    std::vector<geometric::Plane *> ownedWalls{};
    if (getPassages(linkedPassages) != RoomStatus::ROOM_STATUS_SUCCESS ||
        getWalls(ownedWalls) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassages/getWalls returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    for (Passage *p_passage : linkedPassages)
    {
        std::vector<geometric::Plane *> passageWalls{};
        if (p_passage == nullptr ||
            p_passage->getAssociateWalls(passageWalls) !=
                PassageStatus::PASSAGE_STATUS_SUCCESS ||
            std::find(passageWalls.begin(), passageWalls.end(), p_wall_in) ==
                passageWalls.end())
        {
            continue;
        }
        const bool ownsOtherFace =
            std::any_of(passageWalls.begin(),
                        passageWalls.end(),
                        [&](geometric::Plane *p_face)
                        {
                            return p_face != p_wall_in &&
                                   std::find(ownedWalls.begin(),
                                             ownedWalls.end(),
                                             p_face) != ownedWalls.end();
                        });
        /* The host (first) face bounds the near side, never the far side. */
        Room *p_prospectiveRoom = nullptr;
        if (p_passage->getProspectiveRoom(p_prospectiveRoom) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (ownsOtherFace ||
            (p_prospectiveRoom == this && passageWalls.front() == p_wall_in))
        {
            return RoomStatus::ROOM_STATUS_SUCCESS;
        }
    }

    std::lock_guard<std::mutex> lock(wallsMutex);

    /* Deduplicate membership within this room; the manager owns global policy.
     */
    const bool isAlreadyAssociated =
        std::any_of(walls.begin(),
                    walls.end(),
                    [p_wall_in](const geometric::Plane *p_existingWall)
                    { return p_existingWall == p_wall_in; });

    if (!isAlreadyAssociated)
    {
        walls.push_back(p_wall_in);
    }

    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
