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
 * @file            getIsPassageFaceRefused.cc
 *
 * @brief           Implements Room::getIsPassageFaceRefused(), declared in
 *                  scene_graph/room/objects/RoomClass.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/objects/RoomClass.h"
#include <algorithm>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

RoomStatus Room::getIsPassageFaceRefused(geometric::Plane *p_wall_in,
                                         bool             &isRefused_out) const
{
    isRefused_out = false;
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
            isRefused_out = true;
            return RoomStatus::ROOM_STATUS_SUCCESS;
        }
    }
    return RoomStatus::ROOM_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
