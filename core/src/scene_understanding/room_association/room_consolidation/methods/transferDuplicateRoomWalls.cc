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
 * @file            transferDuplicateRoomWalls.cc
 *
 * @brief           Implements SemanticsManager::transferDuplicateRoomWalls(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::transferDuplicateRoomWalls(
    semantic::Room *p_retainedRoom_in,
    semantic::Room *p_duplicateRoom_in,
    unsigned long   mapId_in,
    bool           &areAllWallsAdmitted_out)
{
    std::vector<std::pair<semantic::Room *, std::vector<geometric::Plane *>>>
                                  wallSnapshots;
    std::vector<semantic::Room *> currentMapAllRooms2{};
    if (getMapRoomsInIdOrder(mapId_in, currentMapAllRooms2) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapRoomsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    for (semantic::Room *p_snapshotRoom : currentMapAllRooms2)
    {
        bool snapshotRoomIsBad{};
        if ((p_snapshotRoom != nullptr) &&
            p_snapshotRoom->isBad(snapshotRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_snapshotRoom != nullptr && !snapshotRoomIsBad)
        {
            std::vector<geometric::Plane *> snapshotRoomWalls{};
            if (p_snapshotRoom->getWalls(snapshotRoomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getWalls returned a failure status although it "
                    "cannot fail; continuing as before.",
                    __func__);
            }
            wallSnapshots.emplace_back(p_snapshotRoom, snapshotRoomWalls);
        }
    }

    areAllWallsAdmitted_out = true;

    std::vector<geometric::Plane *> duplicateRoomWalls{};
    if (p_duplicateRoom_in->getWalls(duplicateRoomWalls) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_wall : duplicateRoomWalls)
    {
        bool wasAdmitted{};
        if (admitWallToRoom(p_retainedRoom_in, p_wall, wasAdmitted) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: admitWallToRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!wasAdmitted)
        {
            areAllWallsAdmitted_out = false;
            break;
        }
    }

    /*
     * A topology-rejected provisional wall is not a duplicate room. Keep
     * that structural element alive so another free-space component can
     * claim it instead of repeatedly deleting and recreating it.
     */
    if (!areAllWallsAdmitted_out)
    {
        for (auto &[p_snapshotRoom, snapshotWalls] : wallSnapshots)
        {
            if (p_snapshotRoom->clearWalls() !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: clearWalls returned a failure status although it "
                    "cannot fail; continuing as before.",
                    __func__);
            }
            for (geometric::Plane *p_snapshotWall : snapshotWalls)
            {
                if (p_snapshotRoom->setWalls(p_snapshotWall) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setWalls returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
