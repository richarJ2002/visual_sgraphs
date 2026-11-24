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
 * @file            retireProspectiveRoom.cc
 *
 * @brief           Implements SemanticsManager::retireProspectiveRoom(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::retireProspectiveRoom(
    semantic::Passage *p_passage_inout,
    semantic::Room    *p_prospectiveRoom_inout,
    semantic::Room    *p_farSideConfirmedRoom_in)
{
    /* Transfer uniquely held evidence before retiring the
     * distinct placeholder. Failed admissions remain unowned
     * for the normal wall-association pass below. */
    if (p_passage_inout->setProspectiveRoom(p_farSideConfirmedRoom_in) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setProspectiveRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<geometric::Plane *> prospectiveRoomWalls2{};
    if (p_prospectiveRoom_inout->getWalls(prospectiveRoomWalls2) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWalls returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::geometric::Plane *p_wall : prospectiveRoomWalls2)
    {
        bool wasAdmitted{};
        if (admitWallToRoom(p_farSideConfirmedRoom_in, p_wall, wasAdmitted) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: admitWallToRoom returned a "
                         "failure status although it cannot "
                         "fail; continuing as before.",
                         __func__);
        }
        bool prospectiveRoomWasWallRemoved{};
        if (p_prospectiveRoom_inout->removeWall(
                p_wall,
                prospectiveRoomWasWallRemoved) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            prospectiveRoomWasWallRemoved = false;
            RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                        "%s: removeWall rejected its input; "
                        "continuing as before.",
                        __func__);
        }
    }

    int prospectiveRoomId4{};
    if (p_prospectiveRoom_inout->getId(prospectiveRoomId4) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    prospectiveRoomCycles.erase(prospectiveRoomId4);

    map::Map *p_roomMap = nullptr;
    if (p_prospectiveRoom_inout->getMap(p_roomMap) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    unsigned long roomMapId{};
    if (p_roomMap != nullptr)
    {
        if (p_roomMap->getId(roomMapId) != map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (eraseRoom(roomMapId, p_prospectiveRoom_inout) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseRoom returned a failure "
                         "status; continuing as before.",
                         __func__);
        }
    }

    if (p_prospectiveRoom_inout->clearPassages() !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: clearPassages returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (markRoomBad(roomMapId, p_prospectiveRoom_inout) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: markRoomBad found no scene graph for map %lu; "
                     "the room is flagged alone; continuing as before.",
                     __func__,
                     roomMapId);
    }

    if (p_farSideConfirmedRoom_in->addPassageAssociation(p_passage_inout) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addPassageAssociation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    int prospectiveRoomId5{};
    if (p_prospectiveRoom_inout->getId(prospectiveRoomId5) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    int farSideConfirmedRoomId2{};
    if (p_farSideConfirmedRoom_in->getId(farSideConfirmedRoomId2) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    int passageId19{};
    if (p_passage_inout->getId(passageId19) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "[SemMgr] Resolved prospective Room#" << prospectiveRoomId5
              << " with confirmed Room#" << farSideConfirmedRoomId2
              << " for Passage#" << passageId19 << "." << std::endl;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
