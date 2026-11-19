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
 * @file            compareWallObserverRooms.cc
 *
 * @brief           Implements SemanticsManager::compareWallObserverRooms(),
 *                  declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <string>
#include <utility>
#include <vector>

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::compareWallObserverRooms(map::Map *p_activeMap_in)
{
    if (p_activeMap_in == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    unsigned long                   activeMapId{};
    std::vector<geometric::Plane *> mapPlanes{};
    if (p_activeMap_in->getId(activeMapId) !=
            map::MapStatus::MAP_STATUS_SUCCESS ||
        p_activeMap_in->getAllPlanes(mapPlanes) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId/getAllPlanes returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> mapRooms{};
    if (getMapRoomsInIdOrder(activeMapId, mapRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapRoomsInIdOrder found no scene graph for map "
                     "%lu; continuing without rooms.",
                     __func__,
                     activeMapId);
    }

    /* Live walls in id order. */
    std::vector<std::pair<int, geometric::Plane *>> liveWalls{};
    for (geometric::Plane *p_plane : mapPlanes)
    {
        bool                           isPlaneBad{};
        geometric::Plane::PlaneVariant planeVariant{};
        int                            planeId{};
        if (p_plane != nullptr &&
            (p_plane->isBad(isPlaneBad) !=
                 geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
             p_plane->getPlaneType(planeVariant) !=
                 geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
             p_plane->getId(planeId) !=
                 geometric::PlaneStatus::PLANE_STATUS_SUCCESS))
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad/getPlaneType/getId returned a failure "
                         "status although they cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        if (p_plane != nullptr && !isPlaneBad &&
            planeVariant == geometric::Plane::PlaneVariant::WALL)
        {
            liveWalls.emplace_back(planeId, p_plane);
        }
    }
    std::sort(liveWalls.begin(), liveWalls.end());

    /* Shadow comparison only: today's owner against the observer room. */
    std::size_t agreeCount           = 0U;
    std::size_t disagreeCount        = 0U;
    std::size_t noObserverRoomCount  = 0U;
    std::size_t unownedObservedCount = 0U;
    std::string disagreeWallsText{};
    std::string unownedWallsText{};
    for (const std::pair<int, geometric::Plane *> &liveWall : liveWalls)
    {
        semantic::Room *p_ownerRoom    = nullptr;
        int             ownerRoomId    = -1;
        int             observerRoomId = -1;
        if (findWallRoom(mapRooms, liveWall.second, p_ownerRoom) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
            (p_ownerRoom != nullptr &&
             p_ownerRoom->getId(ownerRoomId) !=
                 semantic::RoomStatus::ROOM_STATUS_SUCCESS) ||
            semantic::findObserverRoomId(liveWall.second, observerRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findWallRoom/getId/findObserverRoomId returned "
                         "a failure status although they cannot fail; "
                         "continuing as before.",
                         __func__);
        }
        if (observerRoomId < 0)
        {
            noObserverRoomCount++;
        }
        else if (p_ownerRoom == nullptr)
        {
            unownedObservedCount++;
            unownedWallsText += (unownedWallsText.empty() ? "" : ",") +
                                std::to_string(liveWall.first) + ">" +
                                std::to_string(observerRoomId);
        }
        else if (ownerRoomId == observerRoomId)
        {
            agreeCount++;
        }
        else
        {
            disagreeCount++;
            disagreeWallsText += (disagreeWallsText.empty() ? "" : ",") +
                                 std::to_string(liveWall.first) + ":" +
                                 std::to_string(ownerRoomId) + ">" +
                                 std::to_string(observerRoomId);
        }
    }

    /* Wall lists: "wall:owner>observer" for a disagreement, "wall>observer"
     * for a wall no room owns. */
    std::cout << "SG_PIPELINE {\"event\":\"wall_observer_room\","
                 "\"semantic_cycle\":"
              << pipelineSemanticCycle << ",\"map_id\":" << activeMapId
              << ",\"walls\":" << liveWalls.size()
              << ",\"agree\":" << agreeCount
              << ",\"disagree\":" << disagreeCount
              << ",\"no_observer_room\":" << noObserverRoomCount
              << ",\"unowned_observed\":" << unownedObservedCount
              << ",\"disagree_walls\":\"" << disagreeWallsText
              << "\",\"unowned_walls\":\"" << unownedWallsText << "\"}"
              << std::endl;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
