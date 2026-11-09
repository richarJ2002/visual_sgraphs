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
 * @file            transformSceneGraph.cc
 *
 * @brief           Implements SemanticsManager::transformSceneGraph(),
 *                  declared in
 * semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <vector>

#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::transformSceneGraph(
    unsigned long    mapId_in,
    const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    GraphLockScope graphScope{};
    if (lockGraphs({mapId_in},
                   GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
                   graphScope) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    std::vector<semantic::Passage *> passages{};
    std::vector<semantic::Room *>    detectedRooms{};
    std::vector<semantic::Room *>    candidateRooms{};
    std::vector<semantic::Floor *>   floors{};
    if (getMapPassagesInIdOrder(mapId_in, passages) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        getMapDetectedRoomsInIdOrder(mapId_in, detectedRooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        getMapCandidateRoomsInIdOrder(mapId_in, candidateRooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        getMapFloorsInIdOrder(mapId_in, floors) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: map %lu has no scene graph although it is locked; "
                     "its rooms, passages and floors are not moved.",
                     __func__,
                     mapId_in);
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    for (semantic::Passage *p_passage : passages)
    {
        if (p_passage != nullptr &&
            p_passage->applyTransform(transform_oldWorldToNewWorld_in) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    detectedRooms.insert(detectedRooms.end(),
                         candidateRooms.begin(),
                         candidateRooms.end());
    for (semantic::Room *p_room : detectedRooms)
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad &&
            p_room->applyTransform(transform_oldWorldToNewWorld_in) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    for (semantic::Floor *p_floor : floors)
    {
        if (p_floor != nullptr &&
            p_floor->applyTransform(transform_oldWorldToNewWorld_in) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyTransform returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
