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
 * @file            setRoomCentroid.cc
 *
 * @brief           Implements SemanticsManager::setRoomCentroid(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>

#include "scene_understanding/scene_graph/room/Room.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::setRoomCentroid(
    unsigned long          mapId_in,
    int                    roomId_in,
    const Eigen::Vector3d &centroid_world_m_in)
{
    GraphLockScope               graphScope{};
    scene_graph::SceneGraph     *p_sceneGraph = nullptr;
    const SemanticsManagerStatus lockStatus =
        lockMapGraph(mapId_in,
                     GraphLockMode::GRAPH_LOCK_MODE_EXCLUSIVE,
                     graphScope,
                     p_sceneGraph);
    if (lockStatus != SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return lockStatus;
    }

    semantic::Room *p_room = nullptr;
    if (getMapRoomById(mapId_in, roomId_in, p_room) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        p_room == nullptr)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }
    if (p_room->setCentroid(centroid_world_m_in) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
