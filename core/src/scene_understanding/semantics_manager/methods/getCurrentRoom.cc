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
 * @file            getCurrentRoom.cc
 *
 * @brief           Implements SemanticsManager::getCurrentRoom(), declared in
 *                  SemanticsManager.h.
 */

#include "scene_understanding/scene_graph/SceneGraph/SceneGraph.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::getCurrentRoom(semantic::Room *&p_currentRoom_out) const
{
    /* Clear room object */
    p_currentRoom_out = nullptr;

    /* Init integer to track id of current room robot is in */
    int currentRoomIdSnapshot{};

    /* Get the room id that the robot is currently in */
    if (getCurrentRoomId(currentRoomIdSnapshot) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentRoomId returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Ini variable which will extract the room object address */
    semantic::Room *p_roomById = nullptr;

    /* Extract the room by id */
    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    if (p_atlas->getSceneGraph(p_sceneGraph) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneGraph returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_sceneGraph->getRoomById(currentRoomIdSnapshot, p_roomById) !=
        scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getRoomById returned a failure status; continuing as before.",
            __func__);
    }

    /* Init variable to track rooms variant */
    semantic::Room::RoomVariant roomVariant{};

    /* Extract the room variant */
    if ((p_roomById != nullptr) &&
        p_roomById->getRoomVariant(roomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomVariant returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* Confirm that the current room is a room semantic */
    if (p_roomById == nullptr &&
        roomVariant != semantic::Room::RoomVariant::ROOM)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: Current room robot is in is not classed as a room "
                     "semantic variant",
                     __func__);
        return SemanticsManagerStatus::
            SEMANTIC_MANAGER_STATUS_INVALID_CURRENT_ROOM;
    }

    /* Output the address of the current room */
    p_currentRoom_out = p_roomById;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
