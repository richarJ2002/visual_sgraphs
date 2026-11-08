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
 * @file            replacePlane.cc
 *
 * @brief           Implements SceneGraph::replacePlane(), declared in
 *                  scene_graph/scene_graph/objects/SceneGraphClass.h.
 */

#include "scene_understanding/scene_graph/scene_graph/objects/SceneGraphClass.h"

#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <shared_mutex>

#include "mapping/geometric/Plane/Plane.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/scene_graph/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

SceneGraphStatus
    SceneGraph::replacePlane(geometric::Plane *p_retiredPlane_inout,
                             geometric::Plane *p_retainedPlane_inout)
{
    std::unique_lock<std::shared_mutex> graphLock(graphMutex);
    if (!activeMapId.has_value())
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
    }
    const std::map<unsigned long, SceneMap *>::iterator sceneMapIterator =
        sceneMaps.find(*activeMapId);
    if (sceneMapIterator == sceneMaps.end())
    {
        return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
    }

    SceneMap &sceneMap = *sceneMapIterator->second;
    for (std::map<int, std::unique_ptr<semantic::Room>> *p_roomsById :
         {&sceneMap.detectedRooms, &sceneMap.candidateRooms})
    {
        for (std::pair<const int, std::unique_ptr<semantic::Room>> &roomEntry :
             *p_roomsById)
        {
            if (!isRoomLive(roomEntry.second.get()))
            {
                continue;
            }
            bool roomWasWallReplaced{};
            bool roomWasGroundPlaneReplaced{};
            if (roomEntry.second->replaceWall(p_retiredPlane_inout,
                                              p_retainedPlane_inout,
                                              roomWasWallReplaced) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
                roomEntry.second->replaceGroundPlane(
                    p_retiredPlane_inout,
                    p_retainedPlane_inout,
                    roomWasGroundPlaneReplaced) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                            "%s: replaceWall/replaceGroundPlane rejected "
                            "its input; continuing as before.",
                            __func__);
            }
        }
    }

    for (std::pair<const int, std::unique_ptr<semantic::Passage>>
             &passageEntry : sceneMap.passages)
    {
        bool passageWasAssociationReplaced{};
        if (isPassageLive(passageEntry.second.get()) &&
            passageEntry.second->replacePlaneAssociation(
                p_retiredPlane_inout,
                p_retainedPlane_inout,
                passageWasAssociationReplaced) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                        "%s: replacePlaneAssociation rejected its input; "
                        "continuing as before.",
                        __func__);
        }
    }
    return SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS;
}

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */
