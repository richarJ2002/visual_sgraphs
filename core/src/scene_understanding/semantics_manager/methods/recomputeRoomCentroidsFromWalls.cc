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
 * @file            recomputeRoomCentroidsFromWalls.cc
 *
 * @brief           Implements
 *                  SemanticsManager::recomputeRoomCentroidsFromWalls(),
 *                  declared in SemanticsManager.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/Room/objects/RoomStatusEnum.h"
#include "scene_understanding/scene_graph/Room/Room.h"
#include <optional>
#include <rclcpp/logging.hpp>

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include "execution/profiling/profiling.h"
#include "scene_understanding/scene_graph/SceneGraph/SceneGraph.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::recomputeRoomCentroidsFromWalls(void)
{
    PROFILE_FUNCTION();
    /* Same centre rule as associateWallsToRoomsFromFreeSpaceClusters()
     * (computeRoomCentreFromWalls()); this function only runs for
     * non-FREE_SPACE roomSeg.method configurations and, as before, sets the
     * centre without damping. */
    std::vector<semantic::Room *> atlasAllRooms{};
    scene_graph::SceneGraph      *p_sceneGraph = nullptr;
    if (p_atlas->getSceneGraph(p_sceneGraph) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneGraph returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_sceneGraph->getAllRoomsInIdOrder(atlasAllRooms) !=
        scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRoomsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::semantic::Room *p_room : atlasAllRooms)
    {
        bool roomIsBad{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad)
        {
            continue;
        }

        std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
        if (p_room->getWalls(roomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (roomWalls.empty())
        {
            continue;
        }

        std::optional<Eigen::Vector3d> wallCentre_world_m{};
        if (computeRoomCentreFromWalls(p_room, roomWalls, wallCentre_world_m) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeRoomCentreFromWalls returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        if (wallCentre_world_m.has_value() &&
            p_room->setCentroid(*wallCentre_world_m) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
