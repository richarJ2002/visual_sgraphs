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
 * @file            getFrontierPassages.cc
 *
 * @brief           Implements SemanticsManager::getFrontierPassages(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <optional>
#include <vector>

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::getFrontierPassages(
    unsigned long                     mapId_in,
    std::vector<semantic::Passage *> &passages_out) const
{
    passages_out.clear();
    GraphLockScope           graphScope{};
    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    if (lockMapGraph(mapId_in,
                     GraphLockMode::GRAPH_LOCK_MODE_SHARED,
                     graphScope,
                     p_sceneGraph) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_MAP_NOT_FOUND;
    }

    std::vector<semantic::Passage *> mapPassages{};
    if (getMapPassagesInIdOrder(mapId_in, mapPassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPassagesInIdOrder returned a failure status "
                     "although the graph is locked; continuing as before.",
                     __func__);
    }
    for (semantic::Passage *p_mapPassage : mapPassages)
    {
        /* The far-side room is looked up by id in this map's graph, so a
         * room that was flagged, freed or never created counts as missing. */
        std::optional<int> farSideRoomId{};
        if (p_mapPassage->getProspectiveRoomId(farSideRoomId) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoomId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room *p_farSideRoom = nullptr;
        if (farSideRoomId.has_value() &&
            getMapRoomById(mapId_in, *farSideRoomId, p_farSideRoom) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapRoomById returned a failure status "
                         "although the graph is locked; continuing as before.",
                         __func__);
        }
        semantic::Room::BoundaryStatus farSideBoundaryStatus{};
        if (p_farSideRoom != nullptr &&
            p_farSideRoom->getBoundaryStatus(farSideBoundaryStatus) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getBoundaryStatus returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_farSideRoom == nullptr ||
            farSideBoundaryStatus != semantic::Room::BoundaryStatus::COMPLETE)
        {
            passages_out.push_back(p_mapPassage);
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
