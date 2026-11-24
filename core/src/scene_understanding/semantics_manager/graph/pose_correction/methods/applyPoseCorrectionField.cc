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
 * @file            applyPoseCorrectionField.cc
 *
 * @brief           Implements SemanticsManager::applyPoseCorrectionField(),
 *                  declared in
 * semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <map>
#include <vector>

#include "scene_understanding/semantics_manager/graph/pose_correction/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::applyPoseCorrectionField(
    unsigned long              mapId_in,
    const map::Map            *p_map_in,
    const PoseCorrectionField &field_in)
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
    if (getMapPassagesInIdOrder(mapId_in, passages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: map %lu has no scene graph although it is locked; "
                     "continuing without its passages.",
                     __func__,
                     mapId_in);
    }
    correctPassagePoses(field_in, passages);

    std::vector<semantic::Room *> rooms{};
    std::vector<semantic::Room *> candidateRooms{};
    if (getMapDetectedRoomsInIdOrder(mapId_in, rooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
        getMapCandidateRoomsInIdOrder(mapId_in, candidateRooms) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: map %lu has no scene graph although it is locked; "
                     "continuing without its rooms.",
                     __func__,
                     mapId_in);
    }
    rooms.insert(rooms.end(), candidateRooms.begin(), candidateRooms.end());

    std::map<semantic::Room *, g2o::Sim3> roomCorrections_oldWorldToNewWorld;
    std::map<semantic::Room *, Eigen::Vector3d> roomCentroids_oldWorld_m;
    correctRoomPoses(field_in,
                     rooms,
                     roomCorrections_oldWorldToNewWorld,
                     roomCentroids_oldWorld_m);

    std::vector<semantic::Floor *> floors{};
    if (getMapFloorsInIdOrder(mapId_in, floors) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: map %lu has no scene graph although it is locked; "
                     "continuing without its floors.",
                     __func__,
                     mapId_in);
    }
    correctFloorPoses(field_in,
                      p_map_in,
                      roomCorrections_oldWorldToNewWorld,
                      roomCentroids_oldWorld_m,
                      floors);

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
