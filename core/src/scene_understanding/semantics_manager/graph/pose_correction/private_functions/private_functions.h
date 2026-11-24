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
 * @file            private_functions.h
 *
 * @brief           Declares the helpers of the SemanticsManager pose
 *                  correction methods. Package-internal: not included by any
 *                  public header.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_GRAPH_POSE_CORRECTION_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_GRAPH_POSE_CORRECTION_PRIVATE_FUNCTIONS_H

#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/semantics_manager/objects/PoseCorrectionFieldStruct.h"

#include <map>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace map
{
class Map;
} /* namespace map */

/*!
 * @brief           Moves passages by the correction of their door plane, else
 *                  of their nearest wall, else of the nearest anchor. Does not
 *                  lock: the caller holds the graph exclusive.
 *
 * @param[in]       field_in
 *                  Corrections of the map.
 *
 * @param[in,out]   passages_inout
 *                  Passages to move; borrowed, null entries skipped.
 */
void correctPassagePoses(const PoseCorrectionField        &field_in,
                         std::vector<semantic::Passage *> &passages_inout);

/*!
 * @brief           Moves rooms by the correction of their nearest wall, else
 *                  of the nearest anchor. A room listed twice moves once. Does
 *                  not lock: the caller holds the graph exclusive.
 *
 * @param[in]       field_in
 *                  Corrections of the map.
 *
 * @param[in,out]   rooms_inout
 *                  Rooms to move; borrowed, null and bad entries skipped.
 *
 * @param[out]      roomCorrections_oldWorldToNewWorld_out
 *                  Correction applied to each moved room; keys borrowed.
 *
 * @param[out]      roomCentroids_oldWorld_m_out
 *                  Centroid of each moved room before the move, old world
 *                  frame, metres; keys borrowed.
 */
void correctRoomPoses(
    const PoseCorrectionField     &field_in,
    std::vector<semantic::Room *> &rooms_inout,
    std::map<semantic::Room *, g2o::Sim3>
        &roomCorrections_oldWorldToNewWorld_out,
    std::map<semantic::Room *, Eigen::Vector3d> &roomCentroids_oldWorld_m_out);

/*!
 * @brief           Moves storeys by the correction of the nearest of their
 *                  moved rooms that belong to the map, else of the nearest
 *                  anchor. Does not lock: the caller holds the graph
 *                  exclusive.
 *
 * @param[in]       field_in
 *                  Corrections of the map.
 *
 * @param[in]       p_map_in
 *                  Map whose rooms count; borrowed.
 *
 * @param[in]       roomCorrections_oldWorldToNewWorld_in
 *                  Corrections applied to the rooms.
 *
 * @param[in]       roomCentroids_oldWorld_m_in
 *                  Room centroids before the move, old world frame, metres.
 *
 * @param[in,out]   floors_inout
 *                  Storeys to move; borrowed, null entries skipped.
 */
void correctFloorPoses(const PoseCorrectionField &field_in,
                       const map::Map            *p_map_in,
                       const std::map<semantic::Room *, g2o::Sim3>
                           &roomCorrections_oldWorldToNewWorld_in,
                       const std::map<semantic::Room *, Eigen::Vector3d>
                           &roomCentroids_oldWorld_m_in,
                       std::vector<semantic::Floor *> &floors_inout);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_GRAPH_POSE_CORRECTION_PRIVATE_FUNCTIONS_H \
        */
