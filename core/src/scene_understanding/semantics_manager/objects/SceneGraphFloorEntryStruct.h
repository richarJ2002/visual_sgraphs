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
 * @file            SceneGraphFloorEntryStruct.h
 *
 * @brief           Declares SceneGraphFloorEntry, the plain-data copy of a
 *                  storey inside a SceneGraphSnapshot.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_FLOOR_ENTRY_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_FLOOR_ENTRY_STRUCT_H

#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

#include "scene_understanding/scene_graph/floor/objects/FloorClass.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           What the publisher, the archive and the health report need
 *                  to know about one storey, copied out under the graph lock.
 *                  Holds no pointer into the graph: rooms are ids.
 */
struct SceneGraphFloorEntry
{
  public:
    /*!
     * @brief           Storey id, unique within its map.
     */
    int id{-1};

    /*!
     * @brief           Display name of the storey.
     */
    std::string name;

    /*!
     * @brief           Storey centroid in the map's world frame, metres.
     */
    Eigen::Vector3d centroid_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Floor plane equation and its evidence quality; empty
     *                  when the storey has no floor plane yet.
     */
    std::optional<semantic::Floor::PlaneIdentity> planeIdentity;

    /*!
     * @brief           Ids of the rooms the storey lists, bad ones included.
     */
    std::vector<int> roomIds;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_FLOOR_ENTRY_STRUCT_H \
        */
