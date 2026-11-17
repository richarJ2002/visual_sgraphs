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
 * @file            SceneGraphSnapshotStruct.h
 *
 * @brief           Declares SceneGraphSnapshot, the plain-data copy of one
 *                  map's scene graph that code outside the semantics manager
 *                  works with.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_SNAPSHOT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_SNAPSHOT_STRUCT_H

#include <vector>

#include "scene_understanding/scene_graph/environment/objects/EnvironmentStruct.h"
#include "scene_understanding/semantics_manager/objects/SceneGraphBuildingEntryStruct.h"
#include "scene_understanding/semantics_manager/objects/SceneGraphFloorEntryStruct.h"
#include "scene_understanding/semantics_manager/objects/SceneGraphPassageEntryStruct.h"
#include "scene_understanding/semantics_manager/objects/SceneGraphRoomEntryStruct.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           The environment, buildings, live storeys, rooms and
 *                  passages of one map, each list in ascending id order,
 *                  copied out under the map's graph lock by
 *                  SemanticsManager::getSceneGraphSnapshot(). Holds no pointer
 *                  into the graph, so it stays valid after the lock is
 *                  released. Thread safety: a plain value owned by its caller.
 */
struct SceneGraphSnapshot
{
  public:
    /*!
     * @brief           Id of the SLAM map the graph belongs to.
     */
    unsigned long mapId{0UL};

    /*!
     * @brief           Top node of the map's graph (copy).
     */
    scene_graph::Environment environment;

    /*!
     * @brief           Buildings of the map, empty until the first one is
     *                  created.
     */
    std::vector<SceneGraphBuildingEntry> buildings;

    /*!
     * @brief           Storeys of the map.
     */
    std::vector<SceneGraphFloorEntry> floors;

    /*!
     * @brief           Detected and candidate rooms of the map; on an id held
     *                  by both, the detected room.
     */
    std::vector<SceneGraphRoomEntry> rooms;

    /*!
     * @brief           Passages of the map.
     */
    std::vector<SceneGraphPassageEntry> passages;

    /*!
     * @brief           Looks a room up by id.
     *
     * @param[in]       roomId_in
     *                  Id of the room.
     *
     * @return          The room inside this snapshot, or null when the
     *                  snapshot has none with that id. Valid while the
     *                  snapshot is neither changed nor destroyed.
     */
    [[nodiscard]] const SceneGraphRoomEntry *findRoom(int roomId_in) const;

    /*!
     * @brief           Looks a passage up by id.
     *
     * @param[in]       passageId_in
     *                  Id of the passage.
     *
     * @return          The passage inside this snapshot, or null when the
     *                  snapshot has none with that id. Valid while the
     *                  snapshot is neither changed nor destroyed.
     */
    [[nodiscard]] const SceneGraphPassageEntry *
        findPassage(int passageId_in) const;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_SNAPSHOT_STRUCT_H \
        */
