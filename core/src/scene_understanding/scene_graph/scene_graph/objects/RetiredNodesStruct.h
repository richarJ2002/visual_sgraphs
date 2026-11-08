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
 * @file            RetiredNodesStruct.h
 *
 * @brief           Declares RetiredNodes, the rooms, passages and storeys
 *                  that left the scene graph but are not yet freed.
 */

#ifndef VS_GRAPHS_SCENE_GRAPH_RETIRED_NODES_STRUCT_H
#define VS_GRAPHS_SCENE_GRAPH_RETIRED_NODES_STRUCT_H

#include <memory>
#include <vector>

#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/Room.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

/*!
 * @brief           Nodes taken out of the tree and kept alive until
 *                  SceneGraph::sweepBadNodes() has unlinked every holder and
 *                  no reader guard is active. No query returns them. Owned by
 *                  SceneGraph and guarded by its graph mutex.
 */
struct RetiredNodes
{
  public:
    /*!
     * @brief           Retired rooms; owned.
     */
    std::vector<std::unique_ptr<semantic::Room>> rooms;

    /*!
     * @brief           Retired passages; owned.
     */
    std::vector<std::unique_ptr<semantic::Passage>> passages;

    /*!
     * @brief           Retired storeys; owned.
     */
    std::vector<std::unique_ptr<semantic::Floor>> floors;
};

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SCENE_GRAPH_RETIRED_NODES_STRUCT_H */
