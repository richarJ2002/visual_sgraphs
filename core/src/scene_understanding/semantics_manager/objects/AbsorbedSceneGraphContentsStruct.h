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
 * @file            AbsorbedSceneGraphContentsStruct.h
 *
 * @brief           Declares AbsorbedSceneGraphContents, the nodes a map merge
 *                  took over from the absorbed map's scene graph.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_ABSORBED_SCENE_GRAPH_CONTENTS_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_ABSORBED_SCENE_GRAPH_CONTENTS_STRUCT_H

#include <vector>

namespace vs_graphs
{
namespace core
{

namespace semantic
{
class Floor;
class Passage;
class Room;
} /* namespace semantic */

/*!
 * @brief           The passages, rooms and storeys the absorbed map's scene
 *                  graph held when a merge started. Filled by
 *                  SemanticsManager::transferSceneGraphContents() and handed
 *                  back to SemanticsManager::reassociateMergedSceneGraph(),
 *                  which tells the imported nodes from the surviving map's own.
 *                  Callers only pass it back: they never read the nodes. The
 *                  pointers are borrowed from the graph that now owns the
 *                  nodes, so they are valid only while the merge holds both
 *                  graph locks and nothing sweeps the graph.
 */
struct AbsorbedSceneGraphContents
{
    /*!
     * @brief           Passages of the absorbed map, in id order.
     */
    std::vector<semantic::Passage *> passages;

    /*!
     * @brief           Detected rooms of the absorbed map, in id order.
     */
    std::vector<semantic::Room *> detectedRooms;

    /*!
     * @brief           Candidate rooms of the absorbed map, in id order.
     */
    std::vector<semantic::Room *> candidateRooms;

    /*!
     * @brief           Storeys of the absorbed map, in id order.
     */
    std::vector<semantic::Floor *> floors;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_ABSORBED_SCENE_GRAPH_CONTENTS_STRUCT_H \
        */
