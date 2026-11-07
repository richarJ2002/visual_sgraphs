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
 * @brief           Declares the helpers shared by the SceneGraph query,
 *                  mutation and lifecycle methods. Package-internal: not
 *                  included by any public header.
 */

#ifndef VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_PRIVATE_FUNCTIONS_H

#include "scene_understanding/scene_graph/SceneMap/objects/SceneMapStruct.h"

#include <map>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
class Floor;
} /* namespace semantic */

namespace scene_graph
{

/*!
 * @brief           Looks up a storey by id across every building of a scene
 *                  map, buildings in id order. Does not lock: the caller
 *                  holds the SceneGraph graph mutex.
 *
 * @param[in]       sceneMap_in
 *                  Scene map to search.
 *
 * @param[in]       floorId_in
 *                  Id of the storey.
 *
 * @return          The first storey keyed by that id, or nullptr when no
 *                  building has one; borrowed.
 */
[[nodiscard]] semantic::Floor *findFloorInBuildings(const SceneMap &sceneMap_in,
                                                    int             floorId_in);

/*!
 * @brief           Removes every entry of an id-keyed index that points at
 *                  one object, whatever key it is under, so an object whose
 *                  id changed after it was keyed still leaves. Does not
 *                  lock: the caller holds the SceneGraph graph mutex.
 *
 * @tparam          EntryType
 *                  Type of the indexed objects (Room, Passage, Floor).
 *
 * @param[in,out]   entriesById_inout
 *                  Index to remove the object from; borrowed pointers.
 *
 * @param[in]       p_entry_in
 *                  Object to remove; borrowed, not deleted.
 */
template <class EntryType>
void eraseEntriesByPointer(std::map<int, EntryType *> &entriesById_inout,
                           const EntryType            *p_entry_in);

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#include "scene_understanding/scene_graph/SceneGraph/private_functions/eraseEntriesByPointer.tpp"

#endif /* VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_PRIVATE_FUNCTIONS_H */
