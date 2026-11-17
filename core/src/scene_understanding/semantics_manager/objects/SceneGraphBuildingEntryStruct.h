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

/*!
 * @file            SceneGraphBuildingEntryStruct.h
 *
 * @brief           Declares SceneGraphBuildingEntry, the plain-data copy of a
 *                  building inside a SceneGraphSnapshot.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_BUILDING_ENTRY_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_BUILDING_ENTRY_STRUCT_H

#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           What the archive needs to know about one building, copied
 *                  out under the graph lock. Holds no pointer into the graph:
 *                  storeys are ids.
 */
struct SceneGraphBuildingEntry
{
  public:
    /*!
     * @brief           Building id, unique within its map.
     */
    int id{-1};

    /*!
     * @brief           Ids of the live storeys the building holds, in
     *                  ascending id order.
     */
    std::vector<int> floorIds;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_BUILDING_ENTRY_STRUCT_H \
        */
