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
 * @file            BuildingStruct.h
 *
 * @brief           Declares Building, the scene-graph level that holds the
 *                  storeys of one building.
 */

#ifndef VS_GRAPHS_SCENE_GRAPH_BUILDING_STRUCT_H
#define VS_GRAPHS_SCENE_GRAPH_BUILDING_STRUCT_H

#include <map>

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

namespace scene_graph
{

/*!
 * @brief           One building of a scene map: its storeys, the rooms and
 *                  passages of each storey, and the passages that join rooms
 *                  of different storeys.
 *
 *                  Every pointer is borrowed: the SLAM map lists own the
 *                  objects and stay the source of truth. Containers are keyed
 *                  by id so iteration runs in id order. Written by
 *                  SceneGraph::refreshFromAtlas() and read only while the
 *                  atlas semantic-update lock is held.
 */
struct Building
{
  public:
    /*!
     * @brief           Storeys of this building by floor id; borrowed.
     */
    std::map<int, semantic::Floor *> floors;

    /*!
     * @brief           Rooms of each storey, by floor id and then room id,
     *                  taken from the floor's own room list; borrowed.
     */
    std::map<int, std::map<int, semantic::Room *>> floorRooms;

    /*!
     * @brief           Passages of each storey, by floor id and then passage
     *                  id, taken from the passage lists of the storey's
     *                  rooms; borrowed.
     */
    std::map<int, std::map<int, semantic::Passage *>> floorPassages;

    /*!
     * @brief           Passages joining rooms of different storeys
     *                  (staircases) by passage id; borrowed. Empty until such
     *                  passages are detected.
     */
    std::map<int, semantic::Passage *> passages;
};

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SCENE_GRAPH_BUILDING_STRUCT_H */
