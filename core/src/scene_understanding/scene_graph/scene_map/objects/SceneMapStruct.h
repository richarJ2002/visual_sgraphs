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
 * @file            SceneMapStruct.h
 *
 * @brief           Declares SceneMap, the scene-graph view of one SLAM map.
 */

#ifndef VS_GRAPHS_SCENE_GRAPH_SCENE_MAP_STRUCT_H
#define VS_GRAPHS_SCENE_GRAPH_SCENE_MAP_STRUCT_H

#include "scene_understanding/scene_graph/building/objects/BuildingStruct.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/Room.h"

#include <map>
#include <memory>

namespace vs_graphs
{
namespace core
{
namespace map
{
class Map;
} /* namespace map */

namespace scene_graph
{

/*!
 * @brief           The scene graph of one SLAM map: its buildings and id
 *                  indexes of its detected rooms, candidate rooms and
 *                  passages.
 *
 *                  This is the only list of the map's rooms, storeys and
 *                  passages. The scene map owns its rooms and passages and
 *                  its buildings own their storeys; nothing outside the tree
 *                  frees them. A room or passage not yet linked to a storey
 *                  is reachable through these maps only. Bad-flagged objects
 *                  stay here, hidden from every query, until the sweep moves
 *                  them out. Owned by SceneGraph and guarded by its graph
 *                  mutex.
 */
struct SceneMap
{
  public:
    /*!
     * @brief           SLAM map this scene map mirrors; borrowed, the atlas
     *                  owns it.
     */
    map::Map *p_map{nullptr};

    /*!
     * @brief           Buildings of this map by building id; owned. Holds
     *                  exactly one building, id 0.
     */
    std::map<int, std::unique_ptr<Building>> buildings;

    /*!
     * @brief           Detected rooms of the map by room id; owned.
     */
    std::map<int, std::unique_ptr<semantic::Room>> detectedRooms;

    /*!
     * @brief           Candidate rooms of the map by room id; owned.
     */
    std::map<int, std::unique_ptr<semantic::Room>> candidateRooms;

    /*!
     * @brief           Passages of the map by passage id; owned.
     */
    std::map<int, std::unique_ptr<semantic::Passage>> passages;

    /*!
     * @brief           Next storey id SceneGraph::addFloor() hands out when a
     *                  storey's id is negative or taken; never lowered, so a
     *                  freed id is not reused. Kept by clearSemantics().
     */
    int nextAvailableFloorId{0};
};

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SCENE_GRAPH_SCENE_MAP_STRUCT_H */
