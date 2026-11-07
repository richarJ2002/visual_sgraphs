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
 * @file            SceneGraphClass.h
 *
 * @brief           Declares SceneGraph, the tree of maps, buildings, storeys,
 *                  rooms and passages held by the semantics manager.
 */

#ifndef VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_CLASS_H
#define VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_CLASS_H

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphStatusEnum.h"
#include "scene_understanding/scene_graph/SceneMap/objects/SceneMapStruct.h"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace atlas
{
class Atlas;
} /* namespace atlas */

namespace map
{
class Map;
} /* namespace map */

namespace semantic
{
class Floor;
class Passage;
class Room;
} /* namespace semantic */

namespace scene_graph
{

/*!
 * @brief           Scene graph of every SLAM map: map -> building -> storey
 *                  -> rooms and passages.
 *
 *                  The tree is a mirror: it is rebuilt from the map lists by
 *                  refreshFromAtlas() and borrows every room, storey and
 *                  passage from them; the map lists stay the owner and the
 *                  source of truth. Queries act on the active map. Not
 *                  thread-safe: refresh and queries run while the caller holds
 *                  the atlas semantic-update lock.
 */
class SceneGraph
{
  public:
    /*!
     * @brief           Creates an empty scene graph with no maps and no active
     *                  map.
     */
    SceneGraph() = default;

    /*!
     * @brief           Frees the scene maps and buildings; the borrowed SLAM
     *                  objects are not touched.
     */
    ~SceneGraph() = default;

    /*!
     * @brief           Copying is forbidden: the scene maps are owned
     *                  uniquely.
     */
    SceneGraph(const SceneGraph &otherSceneGraph_in) = delete;

    /*!
     * @brief           Copying is forbidden: the scene maps are owned
     *                  uniquely.
     */
    SceneGraph &operator=(const SceneGraph &otherSceneGraph_in) = delete;

    /*!
     * @brief           Rebuilds the whole tree from the atlas: one scene map
     *                  per active map, one building (id 0) per scene map
     *                  holding the map's storeys, and the map's detected and
     *                  candidate rooms and passages. Bad-flagged objects are
     *                  kept, as the map lists keep them.
     *
     * @param[in,out]   atlas_inout
     *                  Atlas to read; only its getters are called.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always; a failing getter is
     *                  logged and its part of the tree left empty.
     */
    [[nodiscard]] SceneGraphStatus refreshFromAtlas(atlas::Atlas &atlas_inout);

    /*!
     * @brief           Compares the number of maps, storeys, rooms and
     *                  passages in the tree with the atlas map lists.
     *
     *                  Counted as the lists hold them, bad-flagged objects
     *                  included; rooms are the distinct detected and
     *                  candidate rooms of a map.
     *
     * @param[in,out]   atlas_inout
     *                  Atlas to read; only its getters are called.
     *
     * @param[out]      mismatchReport_out
     *                  Empty when the tree agrees with the lists; otherwise
     *                  one entry per disagreeing count.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus
        verifyAgainstAtlas(atlas::Atlas &atlas_inout,
                           std::string  &mismatchReport_out) const;

    /*!
     * @brief           Looks up a room of the active map by id.
     *
     * @param[in]       roomId_in
     *                  Id of the room.
     *
     * @param[out]      p_room_out
     *                  The room, or nullptr when the active map has none with
     *                  that id; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus
        getRoomById(int roomId_in, semantic::Room *&p_room_out) const;

    /*!
     * @brief           Looks up a passage of the active map by id.
     *
     * @param[in]       passageId_in
     *                  Id of the passage.
     *
     * @param[out]      p_passage_out
     *                  The passage, or nullptr when the active map has none
     *                  with that id; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus
        getPassageById(int                 passageId_in,
                       semantic::Passage *&p_passage_out) const;

    /*!
     * @brief           Looks up a storey of the active map by id.
     *
     * @param[in]       floorId_in
     *                  Id of the storey.
     *
     * @param[out]      p_floor_out
     *                  The storey, or nullptr when the active map has none
     *                  with that id; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus
        getFloorById(int floorId_in, semantic::Floor *&p_floor_out) const;

    /*!
     * @brief           Lists the rooms of the active map in ascending id
     *                  order.
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when there is no active map.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus
        getAllRoomsInIdOrder(std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the passages of the active map in ascending id
     *                  order.
     *
     * @param[out]      passages_out
     *                  Borrowed passages; empty when there is no active map.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus getAllPassagesInIdOrder(
        std::vector<semantic::Passage *> &passages_out) const;

    /*!
     * @brief           Lists the storeys of the active map in ascending id
     *                  order, building by building.
     *
     * @param[out]      floors_out
     *                  Borrowed storeys; empty when there is no active map.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus
        getAllFloorsInIdOrder(std::vector<semantic::Floor *> &floors_out) const;

    /*!
     * @brief           Looks up the scene map of one SLAM map.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      p_sceneMap_out
     *                  The scene map, or nullptr when the tree has none for
     *                  that map; owned by this scene graph and valid until
     *                  the next refreshFromAtlas().
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus
        getSceneMap(unsigned long    mapId_in,
                    const SceneMap *&p_sceneMap_out) const;

  private:
    /*!
     * @brief           Scene maps by SLAM map id; owned. Keyed by the map's
     *                  own id type.
     */
    std::map<unsigned long, std::unique_ptr<SceneMap>> sceneMaps;

    /*!
     * @brief           Id of the atlas's current map at the last refresh;
     *                  empty when there was none.
     */
    std::optional<unsigned long> activeMapId;

    /*!
     * @brief           Returns the scene map of the active map.
     *
     * @param[out]      p_activeSceneMap_out
     *                  The active scene map, or nullptr when there is none;
     *                  owned by this scene graph.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus
        getActiveSceneMap(const SceneMap *&p_activeSceneMap_out) const;

    /*!
     * @brief           Fills the room and passage indexes of a scene map from
     *                  its SLAM map lists.
     *
     * @param[in,out]   map_inout
     *                  SLAM map to read; only its getters are called.
     *
     * @param[in,out]   sceneMap_inout
     *                  Scene map whose indexes are filled.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] static SceneGraphStatus
        mirrorMapIndexes(map::Map &map_inout, SceneMap &sceneMap_inout);

    /*!
     * @brief           Fills a building with the storeys of a SLAM map, the
     *                  rooms each storey lists and the passages those rooms
     *                  list.
     *
     * @param[in,out]   map_inout
     *                  SLAM map to read; only its getters are called.
     *
     * @param[out]      building_out
     *                  Building to fill; expected empty.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] static SceneGraphStatus
        mirrorMapBuilding(map::Map &map_inout, Building &building_out);
};

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_CLASS_H */
