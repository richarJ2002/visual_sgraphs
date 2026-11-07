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
 *                  rooms and passages held by the atlas.
 */

#ifndef VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_CLASS_H
#define VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_CLASS_H

#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphStatusEnum.h"
#include "scene_understanding/scene_graph/SceneMap/objects/SceneMapStruct.h"

#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
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
 *                  Owned by the atlas, which adds a scene map when it creates
 *                  or loads a map and removes it when it drops the map. Every
 *                  room, storey and passage is borrowed from the map lists,
 *                  which own them and stay the source of truth while writers
 *                  move onto the mutation API: each mutation writes the tree
 *                  and the matching map list.
 *
 *                  Thread safety: one graph-wide shared mutex. Every public
 *                  query takes it shared and every public mutation takes it
 *                  exclusive, inside the method; returned pointers are copied
 *                  out under the lock and stay valid while their map list
 *                  keeps the object. Public methods never call each other
 *                  while holding the lock. Lock order: atlasMutex, then this
 *                  lock, then Map::mapMutex and the semantic object mutexes;
 *                  no method takes atlasMutex while holding this lock.
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
     * @brief           Rebuilds the scene map of every atlas map from its map
     *                  lists (resyncMap() per map). Scene maps of maps the
     *                  atlas no longer lists are left to the atlas, which
     *                  erases them when it drops a map; the active map is
     *                  left as the atlas set it.
     *
     *                  Temporary: it keeps the tree equal to the map lists
     *                  while writers still bypass the mutation API, and goes
     *                  once every writer uses that API. Takes atlasMutex
     *                  (through the atlas getters) before this lock.
     *
     * @param[in,out]   atlas_inout
     *                  Atlas to read; only its getters are called.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always; a failing getter is
     *                  logged and its part of the tree left as it was.
     */
    [[nodiscard]] SceneGraphStatus refreshFromAtlas(atlas::Atlas &atlas_inout);

    /*!
     * @brief           Compares the number of maps, storeys, detected rooms,
     *                  candidate rooms and passages in the tree with the atlas
     *                  map lists.
     *
     *                  Counted as the lists hold them, bad-flagged objects
     *                  included. Takes atlasMutex (through the atlas getters)
     *                  before this lock, shared.
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
     * @brief           Rebuilds the scene map of one SLAM map from its map
     *                  lists, replacing any scene map it had: one building
     *                  (id 0) holding the map's storeys, plus its detected
     *                  rooms, candidate rooms and passages. Used by the atlas
     *                  when it creates or loads a map. Takes this lock
     *                  exclusive after reading the map lists.
     *
     * @param[in,out]   map_inout
     *                  SLAM map to mirror; only its getters are called. The
     *                  scene map borrows it; the atlas owns it.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always; a failing getter is
     *                  logged and its part of the scene map left empty.
     */
    [[nodiscard]] SceneGraphStatus resyncMap(map::Map &map_inout);

    /*!
     * @brief           Removes the scene map of a SLAM map the atlas dropped;
     *                  when it was the active map there is no active map
     *                  afterwards. Takes this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the dropped SLAM map; unknown ids are ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus eraseSceneMap(unsigned long mapId_in);

    /*!
     * @brief           Records which SLAM map the active-map queries read.
     *                  Takes this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the atlas's current map.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus setActiveMap(unsigned long mapId_in);

    /*!
     * @brief           Adds a detected room to a map, as
     *                  Map::addDetectedMapRoom does, then calls it. A room
     *                  whose id another room already holds in the tree is
     *                  listed by the map but not by the tree, and
     *                  verifyAgainstAtlas() reports it. Takes this lock
     *                  exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_room_in
     *                  Room to add; borrowed, the map list owns it. A null
     *                  room is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus addDetectedRoom(unsigned long   mapId_in,
                                                   semantic::Room *p_room_in);

    /*!
     * @brief           Adds a candidate room to a map, as
     *                  Map::addCandidateMapRoom does, then calls it. Id
     *                  clashes behave as in addDetectedRoom(). Takes this lock
     *                  exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_room_in
     *                  Room to add; borrowed, the map list owns it. A null
     *                  room is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus addCandidateRoom(unsigned long   mapId_in,
                                                    semantic::Room *p_room_in);

    /*!
     * @brief           Moves a room from the candidate rooms to the detected
     *                  rooms of a map, as Map::promoteCandidateMapRoom does,
     *                  then calls it. Takes this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_room_in
     *                  Room to promote; borrowed. A null room is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus
        promoteCandidateRoom(unsigned long mapId_in, semantic::Room *p_room_in);

    /*!
     * @brief           Removes a room from both the detected and the
     *                  candidate rooms of a map, then calls
     *                  Map::eraseDetectedMapRoom and
     *                  Map::eraseMarkerBasedMapRoom. The room is not deleted.
     *                  Takes this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_room_in
     *                  Room to remove; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus eraseRoom(unsigned long   mapId_in,
                                             semantic::Room *p_room_in);

    /*!
     * @brief           Adds a passage to a map, then calls
     *                  Map::addMapPassage. As there, a passage with a negative
     *                  id or an id another passage holds is rejected by both.
     *                  Takes this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_passage_in
     *                  Passage to add; borrowed, the map list owns it. A null
     *                  passage is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus addPassage(unsigned long      mapId_in,
                                              semantic::Passage *p_passage_in);

    /*!
     * @brief           Removes a passage from a map, then calls
     *                  Map::eraseMapPassage. The passage is not deleted. Takes
     *                  this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_passage_in
     *                  Passage to remove; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus
        erasePassage(unsigned long mapId_in, semantic::Passage *p_passage_in);

    /*!
     * @brief           Adds a storey to building 0 of a map by calling
     *                  Map::addMapFloor first, which gives the storey a free
     *                  id when its id is negative or taken, and then keys it
     *                  in the tree by the id it ends with. Takes this lock
     *                  exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in,out]   p_floor_inout
     *                  Storey to add; its id may be changed. Borrowed, the
     *                  map list owns it. A null storey is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus addFloor(unsigned long    mapId_in,
                                            semantic::Floor *p_floor_inout);

    /*!
     * @brief           Removes a storey from a map, then calls
     *                  Map::eraseMapFloor. The storey is not deleted. Takes
     *                  this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_floor_in
     *                  Storey to remove; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus eraseFloor(unsigned long    mapId_in,
                                              semantic::Floor *p_floor_in);

    /*!
     * @brief           Empties the storeys, rooms and passages of a map, as
     *                  Map::clear does for its semantic lists. Map::clear is
     *                  not called (it also clears key frames and map points);
     *                  the caller clears the map itself. Takes this lock
     *                  exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus clearSemantics(unsigned long mapId_in);

    /*!
     * @brief           Lists the detected and candidate rooms of a map in
     *                  ascending id order; on an id held by both, the
     *                  detected room. Takes this lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when the map is unknown.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus
        getMapRoomsInIdOrder(unsigned long                  mapId_in,
                             std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the detected rooms of a map in ascending id
     *                  order. Takes this lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when the map is unknown.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus getMapDetectedRoomsInIdOrder(
        unsigned long                  mapId_in,
        std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the candidate rooms of a map in ascending id
     *                  order. Takes this lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when the map is unknown.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus getMapCandidateRoomsInIdOrder(
        unsigned long                  mapId_in,
        std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the passages of a map in ascending id order.
     *                  Takes this lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      passages_out
     *                  Borrowed passages; empty when the map is unknown.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus getMapPassagesInIdOrder(
        unsigned long                     mapId_in,
        std::vector<semantic::Passage *> &passages_out) const;

    /*!
     * @brief           Lists the storeys of a map in ascending id order,
     *                  building by building. Takes this lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[out]      floors_out
     *                  Borrowed storeys; empty when the map is unknown.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus
        getMapFloorsInIdOrder(unsigned long                   mapId_in,
                              std::vector<semantic::Floor *> &floors_out) const;

    /*!
     * @brief           Looks up a detected or candidate room of a map by id;
     *                  the detected room when both hold the id. Takes this
     *                  lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       roomId_in
     *                  Id of the room.
     *
     * @param[out]      p_room_out
     *                  The room, or nullptr when there is none; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus
        getMapRoomById(unsigned long    mapId_in,
                       int              roomId_in,
                       semantic::Room *&p_room_out) const;

    /*!
     * @brief           Looks up a passage of a map by id. Takes this lock
     *                  shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       passageId_in
     *                  Id of the passage.
     *
     * @param[out]      p_passage_out
     *                  The passage, or nullptr when there is none; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus
        getMapPassageById(unsigned long       mapId_in,
                          int                 passageId_in,
                          semantic::Passage *&p_passage_out) const;

    /*!
     * @brief           Looks up a storey of a map by id. Takes this lock
     *                  shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       floorId_in
     *                  Id of the storey.
     *
     * @param[out]      p_floor_out
     *                  The storey, or nullptr when there is none; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus
        getMapFloorById(unsigned long     mapId_in,
                        int               floorId_in,
                        semantic::Floor *&p_floor_out) const;

    /*!
     * @brief           Lists the rooms of one storey in ascending id order,
     *                  read from the storey's own room list at call time.
     *                  Takes this lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       floorId_in
     *                  Id of the storey.
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when the map or storey is
     *                  unknown.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus
        getFloorRoomsInIdOrder(unsigned long                  mapId_in,
                               int                            floorId_in,
                               std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the passages of one storey in ascending id
     *                  order: the passages its rooms list at call time. A
     *                  passage between rooms of two storeys is listed by
     *                  both. Takes this lock shared.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       floorId_in
     *                  Id of the storey.
     *
     * @param[out]      passages_out
     *                  Borrowed passages; empty when the map or storey is
     *                  unknown.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id.
     */
    [[nodiscard]] SceneGraphStatus getFloorPassagesInIdOrder(
        unsigned long                     mapId_in,
        int                               floorId_in,
        std::vector<semantic::Passage *> &passages_out) const;

    /*!
     * @brief           Looks up a room of the active map by id, as
     *                  getMapRoomById().
     *
     * @param[in]       roomId_in
     *                  Id of the room.
     *
     * @param[out]      p_room_out
     *                  The room, or nullptr when there is no active map or no
     *                  room with that id; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS when there is no active map,
     *                  otherwise as getMapRoomById().
     */
    [[nodiscard]] SceneGraphStatus
        getRoomById(int roomId_in, semantic::Room *&p_room_out) const;

    /*!
     * @brief           Looks up a passage of the active map by id, as
     *                  getMapPassageById().
     *
     * @param[in]       passageId_in
     *                  Id of the passage.
     *
     * @param[out]      p_passage_out
     *                  The passage, or nullptr when there is no active map or
     *                  no passage with that id; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS when there is no active map,
     *                  otherwise as getMapPassageById().
     */
    [[nodiscard]] SceneGraphStatus
        getPassageById(int                 passageId_in,
                       semantic::Passage *&p_passage_out) const;

    /*!
     * @brief           Looks up a storey of the active map by id, as
     *                  getMapFloorById().
     *
     * @param[in]       floorId_in
     *                  Id of the storey.
     *
     * @param[out]      p_floor_out
     *                  The storey, or nullptr when there is no active map or
     *                  no storey with that id; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS when there is no active map,
     *                  otherwise as getMapFloorById().
     */
    [[nodiscard]] SceneGraphStatus
        getFloorById(int floorId_in, semantic::Floor *&p_floor_out) const;

    /*!
     * @brief           Lists the rooms of the active map in ascending id
     *                  order, as getMapRoomsInIdOrder().
     *
     * @param[out]      rooms_out
     *                  Borrowed rooms; empty when there is no active map.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS when there is no active map,
     *                  otherwise as getMapRoomsInIdOrder().
     */
    [[nodiscard]] SceneGraphStatus
        getAllRoomsInIdOrder(std::vector<semantic::Room *> &rooms_out) const;

    /*!
     * @brief           Lists the passages of the active map in ascending id
     *                  order, as getMapPassagesInIdOrder().
     *
     * @param[out]      passages_out
     *                  Borrowed passages; empty when there is no active map.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS when there is no active map,
     *                  otherwise as getMapPassagesInIdOrder().
     */
    [[nodiscard]] SceneGraphStatus getAllPassagesInIdOrder(
        std::vector<semantic::Passage *> &passages_out) const;

    /*!
     * @brief           Lists the storeys of the active map in ascending id
     *                  order, as getMapFloorsInIdOrder().
     *
     * @param[out]      floors_out
     *                  Borrowed storeys; empty when there is no active map.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS when there is no active map,
     *                  otherwise as getMapFloorsInIdOrder().
     */
    [[nodiscard]] SceneGraphStatus
        getAllFloorsInIdOrder(std::vector<semantic::Floor *> &floors_out) const;

  private:
    /*!
     * @brief           Guards sceneMaps, activeMapId and everything the scene
     *                  maps hold. Shared for queries, exclusive for
     *                  mutations.
     */
    mutable std::shared_mutex graphMutex;

    /*!
     * @brief           Scene maps by SLAM map id; owned. Keyed by the map's
     *                  own id type. Guarded by graphMutex.
     */
    std::map<unsigned long, std::unique_ptr<SceneMap>> sceneMaps;

    /*!
     * @brief           Id of the atlas's current map; empty when there is
     *                  none. Guarded by graphMutex.
     */
    std::optional<unsigned long> activeMapId;
};

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_CLASS_H */
