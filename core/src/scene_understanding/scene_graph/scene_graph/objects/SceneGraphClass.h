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

#include "scene_understanding/scene_graph/scene_graph/objects/RetiredNodesStruct.h"
#include "scene_understanding/scene_graph/scene_graph/objects/RoomListEnum.h"
#include "scene_understanding/scene_graph/scene_graph/objects/SceneGraphStatusEnum.h"
#include "scene_understanding/scene_graph/scene_map/objects/SceneMapStruct.h"

#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace map
{
class Map;
} /* namespace map */

namespace geometric
{
class Plane;
} /* namespace geometric */

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
 *                  Owned by the atlas, which adds an empty scene map when it
 *                  creates or loads a map and removes it when it drops the
 *                  map. The tree is the only list of each map's rooms,
 *                  storeys and passages and it owns them: a scene map owns
 *                  its rooms and passages, a building owns its storeys. They
 *                  are added and removed through the mutations only, which
 *                  also enforce the id rules.
 *
 *                  Deletion. markRoomBad() flags a node; every query hides a
 *                  flagged node, so to readers it is gone at once. Nothing is
 *                  freed on the spot: sweepBadNodes(), called once at the end
 *                  of each semantics cycle, retires flagged nodes, unlinks
 *                  every holder and frees the retired nodes when no reader
 *                  holds the reclaim guard. Planes belong to the SLAM map and
 *                  are only unlinked.
 *
 *                  Thread safety: one graph-wide shared mutex. Every public
 *                  query takes it shared and every public mutation takes it
 *                  exclusive, inside the method; returned pointers are copied
 *                  out under the lock. A caller that uses them outside the
 *                  semantic update lock holds the reclaim guard
 *                  (acquireReclaimGuard) for as long as it uses them, which
 *                  keeps the sweep from freeing the nodes. Public methods
 *                  never call each other while holding the lock. Lock order:
 *                  atlasMutex, then this lock, then the semantic object
 *                  mutexes; no method takes atlasMutex or Map::mapMutex while
 *                  holding this lock. The reclaim mutex is only ever
 *                  try-locked exclusive, so it cannot block.
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
     * @brief           Frees the scene maps, their rooms, passages and
     *                  storeys and the retired nodes; the borrowed SLAM
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
     * @brief           Adds an empty scene map for a SLAM map: one building
     *                  (id 0) and no storeys, rooms or passages, replacing any
     *                  scene map it had (that map's nodes are retired for the
     *                  next sweep, not freed here). Used by the atlas when it
     *                  creates or loads a map. Takes this lock exclusive after
     *                  reading the map id.
     *
     * @param[in,out]   map_inout
     *                  SLAM map the scene map belongs to; only its id is
     *                  read. The scene map borrows it; the atlas owns it.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus addSceneMap(map::Map &map_inout);

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
     * @brief           Adds a new candidate room to a map, keyed by its id,
     *                  and takes ownership of it. A room whose id another
     *                  candidate room already holds is not added (a warning
     *                  is logged); it is retired instead, so a raw pointer
     *                  the caller kept stays valid until the next sweep. Takes
     *                  this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_room_in
     *                  Room to add; ownership moves to the tree, also when
     *                  the map is unknown (the room is then retired). A null
     *                  room is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (the room is retired).
     */
    [[nodiscard]] SceneGraphStatus
        addCandidateRoom(unsigned long                   mapId_in,
                         std::unique_ptr<semantic::Room> p_room_in);

    /*!
     * @brief           Moves a room from the candidate rooms to the detected
     *                  rooms of a map; when its id is already held by another
     *                  detected room it is retired. Takes this lock
     *                  exclusive.
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
     * @brief           Takes a room out of the tree, by pointer, whatever id
     *                  it is keyed under, and retires it: it stays alive but
     *                  no query returns it, and the next sweep unlinks and
     *                  frees it. Takes this lock exclusive.
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
     * @brief           Moves a room, with its ownership, from one scene map
     *                  to a list of another (or the same) scene map, keyed
     *                  by the id the room holds now. This is how merge hands
     *                  rooms over and how a room is re-keyed after a new id.
     *                  A room whose id the destination list already holds is
     *                  retired. Takes this lock exclusive; the move is one
     *                  step, so no reader sees the room missing.
     *
     * @param[in]       sourceMapId_in
     *                  Id of the SLAM map that holds the room.
     *
     * @param[in]       destinationMapId_in
     *                  Id of the SLAM map that receives it.
     *
     * @param[in]       p_room_in
     *                  Room to move; borrowed, found by pointer in either
     *                  list of the source map. A null room is ignored.
     *
     * @param[in]       destinationList_in
     *                  Room list of the destination map.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when either scene map
     *                  is missing or the source map does not hold the room
     *                  (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus moveRoom(unsigned long   sourceMapId_in,
                                            unsigned long   destinationMapId_in,
                                            semantic::Room *p_room_in,
                                            RoomList        destinationList_in);

    /*!
     * @brief           Adds a new passage to a map, keyed by its id, and
     *                  takes ownership of it. A passage with a negative id or
     *                  an id another passage holds is not added (a message is
     *                  printed): it is retired, so a raw pointer the caller
     *                  kept stays valid until the next sweep, and the status
     *                  is still success, so the caller must resolve the id
     *                  first. Takes this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_passage_in
     *                  Passage to add; ownership moves to the tree. A null
     *                  passage is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (the passage is retired).
     */
    [[nodiscard]] SceneGraphStatus
        addPassage(unsigned long                      mapId_in,
                   std::unique_ptr<semantic::Passage> p_passage_in);

    /*!
     * @brief           Takes a passage out of the tree, by pointer, and
     *                  retires it (see eraseRoom). Takes this lock exclusive.
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
     * @brief           Moves a passage, with its ownership, from one scene
     *                  map to another (or re-keys it in the same one) under
     *                  the id it holds now. A passage whose id the
     *                  destination already holds is retired. Takes this lock
     *                  exclusive.
     *
     * @param[in]       sourceMapId_in
     *                  Id of the SLAM map that holds the passage.
     *
     * @param[in]       destinationMapId_in
     *                  Id of the SLAM map that receives it.
     *
     * @param[in]       p_passage_in
     *                  Passage to move; borrowed. A null passage is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when either scene map
     *                  is missing or the source map does not hold the passage
     *                  (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus
        movePassage(unsigned long      sourceMapId_in,
                    unsigned long      destinationMapId_in,
                    semantic::Passage *p_passage_in);

    /*!
     * @brief           Adds a new storey to building 0 of a map, keyed by its
     *                  id, and takes ownership of it. A storey whose id is
     *                  negative or held by another storey of the map is given
     *                  the next free id (a message is printed); an entry for
     *                  the storey under an older id is dropped. Ids handed out
     *                  are never reused. Takes this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map.
     *
     * @param[in]       p_floor_in
     *                  Storey to add; its id may be changed. Ownership moves
     *                  to the tree, also when the map is unknown (the storey
     *                  is then retired). A null storey is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (the storey is retired).
     */
    [[nodiscard]] SceneGraphStatus
        addFloor(unsigned long                    mapId_in,
                 std::unique_ptr<semantic::Floor> p_floor_in);

    /*!
     * @brief           Takes a storey out of every building of a map, by
     *                  pointer, and retires it (see eraseRoom). Takes this
     *                  lock exclusive.
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
     * @brief           Moves a storey, with its ownership, from building 0 of
     *                  one scene map to building 0 of another, under the id
     *                  it holds now; an id the destination holds gives it the
     *                  next free id there (as addFloor does). Takes this lock
     *                  exclusive.
     *
     * @param[in]       sourceMapId_in
     *                  Id of the SLAM map that holds the storey.
     *
     * @param[in]       destinationMapId_in
     *                  Id of the SLAM map that receives it.
     *
     * @param[in]       p_floor_in
     *                  Storey to move; borrowed. A null storey is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when either scene map
     *                  is missing or the source map does not hold the storey
     *                  (nothing is written).
     */
    [[nodiscard]] SceneGraphStatus moveFloor(unsigned long sourceMapId_in,
                                             unsigned long destinationMapId_in,
                                             semantic::Floor *p_floor_in);

    /*!
     * @brief           Flags a room invalid and everything that depends on
     *                  it, in one step, so no reader sees a half-deleted
     *                  room. The room is hidden from every query at once.
     *                  Its passages are flagged too when no other live room
     *                  of the map lists them or is their known or prospective
     *                  side; a passage that still leads to a live room stays.
     *                  The room's planes are not touched here: the sweep
     *                  clears the room's plane lists (planes are never freed
     *                  by the sweep). Works for a room that is already
     *                  retired. Takes this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map the room belongs to.
     *
     * @param[in,out]   p_room_inout
     *                  Room to flag; borrowed. A null room is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (the room is still flagged).
     */
    [[nodiscard]] SceneGraphStatus markRoomBad(unsigned long   mapId_in,
                                               semantic::Room *p_room_inout);

    /*!
     * @brief           Flags a storey invalid and every room on it (see
     *                  markRoomBad). Takes this lock exclusive.
     *
     * @param[in]       mapId_in
     *                  Id of the SLAM map the storey belongs to.
     *
     * @param[in,out]   p_floor_inout
     *                  Storey to flag; borrowed. A null storey is ignored.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS, or
     *                  SCENE_GRAPH_STATUS_MAP_NOT_FOUND when the tree has no
     *                  scene map for the id (the storey is still flagged).
     */
    [[nodiscard]] SceneGraphStatus markFloorBad(unsigned long    mapId_in,
                                                semantic::Floor *p_floor_inout);

    /*!
     * @brief           Points every room and passage of the active map that
     *                  listed a retired plane at the plane that replaces it,
     *                  both sides written in one step. Takes this lock
     *                  exclusive.
     *
     * @param[in,out]   p_retiredPlane_inout
     *                  Plane being fused away; borrowed.
     *
     * @param[in,out]   p_retainedPlane_inout
     *                  Plane that survives; borrowed.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always (no active map or an
     *                  unlisted plane changes nothing).
     */
    [[nodiscard]] SceneGraphStatus
        replacePlane(geometric::Plane *p_retiredPlane_inout,
                     geometric::Plane *p_retainedPlane_inout);

    /*!
     * @brief           Ends a cycle's deletion: moves every bad room,
     *                  passage and storey out of the tree, unlinks every
     *                  retired node from the nodes that still list it (the
     *                  storeys' room lists, the passages' room links, the
     *                  rooms' passage lists, the maps' linked rooms and the
     *                  rooms' own plane lists) and frees the retired nodes
     *                  when no reader holds the reclaim guard. Planes are
     *                  unlinked only, never freed. A node that cannot be
     *                  freed now stays retired and is freed by a later sweep.
     *                  Call it only from the cycle owner that holds the
     *                  semantic update lock. Takes this lock exclusive for
     *                  short steps, never while it calls into a SLAM map.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus sweepBadNodes();

    /*!
     * @brief           Empties the storeys, rooms and passages of a map by
     *                  retiring them; the buildings and the next free storey
     *                  id stay. Map::clear is not called (it clears key
     *                  frames and map points); the caller clears the map
     *                  itself. Takes this lock exclusive.
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
     * @brief           Takes the reclaim guard shared. While a reader holds
     *                  it, the sweep frees nothing, so every room, passage and
     *                  storey pointer the reader obtained from a query stays
     *                  valid. Every reader that keeps such a pointer across
     *                  calls without holding the semantic update lock takes
     *                  the guard before its first query and releases it after
     *                  its last use of the pointers. The sweep only try-locks
     *                  the reclaim mutex, so holding the guard never blocks
     *                  the cycle and a function that holds it may call
     *                  another that takes it again.
     *
     * @param[out]      reclaimGuard_out
     *                  Shared lock on the reclaim mutex; it unlocks when it
     *                  goes out of scope.
     *
     * @return          SCENE_GRAPH_STATUS_SUCCESS always.
     */
    [[nodiscard]] SceneGraphStatus acquireReclaimGuard(
        std::shared_lock<std::shared_mutex> &reclaimGuard_out) const;

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
     * @brief           Scene maps by SLAM map id; borrowed. Each SLAM map
     *                  owns its scene map as a member and is registered here
     *                  by addSceneMap() until eraseSceneMap(). Guarded by
     *                  graphMutex.
     */
    std::map<unsigned long, SceneMap *> sceneMaps;

    /*!
     * @brief           Id of the atlas's current map; empty when there is
     *                  none. Guarded by graphMutex.
     */
    std::optional<unsigned long> activeMapId;

    /*!
     * @brief           Nodes that left the tree and wait to be freed by the
     *                  sweep. Guarded by graphMutex.
     */
    RetiredNodes retiredNodes;

    /*!
     * @brief           The reclaim guard: readers that keep node pointers
     *                  outside the semantic update lock hold it shared, and
     *                  the sweep frees only while it can take it exclusive
     *                  without waiting.
     */
    mutable std::shared_mutex reclaimMutex;
};

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_CLASS_H */
