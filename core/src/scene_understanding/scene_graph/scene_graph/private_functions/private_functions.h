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

#include "scene_understanding/scene_graph/scene_graph/objects/RetiredNodesStruct.h"
#include "scene_understanding/scene_graph/scene_map/objects/SceneMapStruct.h"

#include <map>
#include <memory>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

/*!
 * @brief           Looks up a storey by id across every building of a scene
 *                  map, buildings in id order. Finds flagged storeys too.
 *                  Does not lock: the caller holds the SceneGraph graph
 *                  mutex.
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
 * @brief           Tells whether a room is usable: not null and not flagged
 *                  bad. Every query filters with it, so a flagged room is
 *                  gone for readers at once.
 *
 * @param[in]       p_room_in
 *                  Room to test; borrowed.
 *
 * @return          True for a non-null room that is not flagged bad.
 */
[[nodiscard]] bool isRoomLive(semantic::Room *p_room_in);

/*!
 * @brief           Tells whether a passage is usable: not null and not
 *                  flagged bad.
 *
 * @param[in]       p_passage_in
 *                  Passage to test; borrowed.
 *
 * @return          True for a non-null passage that is not flagged bad.
 */
[[nodiscard]] bool isPassageLive(semantic::Passage *p_passage_in);

/*!
 * @brief           Tells whether a storey is usable: not null and not flagged
 *                  bad.
 *
 * @param[in]       p_floor_in
 *                  Storey to test; borrowed.
 *
 * @return          True for a non-null storey that is not flagged bad.
 */
[[nodiscard]] bool isFloorLive(semantic::Floor *p_floor_in);

/*!
 * @brief           Removes every entry of an id-keyed index of borrowed
 *                  pointers that points at one object, whatever key it is
 *                  under. Does not lock: the caller holds the SceneGraph
 *                  graph mutex.
 *
 * @tparam          EntryType
 *                  Type of the indexed objects.
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

/*!
 * @brief           Moves every entry of an owning id-keyed index that holds
 *                  one object into a retired list, whatever key it is under,
 *                  so an object whose id changed after it was keyed still
 *                  leaves. Does not lock: the caller holds the SceneGraph
 *                  graph mutex.
 *
 * @tparam          EntryType
 *                  Type of the indexed objects (Room, Passage, Floor).
 *
 * @param[in,out]   entriesById_inout
 *                  Index to take the object out of.
 *
 * @param[in]       p_entry_in
 *                  Object to retire; borrowed.
 *
 * @param[in,out]   retiredEntries_inout
 *                  List that receives the object's ownership.
 */
template <class EntryType>
void retireEntryByPointer(
    std::map<int, std::unique_ptr<EntryType>> &entriesById_inout,
    const EntryType                           *p_entry_in,
    std::vector<std::unique_ptr<EntryType>>   &retiredEntries_inout);

/*!
 * @brief           Takes one object, with its ownership, out of an owning
 *                  id-keyed index, whatever key it is under. Does not lock:
 *                  the caller holds the SceneGraph graph mutex.
 *
 * @tparam          EntryType
 *                  Type of the indexed objects.
 *
 * @param[in,out]   entriesById_inout
 *                  Index to take the object out of.
 *
 * @param[in]       p_entry_in
 *                  Object to extract; borrowed.
 *
 * @return          The object's ownership, or nullptr when the index does
 *                  not hold it.
 */
template <class EntryType>
[[nodiscard]] std::unique_ptr<EntryType> extractEntryByPointer(
    std::map<int, std::unique_ptr<EntryType>> &entriesById_inout,
    const EntryType                           *p_entry_in);

/*!
 * @brief           Moves every entry of an owning id-keyed index that the
 *                  predicate calls not live into a retired list. Does not
 *                  lock: the caller holds the SceneGraph graph mutex.
 *
 * @tparam          EntryType
 *                  Type of the indexed objects.
 *
 * @param[in,out]   entriesById_inout
 *                  Index to take dead objects out of.
 *
 * @param[in]       isLive_in
 *                  Predicate, such as isRoomLive; must not be null.
 *
 * @param[in,out]   retiredEntries_inout
 *                  List that receives the dead objects' ownership.
 */
template <class EntryType>
void retireDeadEntries(
    std::map<int, std::unique_ptr<EntryType>> &entriesById_inout,
    bool (*isLive_in)(EntryType *),
    std::vector<std::unique_ptr<EntryType>> &retiredEntries_inout);

/*!
 * @brief           Adds a room to a room list under the id it holds now. A
 *                  room whose id the list already holds is retired instead.
 *                  Does not lock: the caller holds the SceneGraph graph
 *                  mutex.
 *
 * @param[in,out]   roomsById_inout
 *                  Room list to add to.
 *
 * @param[in]       p_room_in
 *                  Room to add; ownership moves. A null room is ignored.
 *
 * @param[in]       mapId_in
 *                  Id of the SLAM map, for the warning.
 *
 * @param[in,out]   retiredNodes_inout
 *                  Receives a room that could not be added.
 */
void insertRoom(std::map<int, std::unique_ptr<semantic::Room>> &roomsById_inout,
                std::unique_ptr<semantic::Room>                 p_room_in,
                unsigned long                                   mapId_in,
                RetiredNodes &retiredNodes_inout);

/*!
 * @brief           Adds a passage to a scene map under the id it holds now. A
 *                  passage with a negative id or an id the map already holds
 *                  is retired instead and a message is printed. Does not
 *                  lock: the caller holds the SceneGraph graph mutex.
 *
 * @param[in,out]   sceneMap_inout
 *                  Scene map to add to.
 *
 * @param[in]       p_passage_in
 *                  Passage to add; ownership moves. A null passage is
 *                  ignored.
 *
 * @param[in,out]   retiredNodes_inout
 *                  Receives a passage that could not be added.
 */
void insertPassage(SceneMap                          &sceneMap_inout,
                   std::unique_ptr<semantic::Passage> p_passage_in,
                   RetiredNodes                      &retiredNodes_inout);

/*!
 * @brief           Adds a storey to building 0 of a scene map. A storey whose
 *                  id is negative or already held in the map is given the
 *                  next free id (a message is printed). Does not lock: the
 *                  caller holds the SceneGraph graph mutex.
 *
 * @param[in,out]   sceneMap_inout
 *                  Scene map to add to.
 *
 * @param[in]       p_floor_in
 *                  Storey to add; ownership moves, its id may change. A null
 *                  storey is ignored.
 */
void insertFloor(SceneMap                        &sceneMap_inout,
                 std::unique_ptr<semantic::Floor> p_floor_in);

/*!
 * @brief           Tells whether a passage still leads to a live room: its
 *                  known side or prospective room is live, or a live room of
 *                  the scene map lists it. Does not lock: the caller holds the
 *                  SceneGraph graph mutex.
 *
 * @param[in]       sceneMap_in
 *                  Scene map whose rooms are searched.
 *
 * @param[in]       p_passage_in
 *                  Passage to test; borrowed, not null.
 *
 * @return          True when at least one live room is linked to it.
 */
[[nodiscard]] bool hasLiveRoomLink(const SceneMap    &sceneMap_in,
                                   semantic::Passage *p_passage_in);

/*!
 * @brief           Flags a room bad and every passage that no longer leads to
 *                  a live room. Does not lock: the caller holds the SceneGraph
 *                  graph mutex.
 *
 * @param[in]       sceneMap_in
 *                  Scene map the room belongs to.
 *
 * @param[in,out]   p_room_inout
 *                  Room to flag; borrowed. A null room is ignored.
 */
void flagRoomBad(const SceneMap &sceneMap_in, semantic::Room *p_room_inout);

/*!
 * @brief           Moves every flagged room, passage and storey of a scene
 *                  map into the retired list. Does not lock: the caller
 *                  holds the SceneGraph graph mutex.
 *
 * @param[in,out]   sceneMap_inout
 *                  Scene map to clean.
 *
 * @param[in,out]   retiredNodes_inout
 *                  Receives the ownership of the flagged nodes.
 */
void retireBadNodes(SceneMap &sceneMap_inout, RetiredNodes &retiredNodes_inout);

/*!
 * @brief           Moves every room, passage and storey of a scene map into
 *                  the retired list and leaves the buildings in place. Does
 *                  not lock: the caller holds the SceneGraph graph mutex.
 *
 * @param[in,out]   sceneMap_inout
 *                  Scene map to empty.
 *
 * @param[in,out]   retiredNodes_inout
 *                  Receives the ownership of every node.
 */
void retireSceneMapContents(SceneMap     &sceneMap_inout,
                            RetiredNodes &retiredNodes_inout);

/*!
 * @brief           Lets every live room, passage and storey drop its link to
 *                  a retired node, and the retired rooms drop their planes.
 *                  Planes are not freed. Does not lock: the caller holds the
 *                  SceneGraph graph mutex. The links held by SLAM maps are
 *                  cleared by the caller, outside the graph mutex.
 *
 * @param[in]       sceneMaps_in
 *                  Scene maps whose remaining nodes are live.
 *
 * @param[in]       retiredNodes_in
 *                  Nodes about to be freed.
 */
void unlinkRetiredNodes(
    const std::map<unsigned long, std::unique_ptr<SceneMap>> &sceneMaps_in,
    const RetiredNodes                                       &retiredNodes_in);

/*!
 * @brief           Lets live storeys and passages drop their links to retired
 *                  rooms and the retired rooms drop their plane lists. Does
 *                  not lock: the caller holds the SceneGraph graph mutex.
 *
 * @param[in]       retiredRooms_in
 *                  Rooms about to be freed.
 *
 * @param[in]       liveFloors_in
 *                  Storeys whose room lists are cleaned.
 *
 * @param[in]       livePassages_in
 *                  Passages whose room links are cleaned.
 */
void unlinkRetiredRooms(
    const std::vector<std::unique_ptr<semantic::Room>> &retiredRooms_in,
    const std::vector<semantic::Floor *>               &liveFloors_in,
    const std::vector<semantic::Passage *>             &livePassages_in);

/*!
 * @brief           Lets live rooms drop retired passages from their passage
 *                  lists. Does not lock: the caller holds the SceneGraph graph
 *                  mutex.
 *
 * @param[in]       retiredPassages_in
 *                  Passages about to be freed.
 *
 * @param[in]       liveRooms_in
 *                  Rooms whose passage lists are cleaned.
 */
void unlinkRetiredPassages(
    const std::vector<std::unique_ptr<semantic::Passage>> &retiredPassages_in,
    const std::vector<semantic::Room *>                   &liveRooms_in);

/*!
 * @brief           Lets live rooms drop their link to retired storeys. Does
 *                  not lock: the caller holds the SceneGraph graph mutex.
 *
 * @param[in]       retiredFloors_in
 *                  Storeys about to be freed.
 *
 * @param[in]       liveRooms_in
 *                  Rooms whose storey link is cleaned.
 */
void unlinkRetiredFloors(
    const std::vector<std::unique_ptr<semantic::Floor>> &retiredFloors_in,
    const std::vector<semantic::Room *>                 &liveRooms_in);

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#include "scene_understanding/scene_graph/scene_graph/private_functions/eraseEntriesByPointer.tpp"
#include "scene_understanding/scene_graph/scene_graph/private_functions/extractEntryByPointer.tpp"
#include "scene_understanding/scene_graph/scene_graph/private_functions/retireDeadEntries.tpp"
#include "scene_understanding/scene_graph/scene_graph/private_functions/retireEntryByPointer.tpp"

#endif /* VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_PRIVATE_FUNCTIONS_H */
