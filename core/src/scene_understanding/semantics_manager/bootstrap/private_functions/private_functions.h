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
 * @brief           Declares the helpers of the SemanticsManager bootstrap
 *                  methods. Package-internal: not included by any public
 *                  header.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_BOOTSTRAP_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_SEMANTICS_MANAGER_BOOTSTRAP_PRIVATE_FUNCTIONS_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "map_state/atlas/atlas.h"
#include "map_state/map/map.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/PassageContextStruct.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Tells whether a room is a live, confirmed room: not null,
 *                  not flagged bad and of the ROOM variant (not a free-space
 *                  structural element).
 *
 * @param[in]       p_room_in
 *                  Room to test; borrowed, may be null.
 *
 * @return          True for a non-null, live room of the ROOM variant.
 */
[[nodiscard]] bool isLiveConfirmedRoom(semantic::Room *p_room_in);

/*!
 * @brief           Finds the live, confirmed room with a given id.
 *
 * @param[in]       rooms_in
 *                  Rooms to search; borrowed.
 *
 * @param[in]       roomId_in
 *                  Room id to look for; a negative id finds nothing.
 *
 * @return          The room, or nullptr when no live confirmed room has the
 *                  id; borrowed from rooms_in.
 */
[[nodiscard]] semantic::Room *
    findLiveRoomById(const std::vector<semantic::Room *> &rooms_in,
                     int                                  roomId_in);

/*!
 * @brief           Finds the first live, confirmed room of an id-ordered list,
 *                  which is the one with the lowest id.
 *
 * @param[in]       rooms_in
 *                  Rooms in ascending id order; borrowed.
 *
 * @return          The room, or nullptr when the list holds none; borrowed
 *                  from rooms_in.
 */
[[nodiscard]] semantic::Room *
    findFirstLiveRoom(const std::vector<semantic::Room *> &rooms_in);

/*!
 * @brief           Chooses the room a map's bootstrap continues with: the
 *                  current room, else the room the robot was in before a
 *                  tracking loss, else (cold start only, when no recovery room
 *                  exists) the lowest-id live room. With a recovery room, a
 *                  spurious room created during the reset never replaces the
 *                  last known one.
 *
 * @param[in]       rooms_in
 *                  Rooms of the active map in ascending id order; borrowed.
 *
 * @param[in]       currentRoomId_in
 *                  Id of the current room, or -1.
 *
 * @param[in]       recoveryRoomId_in
 *                  Id of the room the robot was lost in, or -1.
 *
 * @return          The room, or nullptr when none applies; borrowed from
 *                  rooms_in.
 */
[[nodiscard]] semantic::Room *
    selectBootstrapRoom(const std::vector<semantic::Room *> &rooms_in,
                        int                                  currentRoomId_in,
                        int                                  recoveryRoomId_in);

/*!
 * @brief           Writes one SG_PIPELINE initialization record to standard
 *                  output. Fields appear in the order map_id, reason, room_id,
 *                  floor_id, restored_passages, semantic_cycle; an absent
 *                  optional field is left out.
 *
 * @param[in]       reason_in
 *                  Reason text, for example BOOTSTRAP_CREATED.
 *
 * @param[in]       mapId_in
 *                  Id of the active map, when known.
 *
 * @param[in]       roomId_in
 *                  Id of the bootstrap room, when it exists.
 *
 * @param[in]       floorId_in
 *                  Id of the canonical storey, when it exists.
 *
 * @param[in]       restoredPassageCount_in
 *                  Number of passages restored after a tracking loss, when
 *                  that applies.
 *
 * @param[in]       semanticCycle_in
 *                  Number of the semantic cycle.
 */
void printBootstrapRecord(
    const std::string                  &reason_in,
    const std::optional<unsigned long> &mapId_in,
    const std::optional<int>           &roomId_in,
    const std::optional<int>           &floorId_in,
    const std::optional<std::size_t>   &restoredPassageCount_in,
    std::uint64_t                       semanticCycle_in);

/*!
 * @brief           Creates a passage object that stands in for one of the lost
 *                  map's passages: a fresh id, no position or aperture size
 *                  (not knowable across a map break), only the frame-free state
 *                  (passable, traversal history). The passage is marked as a
 *                  recovery proxy and is not yet in the scene graph.
 *
 * @param[in,out]   p_atlas_inout
 *                  Atlas that hands out the passage id; borrowed.
 *
 * @param[in,out]   p_activeMap_inout
 *                  Map the passage belongs to; borrowed.
 *
 * @param[in]       passageContext_in
 *                  The lost passage's saved state.
 *
 * @return          The new passage, owned by the caller.
 */
[[nodiscard]] std::unique_ptr<semantic::Passage>
    createRecoveryPassage(atlas::Atlas                   *p_atlas_inout,
                          map::Map                       *p_activeMap_inout,
                          const semantic::PassageContext &passageContext_in);

/*!
 * @brief           Links a restored passage to the recovered room: as the
 *                  known side or far side when the saved state named the
 *                  room's previous id, as the known side when the passage has
 *                  no room yet, and into the room's passage list.
 *
 * @param[in]       passageContext_in
 *                  The lost passage's saved state.
 *
 * @param[in]       recoveredRoomPreviousId_in
 *                  Id the recovered room had in the lost map.
 *
 * @param[in,out]   p_passage_inout
 *                  Restored passage; borrowed.
 *
 * @param[in,out]   p_room_inout
 *                  Recovered room in the new map; borrowed.
 */
void linkRecoveryPassage(const semantic::PassageContext &passageContext_in,
                         int                recoveredRoomPreviousId_in,
                         semantic::Passage *p_passage_inout,
                         semantic::Room    *p_room_inout);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_BOOTSTRAP_PRIVATE_FUNCTIONS_H */
