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
 * @brief           Declares the helpers of the map linking methods.
 *                  Package-internal: not included by any public header.
 */

#ifndef VS_GRAPHS_MAP_LINKING_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_MAP_LINKING_PRIVATE_FUNCTIONS_H

#include <memory>

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

#endif /* VS_GRAPHS_MAP_LINKING_PRIVATE_FUNCTIONS_H */
