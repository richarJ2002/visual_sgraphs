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
 * @brief           Declares the helpers of the SemanticsManager graph snapshot
 *                  methods. Package-internal: not included by any public
 *                  header.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_GRAPH_SNAPSHOTS_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_SEMANTICS_MANAGER_GRAPH_SNAPSHOTS_PRIVATE_FUNCTIONS_H

#include "scene_understanding/scene_graph/building/objects/BuildingStruct.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/semantics_manager/objects/SceneGraphBuildingEntryStruct.h"
#include "scene_understanding/semantics_manager/objects/SceneGraphFloorEntryStruct.h"
#include "scene_understanding/semantics_manager/objects/SceneGraphPassageEntryStruct.h"
#include "scene_understanding/semantics_manager/objects/SceneGraphRoomEntryStruct.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Copies one room into plain data. The caller holds the
 *                  room's graph lock.
 *
 * @param[in,out]   p_room_inout
 *                  Room to copy; borrowed, must not be null.
 *
 * @param[out]      roomEntry_out
 *                  The copy; wall planes and passages that are null are left
 *                  out.
 */
void captureRoomEntry(semantic::Room      *p_room_inout,
                      SceneGraphRoomEntry &roomEntry_out);

/*!
 * @brief           Copies one passage into plain data. The caller holds the
 *                  passage's graph lock.
 *
 * @param[in,out]   p_passage_inout
 *                  Passage to copy; borrowed, must not be null.
 *
 * @param[out]      passageEntry_out
 *                  The copy; wall planes that are null are left out.
 */
void capturePassageEntry(semantic::Passage      *p_passage_inout,
                         SceneGraphPassageEntry &passageEntry_out);

/*!
 * @brief           Copies one storey into plain data. The caller holds the
 *                  storey's graph lock.
 *
 * @param[in]       p_floor_in
 *                  Storey to copy; borrowed, must not be null.
 *
 * @param[out]      floorEntry_out
 *                  The copy; rooms that are null are left out.
 */
void captureFloorEntry(const semantic::Floor *p_floor_in,
                       SceneGraphFloorEntry  &floorEntry_out);

/*!
 * @brief           Copies one building into plain data. The caller holds the
 *                  building's graph lock.
 *
 * @param[in]       buildingId_in
 *                  Id the building is keyed by.
 *
 * @param[in]       building_in
 *                  Building to copy.
 *
 * @param[out]      buildingEntry_out
 *                  The copy; storeys flagged bad are left out.
 */
void captureBuildingEntry(int                          buildingId_in,
                          const scene_graph::Building &building_in,
                          SceneGraphBuildingEntry     &buildingEntry_out);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_GRAPH_SNAPSHOTS_PRIVATE_FUNCTIONS_H */
