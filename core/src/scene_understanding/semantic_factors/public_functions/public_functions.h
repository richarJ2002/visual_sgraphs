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
 * @file            public_functions.h
 *
 * @brief           Declares the functions that add the semantic factors
 *                  (wall-face thickness, room-wall, semantic layout) to a
 *                  bundle adjustment, and the plane lookups they need.
 */

#ifndef VS_GRAPHS_SEMANTIC_FACTORS_PUBLIC_FUNCTIONS_H
#define VS_GRAPHS_SEMANTIC_FACTORS_PUBLIC_FUNCTIONS_H

#include <functional>
#include <map>
#include <unordered_map>
#include <vector>

#include "optimization/optimizer/objects/OptimizerStatusEnum.h"
#include "scene_understanding/semantics_manager/objects/RoomSnapshotStruct.h"

namespace vs_graphs::core::geometric
{
class Plane;
} /* namespace vs_graphs::core::geometric */
namespace vs_graphs::core::map
{
class Map;
} /* namespace vs_graphs::core::map */
namespace vs_graphs::core::types
{
class SystemParams;
} /* namespace vs_graphs::core::types */
namespace g2o
{
class SparseOptimizer;
} /* namespace g2o */

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Adds one EdgeWallFaceThickness plus one
 *                  EdgeVertexPlaneParallelism between each symmetric twin
 *                  plane pair visible to the optimiser.
 *
 * @param[in,out]   optimizer_inout
 *                  g2o optimiser owning the plane vertices.
 *
 * @param[in]       planes_in
 *                  Planes with vertices in the optimiser.
 *
 * @param[in]       getVertexId_in
 *                  Callable returning a plane's optimiser vertex id.
 *
 * @param[in]       p_systemParams_in
 *                  System parameters (optimization.wall_thickness).
 *
 * @param[in]       huberThreshold1d_in
 *                  Huber delta for the one-dimensional edges.
 *
 * @return          OPTIMIZER_STATUS_SUCCESS always.
 */
[[nodiscard]] optimizer::OptimizerStatus addTwinFaceThicknessEdges(
    g2o::SparseOptimizer                         &optimizer_inout,
    const std::vector<geometric::Plane *>        &planes_in,
    const std::function<int(geometric::Plane *)> &getVertexId_in,
    const types::SystemParams                    *p_systemParams_in,
    double                                        huberThreshold1d_in);

/*!
 * @brief           Indexes planes by their id.
 *
 * @param[in]       planes_in
 *                  Planes to index; borrowed, owned by the map.
 *
 * @param[out]      planesById_out
 *                  Receives the index; replaced.
 */
void indexPlanesById(
    const std::vector<geometric::Plane *>       &planes_in,
    std::unordered_map<int, geometric::Plane *> &planesById_out);

/*!
 * @brief           Resolves the wall plane ids of a room snapshot to planes,
 *                  keeping the room's order. An id the index does not hold
 *                  (the plane left the map) is skipped.
 *
 * @param[in]       room_in
 *                  Room snapshot.
 *
 * @param[in]       planesById_in
 *                  Planes of the map by id.
 *
 * @param[out]      walls_out
 *                  Receives the wall planes; borrowed, replaced.
 */
void resolveRoomWallPlanes(
    const RoomSnapshot                                &room_in,
    const std::unordered_map<int, geometric::Plane *> &planesById_in,
    std::vector<geometric::Plane *>                   &walls_out);

/*!
 * @brief           Adds room-wall edges: one EdgeVertex4PlaneProjectSE3Room
 *                  for a four-wall room bounded by two perpendicular facing
 *                  wall pairs, otherwise one EdgeVertexNPlaneProjectSE3Room
 *                  per facing wall pair. Each edge weighs the room-centre
 *                  error only along the pair normals. Only rooms with a
 *                  closed (COMPLETE) boundary get edges.
 *
 * @param[in,out]   optimizer_inout
 *                  g2o optimiser owning the room and plane vertices.
 *
 * @param[in]       rooms_in
 *                  Snapshots of the rooms; a room without an entry in
 *                  roomVertexIdsByRoomId_in is skipped.
 *
 * @param[in]       planesById_in
 *                  Planes of the map by plane id, to resolve the walls.
 *
 * @param[in]       getWallVertexId_in
 *                  Callable returning a wall's optimiser vertex id.
 *
 * @param[in]       roomVertexIdsByRoomId_in
 *                  Optimiser vertex id of each room's centre, by room id.
 *
 * @param[in]       p_systemParams_in
 *                  System parameters (optimization.room_wall).
 *
 * @param[in]       huberThreshold3d_in
 *                  Huber delta for the three-dimensional edges.
 *
 * @return          OPTIMIZER_STATUS_SUCCESS always.
 */
[[nodiscard]] optimizer::OptimizerStatus addRoomWallEdges(
    g2o::SparseOptimizer                              &optimizer_inout,
    const std::vector<RoomSnapshot>                   &rooms_in,
    const std::unordered_map<int, geometric::Plane *> &planesById_in,
    const std::function<int(geometric::Plane *)>      &getWallVertexId_in,
    const std::map<int, int>                          &roomVertexIdsByRoomId_in,
    const types::SystemParams                         *p_systemParams_in,
    double                                             huberThreshold3d_in);

/*!
 * @brief           Adds the semantic layout edges whose switches are set in
 *                  semantics.factors (EdgeWallCorner for wall_edge,
 *                  EdgeRoomWallOverlap for room_overlap) on the walls of
 *                  this optimisation, and logs one line with the number and
 *                  initial chi2 of each enabled edge type. Rooms are inputs
 *                  only; no room vertex is used.
 *
 * @param[in,out]   optimizer_inout
 *                  g2o optimiser owning the plane vertices; takes ownership
 *                  of the new edges.
 *
 * @param[in]       planes_in
 *                  Planes with vertices in this optimisation; borrowed.
 *
 * @param[in]       getVertexId_in
 *                  Callable returning a plane's optimiser vertex id.
 *
 * @param[in]       rooms_in
 *                  Snapshots of the map's rooms (wall membership, closed
 *                  boundaries).
 *
 * @param[in]       p_map_in
 *                  Map of the planes, for its up direction; borrowed. Null:
 *                  no edge is added.
 *
 * @param[in]       p_systemParams_in
 *                  System parameters (semantics.factors). Null: no edge is
 *                  added.
 *
 * @param[in]       p_optimisationName_in
 *                  Name of the optimisation in the log line ("LBA", "GBA");
 *                  not null.
 *
 * @return          OPTIMIZER_STATUS_SUCCESS always.
 */
[[nodiscard]] optimizer::OptimizerStatus addSemanticLayoutEdges(
    g2o::SparseOptimizer                         &optimizer_inout,
    const std::vector<geometric::Plane *>        &planes_in,
    const std::function<int(geometric::Plane *)> &getVertexId_in,
    const std::vector<RoomSnapshot>              &rooms_in,
    map::Map                                     *p_map_in,
    const types::SystemParams                    *p_systemParams_in,
    const char                                   *p_optimisationName_in);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTIC_FACTORS_PUBLIC_FUNCTIONS_H */
