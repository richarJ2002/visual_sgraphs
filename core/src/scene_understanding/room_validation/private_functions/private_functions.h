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
 * @brief           Declares the helpers of the room validation stage.
 *                  Package-internal: not included by any public header.
 */

#ifndef VS_GRAPHS_CORE_SRC_ROOM_VALIDATION_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_ROOM_VALIDATION_PRIVATE_FUNCTIONS_H

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "configuration/types/types.h"
#include "scene_understanding/scene_graph/room/objects/RoomClass.h"
#include "scene_understanding/semantics_manager/objects/FiniteWallSegment2dStruct.h"
#include "scene_understanding/semantics_manager/objects/SemanticsManagerStatusEnum.h"
#include "scene_understanding/semantics_manager/objects/WallLoopClosureStruct.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Attempts to close the given wall segments (sorted here by
 *                  angle from the room centroid) into one ordered loop, exactly
 *                  as validateRoomBoundaries() always did for a room's full
 *                  wall set. Factored out so the caller can retry on a reduced
 *                  subset when the full set doesn't close (see
 *                  validateRoomBoundaries()'s single-outlier-exclusion retry).
 */
[[nodiscard]] SemanticsManagerStatus tryCloseWallLoop(
    std::vector<FiniteWallSegment2d> wallSegments_in,
    const Eigen::Vector2d           &roomCentroidGround_m_in,
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters_in,
    WallLoopClosure                                      &closure_out);

/*!
 * @brief           Finds the angular sectors (from roomCentroidGround_m_in)
 *                  with no wall evidence -- the "where is this room still
 *                  unobserved" signal (user rule: track and expose
 *                  incomplete-room state, not just a pass/fail boundary
 *                  status).
 *
 *                  Deliberately coarser than the corner-closing algorithm
 *                  above: each wall is reduced to its 2D midpoint angle from
 *                  the centroid, not its true angular extent, trading a small
 *                  amount of precision (a wide wall's own angular span isn't
 *                  subtracted from a neighbouring gap) for a computation that
 *                  stays meaningful at any wall count, including 0 or 1 -- the
 *                  boundary-loop algorithm's own machinery only starts
 *                  producing useful output once minimumWallCount is met.
 *
 * @param[in]       wallSegments_in
 *                  Finite wall segments of the room.
 *
 * @param[in]       roomCentroidGround_m_in
 *                  Room centroid on the ground plane, in
 *                  metres.
 *
 * @param[out]      roomObservationGaps_out
 *                  Sectors wider than the threshold that
 *                  no wall covers.
 *
 * @param[in]       gapThreshold_rad_in
 *                  Smallest angle between neighbouring
 *                  wall midpoints reported as a gap, in
 *                  radians.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus computeRoomObservationGaps(
    const std::vector<FiniteWallSegment2d>      &wallSegments_in,
    const Eigen::Vector2d                       &roomCentroidGround_m_in,
    /* An axis-aligned (or any) rectangle's four wall midpoints sit exactly
     * on its principal axes as seen from the centroid -- always exactly 90
     * deg apart by construction, regardless of aspect ratio. The threshold
     * must clear that deterministic case with margin, or every well-formed
     * rectangular room reports four phantom gaps. */
    std::vector<semantic::Room::ObservationGap> &roomObservationGaps_out,
    double gapThreshold_rad_in = 100.0 * M_PI / 180.0);

/*!
 * @brief           Computes the unsigned area of an ordered horizontal polygon.
 *
 * @param[in]       polygonVertices_world_m_in
 *                  Vertices in order, in metres; fewer
 *                  than three give an area of 0.
 *
 * @param[out]      polygonArea_m2_out
 *                  Area, in square metres.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus computePolygonArea_m2(
    const std::vector<Eigen::Vector2d> &polygonVertices_world_m_in,
    double                             &polygonArea_m2_out);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_ROOM_VALIDATION_PRIVATE_FUNCTIONS_H */
