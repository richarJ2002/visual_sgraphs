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

/*!
 * @brief           Writes a room's boundary status and corners, and logs the
 *                  status when it changes.
 *
 *                  The stored corners are refreshed every call whose status
 *                  is COMPLETE (wall positions can still drift) and cleared
 *                  for any other status.
 *
 * @param[in,out]   p_room_inout
 *                  Room whose boundary status is written; borrowed.
 *
 * @param[in]       boundaryStatus_in
 *                  New boundary status.
 *
 * @param[in]       boundaryCorners_world_m_in
 *                  Closed-loop corners in world coordinates, in metres; used
 *                  only for COMPLETE.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus updateRoomBoundaryStatus(
    semantic::Room                      *p_room_inout,
    const semantic::Room::BoundaryStatus boundaryStatus_in,
    const std::vector<Eigen::Vector3d>  &boundaryCorners_world_m_in = {});

/*!
 * @brief           Builds a room's finite wall segments and detaches the
 *                  weaker wall of every decisive interior clash, repeating
 *                  until no clash is repaired.
 *
 *                  A room with fewer segments than the minimum wall count is
 *                  marked INCOMPLETE and left unrepaired; an interior clash
 *                  without a decisive support ratio stops the repair as an
 *                  ambiguous conflict.
 *
 * @param[in,out]   p_room_inout
 *                  Room whose clashing walls are detached; borrowed.
 *
 * @param[in]       groundNormal_world_in
 *                  Unit ground normal in world coordinates.
 *
 * @param[in]       groundAxisU_world_in
 *                  First unit axis of the ground plane in world coordinates.
 *
 * @param[in]       groundAxisV_world_in
 *                  Second unit axis of the ground plane in world coordinates.
 *
 * @param[in]       topologyParameters_in
 *                  Boundary topology parameters.
 *
 * @param[out]      wallSegments_out
 *                  Finite wall segments of the room after the last repair.
 *
 * @param[out]      hasAmbiguousConflict_out
 *                  True when an interior clash has no decisive winner.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus repairRoomWallClashes(
    semantic::Room                                       *p_room_inout,
    const Eigen::Vector3d                                &groundNormal_world_in,
    const Eigen::Vector3d                                &groundAxisU_world_in,
    const Eigen::Vector3d                                &groundAxisV_world_in,
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters_in,
    std::vector<FiniteWallSegment2d>                     &wallSegments_out,
    bool &hasAmbiguousConflict_out);

/*!
 * @brief           Stores a room's unobserved angular sectors, computed from
 *                  its wall segments around the room centroid; clears them
 *                  when the centroid is not finite.
 *
 * @param[in,out]   p_room_inout
 *                  Room whose observation gaps are written; borrowed.
 *
 * @param[in]       wallSegments_in
 *                  Finite wall segments of the room.
 *
 * @param[in]       groundAxisU_world_in
 *                  First unit axis of the ground plane in world coordinates.
 *
 * @param[in]       groundAxisV_world_in
 *                  Second unit axis of the ground plane in world coordinates.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus updateRoomObservationGaps(
    semantic::Room                         *p_room_inout,
    const std::vector<FiniteWallSegment2d> &wallSegments_in,
    const Eigen::Vector3d                  &groundAxisU_world_in,
    const Eigen::Vector3d                  &groundAxisV_world_in);

/*!
 * @brief           Retries an open wall loop without one wall, weakest
 *                  support first; the first exclusion that closes the loop
 *                  replaces the segments and the closure.
 *
 * @param[in]       p_room_in
 *                  Room the segments belong to, for the log line; borrowed.
 *
 * @param[in]       roomCentroidGround_m_in
 *                  Room centroid on the ground plane, in metres.
 *
 * @param[in]       topologyParameters_in
 *                  Boundary topology parameters.
 *
 * @param[in,out]   wallSegments_inout
 *                  Wall segments of the open loop; the closing subset on
 *                  success.
 *
 * @param[in,out]   closure_inout
 *                  Closure of the full set; the closing closure on success.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus excludeOffLoopOutlierWall(
    semantic::Room        *p_room_in,
    const Eigen::Vector2d &roomCentroidGround_m_in,
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters_in,
    std::vector<FiniteWallSegment2d>                     &wallSegments_inout,
    WallLoopClosure                                      &closure_inout);

/*!
 * @brief           Tests whether two non-adjacent edges of a closed boundary
 *                  polygon cross in their interiors.
 *
 * @param[in]       boundaryCorners_world_m_in
 *                  Polygon corners in order on the ground plane, in metres.
 *
 * @param[out]      polygonSelfIntersects_out
 *                  True when the polygon intersects itself.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus checkPolygonSelfIntersection(
    const std::vector<Eigen::Vector2d> &boundaryCorners_world_m_in,
    bool                               &polygonSelfIntersects_out);

/*!
 * @brief           Lifts closed-loop corners from the ground plane into
 *                  world coordinates; each corner's height is the mean height
 *                  of its two meeting walls along the ground normal.
 *
 * @param[in]       boundaryCorners_world_m_in
 *                  Loop corners on the ground plane, in metres.
 *
 * @param[in]       wallSegments_in
 *                  Loop wall segments, corner i between walls i and i + 1.
 *
 * @param[in]       groundNormal_world_in
 *                  Unit ground normal in world coordinates.
 *
 * @param[in]       groundAxisU_world_in
 *                  First unit axis of the ground plane in world coordinates.
 *
 * @param[in]       groundAxisV_world_in
 *                  Second unit axis of the ground plane in world coordinates.
 *
 * @param[out]      boundaryCorners3d_world_m_out
 *                  Corners in world coordinates, in metres.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus liftBoundaryCorners(
    const std::vector<Eigen::Vector2d>     &boundaryCorners_world_m_in,
    const std::vector<FiniteWallSegment2d> &wallSegments_in,
    const Eigen::Vector3d                  &groundNormal_world_in,
    const Eigen::Vector3d                  &groundAxisU_world_in,
    const Eigen::Vector3d                  &groundAxisV_world_in,
    std::vector<Eigen::Vector3d>           &boundaryCorners3d_world_m_out);

/*!
 * @brief           Detaches every wall of a COMPLETE room that is neither in
 *                  its closed wall loop nor a framing wall of one of its
 *                  passages.
 *
 * @param[in,out]   p_room_inout
 *                  Room whose off-loop walls are detached; borrowed.
 *
 * @param[in]       wallSegments_in
 *                  Wall segments of the closed loop.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus
    detachOffLoopWalls(semantic::Room                         *p_room_inout,
                       const std::vector<FiniteWallSegment2d> &wallSegments_in);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_ROOM_VALIDATION_PRIVATE_FUNCTIONS_H */
