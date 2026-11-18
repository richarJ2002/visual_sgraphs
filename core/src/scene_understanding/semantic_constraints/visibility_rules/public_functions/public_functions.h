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
 * @brief           Declares the visibility rules (one check per file, listed
 *                  in visibility_rule_table.h), the function that applies
 *                  them and the wall-separation test they share.
 *
 * @note            Every rule is an infallible predicate with the
 *                  VisibilityRule signature: true means the room sees the
 *                  candidate wall through another wall, so the wall is not
 *                  admitted to the room. A rule does nothing (false) while
 *                  room_seg.see_through_gate.enabled is off. The rules read
 *                  only their arguments and the planes they point to; the
 *                  caller holds the graph lock.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_VISIBILITY_RULES_PUBLIC_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_VISIBILITY_RULES_PUBLIC_FUNCTIONS_H

#include <vector>

#include <Eigen/Core>

#include "configuration/types/types.h"
#include "mapping/geometric/Plane/objects/PlaneClass.h"
#include "scene_understanding/semantic_constraints/visibility_rules/objects/VisibilityRuleInputStruct.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

/*!
 * @brief           Checks whether a finite mapped wall separates two points.
 *
 *                  The infinite plane performs the side test, while the
 *                  mapped cloud bounds reject unrelated coplanar wall
 *                  segments. Both points must lie at least 0.10 m from the
 *                  wall plane.
 *
 * @param[in]       wallList_world_in
 *                  Candidate wall surfaces expressed in the active map frame.
 *
 * @param[in]       firstPoint_world_m_in
 *                  First position expressed in the active map frame, in
 *                  metres.
 *
 * @param[in]       secondPoint_world_m_in
 *                  Second position expressed in the active map frame, in
 *                  metres.
 *
 * @param[in]       finiteBoundsMargin_m_in
 *                  Margin applied around the observed wall-cloud bounds, in
 *                  metres.
 *
 * @param[in]       isLeadingWallVoteAccepted_in
 *                  When true, a plane whose type is not yet WALL but whose
 *                  leading semantic vote is WALL also counts as a wall;
 *                  when false, only planes of confirmed type WALL count.
 *
 * @return          True when the segment crosses an observed finite wall
 *                  patch.
 */
[[nodiscard]] bool hasSeparatingFiniteWall(
    const std::vector<geometric::Plane *> &wallList_world_in,
    const Eigen::Vector3d                 &firstPoint_world_m_in,
    const Eigen::Vector3d                 &secondPoint_world_m_in,
    const double                           finiteBoundsMargin_m_in,
    const bool                             isLeadingWallVoteAccepted_in);

/*!
 * @brief           Passage rule, for a room the camera is not in: the room
 *                  is judged from its centroid, and a candidate whose
 *                  centroid lies beyond a wall that carries a passage bounds
 *                  the room on the other side.
 *
 *                  A wall with a passage is a known boundary between two
 *                  rooms. The aperture backstop only catches a line through
 *                  the passage's opening; this catches a line through the
 *                  solid part of the same wall. The candidate and its twin
 *                  face never block.
 *
 * @param[in]       visibilityInput_in
 *                  Candidate wall and room viewpoints; reads p_mapPassages.
 *
 * @param[in]       systemParams_in
 *                  System parameters (roomSeg.isSeeThroughGateEnabled,
 *                  semSeg.reassociate.wallExtension.maximumInPlaneGap_m).
 *
 * @param[out]      p_blockingWall_out
 *                  Always nullptr: the refusal rests on the set of passage
 *                  walls.
 *
 * @return          True when the candidate is beyond a passage wall.
 */
[[nodiscard]] bool
    isSeenThroughPassage(const VisibilityRuleInput &visibilityInput_in,
                         const types::SystemParams &systemParams_in,
                         geometric::Plane         *&p_blockingWall_out);

/*!
 * @brief           Solid-wall rule, only for the room the camera is in: a
 *                  candidate whose centroid the camera sees through a nearer
 *                  wall N (through N's doorway) lies beyond N.
 *
 *                  N is any wall of the room, or any mapped plane whose
 *                  leading vote is WALL before its type is confirmed. A wall
 *                  with a passage in it keeps blocking. The candidate's twin
 *                  face never blocks. The hold ends by itself when the camera
 *                  crosses N's plane or N's vote or type moves away from
 *                  WALL. The crossing must lie inside N's observed extent
 *                  widened by wall_extension.maximum_in_plane_gap.
 *
 * @param[in]       visibilityInput_in
 *                  Candidate wall and room viewpoints; reads p_roomWalls and
 *                  p_mapPlanes.
 *
 * @param[in]       systemParams_in
 *                  System parameters (roomSeg.isSeeThroughGateEnabled,
 *                  semSeg.reassociate.wallExtension.maximumInPlaneGap_m).
 *
 * @param[out]      p_blockingWall_out
 *                  The nearer wall the candidate is seen through, or nullptr
 *                  when the rule does not refuse.
 *
 * @return          True when the camera sees the candidate through a nearer
 *                  wall.
 */
[[nodiscard]] bool
    isSeenThroughSolidWall(const VisibilityRuleInput &visibilityInput_in,
                           const types::SystemParams &systemParams_in,
                           geometric::Plane         *&p_blockingWall_out);

/*!
 * @brief           Applies the visibility rule table in table order and
 *                  stops at the first rule that refuses.
 *
 * @param[in]       visibilityInput_in
 *                  Candidate wall and room viewpoints.
 *
 * @param[in]       systemParams_in
 *                  System parameters the rules read.
 *
 * @param[out]      p_blockingWall_out
 *                  Blocking wall named by the refusing rule, or nullptr.
 *
 * @return          True when a rule refuses the candidate.
 */
[[nodiscard]] bool
    isSeenThroughAnyWall(const VisibilityRuleInput &visibilityInput_in,
                         const types::SystemParams &systemParams_in,
                         geometric::Plane         *&p_blockingWall_out);

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_VISIBILITY_RULES_PUBLIC_FUNCTIONS_H \
        */
