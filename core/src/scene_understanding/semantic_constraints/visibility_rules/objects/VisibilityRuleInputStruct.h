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
 * @file            VisibilityRuleInputStruct.h
 *
 * @brief           Declares what one visibility rule judges: a candidate wall
 *                  offered to a room, the points the room is judged from and
 *                  the walls that may stand in between.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_VISIBILITY_RULE_INPUT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_VISIBILITY_RULE_INPUT_STRUCT_H

#include <vector>

#include <Eigen/Core>

#include "mapping/geometric/Plane/objects/PlaneClass.h"
#include "scene_understanding/scene_graph/passage/objects/PassageClass.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

/*!
 * @brief           One candidate wall offered to one room, as the visibility
 *                  rules see it. All points are in the active map frame, in
 *                  metres. The pointers are borrowed for the duration of one
 *                  rule call; the map owns the planes and passages.
 */
struct VisibilityRuleInput
{
  public:
    /*!
     * @brief           True when the camera is inside the room, so the room
     *                  is judged from the camera; false when it is judged
     *                  from its centroid.
     */
    bool isCameraViewpoint = false;

    /*!
     * @brief           Room centroid (SemanticsManager::getRoomViewpoints()).
     */
    Eigen::Vector3d roomCentroid_world_m = Eigen::Vector3d::Zero();

    /*!
     * @brief           Point the room is seen from: the camera centre for the
     *                  camera's room, otherwise the room centroid.
     */
    Eigen::Vector3d roomViewpoint_world_m = Eigen::Vector3d::Zero();

    /*!
     * @brief           Camera centres of the observers the room is judged
     *                  from, one sight line each to the candidate centroid.
     *                  Empty (room_seg.see_through_gate.
     *                  observer_sight_lines off, or no observer) keeps the
     *                  single viewpoint above; non-empty, both rules judge
     *                  these lines for any room and refuse when more than
     *                  half of them are blocked.
     */
    std::vector<Eigen::Vector3d> observerViewpoints_world_m{};

    /*!
     * @brief           Centroid of the candidate wall.
     */
    Eigen::Vector3d candidateCentroid_world_m = Eigen::Vector3d::Zero();

    /*!
     * @brief           Candidate wall; never a blocker of itself.
     */
    geometric::Plane *p_candidateWall = nullptr;

    /*!
     * @brief           Other face of the candidate wall, or nullptr; the
     *                  same physical wall, so never a blocker.
     */
    geometric::Plane *p_candidateTwinWall = nullptr;

    /*!
     * @brief           Walls the room owns now; must not be nullptr.
     */
    const std::vector<geometric::Plane *> *p_roomWalls = nullptr;

    /*!
     * @brief           Every passage of the active map in id order; must not
     *                  be nullptr.
     */
    const std::vector<semantic::Passage *> *p_mapPassages = nullptr;

    /*!
     * @brief           Every plane of the active map; read only for the
     *                  camera's room, may be nullptr otherwise.
     */
    const std::vector<geometric::Plane *> *p_mapPlanes = nullptr;
};

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_VISIBILITY_RULE_INPUT_STRUCT_H \
        */
