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
 * @file            PoseCorrectionFieldStruct.h
 *
 * @brief           Declares PoseCorrectionField, the corrections a pose
 *                  optimisation computed for one map, handed to the
 *                  semantics manager to move the map's scene graph.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_POSE_CORRECTION_FIELD_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_POSE_CORRECTION_FIELD_STRUCT_H

#include <Eigen/Core>
#include <map>
#include <vector>
#include <vs_graphs_g2o/types/sim3.h>

namespace vs_graphs
{
namespace core
{
namespace geometric
{
class Plane;
} /* namespace geometric */

/*!
 * @brief           One key frame's correction: where the key frame was before
 *                  the optimisation and how its surroundings moved.
 */
struct PoseCorrectionAnchor
{
  public:
    /*!
     * @brief           Camera centre before the optimisation, old world frame,
     *                  metres.
     */
    Eigen::Vector3d cameraCenter_oldWorld_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Similarity taking the old world frame to the new one
     *                  at this key frame.
     */
    g2o::Sim3 correction_oldWorldToNewWorld{};
};

/*!
 * @brief           The corrections of one map. Plain data and borrowed plane
 *                  keys: holds no scene graph node.
 */
struct PoseCorrectionField
{
  public:
    /*!
     * @brief           Anchors in ascending key frame id order.
     */
    std::vector<PoseCorrectionAnchor> anchors;

    /*!
     * @brief           Correction used when there is no anchor.
     */
    g2o::Sim3 fallbackCorrection_oldWorldToNewWorld{};

    /*!
     * @brief           Correction applied to each moved plane; keys borrowed,
     *                  owned by the map.
     */
    std::map<geometric::Plane *, g2o::Sim3> planeCorrections_oldWorldToNewWorld;

    /*!
     * @brief           Centroid of each moved plane before the move, old world
     *                  frame, metres; keys borrowed.
     */
    std::map<geometric::Plane *, Eigen::Vector3d> planeCentroids_oldWorld_m;

    /*!
     * @brief           Picks the correction of the anchor whose camera centre
     *                  is nearest to a point; the first anchor wins a tie.
     *
     * @param[in]       point_oldWorld_m_in
     *                  Point before the move, old world frame, metres.
     *
     * @return          The nearest anchor's correction, or the fallback when
     *                  the point is not finite or there is no anchor.
     */
    [[nodiscard]] const g2o::Sim3 &selectCorrectionForPoint(
        const Eigen::Vector3d &point_oldWorld_m_in) const;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_POSE_CORRECTION_FIELD_STRUCT_H \
        */
