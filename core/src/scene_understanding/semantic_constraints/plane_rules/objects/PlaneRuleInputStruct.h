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
 * @file            PlaneRuleInputStruct.h
 *
 * @brief           Declares the evidence one plane rule judges: a plane's
 *                  label, its orientation and its height.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULE_INPUT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULE_INPUT_STRUCT_H

#include <optional>

#include <Eigen/Core>

#include "mapping/geometric/Plane/objects/PlaneClass.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

/*!
 * @brief           Evidence about one plane (or one candidate patch) that the
 *                  plane rules judge. A rule whose evidence is missing does
 *                  not reject, so each caller fills only what it measured.
 */
struct PlaneRuleInput
{
  public:
    /*!
     * @brief           Label to be checked against the geometry by the reject
     *                  rules; the segmenter's guess for the label rules.
     */
    geometric::Plane::PlaneVariant planeLabel =
        geometric::Plane::PlaneVariant::UNDEFINED;

    /*!
     * @brief           True when planeNormal_common and upDirection_common
     *                  hold a measured orientation.
     */
    bool hasOrientation = false;

    /*!
     * @brief           Unit plane normal, expressed in the same frame as
     *                  upDirection_common (the caller's choice: world for map
     *                  planes, camera for patches). The reject rules ignore
     *                  its sign; the label rules need it to face the
     *                  observing camera.
     */
    Eigen::Vector3d planeNormal_common = Eigen::Vector3d::Zero();

    /*!
     * @brief           Unit up direction (Atlas::getUpDirection()) in the
     *                  frame of planeNormal_common.
     */
    Eigen::Vector3d upDirection_common = Eigen::Vector3d::Zero();

    /*!
     * @brief           Height of the plane in metres along the down direction
     *                  (a larger value is lower), from the ground-height
     *                  measure; absent when not measured.
     */
    std::optional<float> planeHeight_down_m{};

    /*!
     * @brief           Height of the main ground plane in metres along the
     *                  down direction, measured the same way; absent when not
     *                  measured.
     */
    std::optional<float> mainGroundHeight_down_m{};
};

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULE_INPUT_STRUCT_H */
