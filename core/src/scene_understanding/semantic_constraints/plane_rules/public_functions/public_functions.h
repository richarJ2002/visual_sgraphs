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
 * @brief           Declares the plane rules (one check per file): the reject
 *                  rules listed in plane_rule_table.h, the label rules listed
 *                  in plane_label_rule_table.h, and the functions that apply
 *                  each table.
 *
 * @note            Every reject rule is an infallible predicate with the
 *                  PlaneRule signature. A reject rule accepts a plane its
 *                  label does not concern and a plane whose evidence for it
 *                  is missing; its comparisons are written as "not over the
 *                  limit" so that a NaN measure is accepted, as the checks
 *                  always did. Every label rule has the PlaneLabelRule
 *                  signature and gives no label for a NaN measure.
 *                  Thread-safe: the rules read only their arguments.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULES_PUBLIC_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULES_PUBLIC_FUNCTIONS_H

#include <optional>

#include "configuration/types/types.h"
#include "scene_understanding/semantic_constraints/plane_rules/objects/PlaneRuleInputStruct.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

/*!
 * @brief           Level tilt rule: a GROUND or CEILING plane's normal may
 *                  lean off up by at most plane_checkpoint.max_tilt_ground,
 *                  measured as the length of the normal's part across up.
 *
 * @param[in]       planeInput_in
 *                  Plane evidence; needs hasOrientation.
 *
 * @param[in]       systemParams_in
 *                  System parameters (planeCheckpoint.maxTiltGround).
 *
 * @return          False only for a GROUND or CEILING plane tilted past the
 *                  limit.
 */
[[nodiscard]] bool
    isGroundTiltAcceptable(const PlaneRuleInput      &planeInput_in,
                           const types::SystemParams &systemParams_in);

/*!
 * @brief           Upright tilt rule: a WALL, DOOR or WINDOW plane's normal
 *                  may lean along up by at most plane_checkpoint.max_tilt_wall,
 *                  measured as the absolute part of the normal along up.
 *
 * @param[in]       planeInput_in
 *                  Plane evidence; needs hasOrientation.
 *
 * @param[in]       systemParams_in
 *                  System parameters (planeCheckpoint.maxTiltWall).
 *
 * @return          False only for an upright plane tilted past the limit.
 */
[[nodiscard]] bool
    isUprightTiltAcceptable(const PlaneRuleInput      &planeInput_in,
                            const types::SystemParams &systemParams_in);

/*!
 * @brief           Ground step rule: a GROUND plane may lie at most
 *                  sem_seg.max_step_elevation metres above the main ground
 *                  plane. Lower planes always pass.
 *
 * @param[in]       planeInput_in
 *                  Plane evidence; needs planeHeight_down_m and
 *                  mainGroundHeight_down_m.
 *
 * @param[in]       systemParams_in
 *                  System parameters (semSeg.maxStepElevation, metres).
 *
 * @return          False only for a GROUND plane above the step limit.
 */
[[nodiscard]] bool
    isGroundWithinMainStep(const PlaneRuleInput      &planeInput_in,
                           const types::SystemParams &systemParams_in);

/*!
 * @brief           Upright label rule: a plane whose normal leans along up by
 *                  at most plane_checkpoint.max_tilt_wall (absolute part of
 *                  the normal along up) is upright.
 *
 * @param[in]       planeInput_in
 *                  Plane evidence; needs hasOrientation. planeLabel is the
 *                  segmenter's guess.
 *
 * @param[in]       systemParams_in
 *                  System parameters (planeCheckpoint.maxTiltWall).
 *
 * @return          DOOR or WINDOW when that is the guess, otherwise WALL, for
 *                  an upright plane; no value for any other plane.
 */
[[nodiscard]] std::optional<geometric::Plane::PlaneVariant>
    labelUprightPlane(const PlaneRuleInput      &planeInput_in,
                      const types::SystemParams &systemParams_in);

/*!
 * @brief           Level label rule: a plane whose normal leans off up by at
 *                  most plane_checkpoint.max_tilt_ground (length of the
 *                  normal's part across up) is level.
 *
 * @param[in]       planeInput_in
 *                  Plane evidence; needs hasOrientation and a normal facing
 *                  the observing camera.
 *
 * @param[in]       systemParams_in
 *                  System parameters (planeCheckpoint.maxTiltGround).
 *
 * @return          GROUND when the normal points along up, CEILING when it
 *                  points against up, for a level plane; no value for any
 *                  other plane.
 */
[[nodiscard]] std::optional<geometric::Plane::PlaneVariant>
    labelLevelPlane(const PlaneRuleInput      &planeInput_in,
                    const types::SystemParams &systemParams_in);

/*!
 * @brief           Gives a plane its label from its geometry with the plane
 *                  label rule table: geometry decides, the segmenter's label
 *                  is only the guess.
 *
 * @param[in]       planeInput_in
 *                  Plane evidence; planeLabel is the segmenter's guess
 *                  (UNDEFINED without a segmenter).
 *
 * @param[in]       systemParams_in
 *                  System parameters the rules read.
 *
 * @return          The first label a rule gives, in table order; UNDEFINED
 *                  when no rule gives one (a slanted plane or no
 *                  orientation).
 */
[[nodiscard]] geometric::Plane::PlaneVariant
    assignPlaneLabel(const PlaneRuleInput      &planeInput_in,
                     const types::SystemParams &systemParams_in);

/*!
 * @brief           Applies every rule of the plane rule table, in table
 *                  order.
 *
 * @param[in]       planeInput_in
 *                  Plane evidence.
 *
 * @param[in]       systemParams_in
 *                  System parameters the rules read.
 *
 * @return          True when no rule rejects the plane.
 */
[[nodiscard]] bool
    arePlaneRulesSatisfied(const PlaneRuleInput      &planeInput_in,
                           const types::SystemParams &systemParams_in);

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULES_PUBLIC_FUNCTIONS_H \
        */
