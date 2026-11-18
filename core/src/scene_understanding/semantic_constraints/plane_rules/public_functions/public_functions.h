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
 * @brief           Declares the plane rules (one check per file, listed in
 *                  plane_rule_table.h) and the function that applies them.
 *
 * @note            Every rule is an infallible predicate with the PlaneRule
 *                  signature. A rule accepts a plane its label does not
 *                  concern and a plane whose evidence for it is missing. The
 *                  comparisons are written as "not over the limit" so that a
 *                  NaN measure is accepted, as the checks always did.
 *                  Thread-safe: the rules read only their arguments.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULES_PUBLIC_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULES_PUBLIC_FUNCTIONS_H

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
