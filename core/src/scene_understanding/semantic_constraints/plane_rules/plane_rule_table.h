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
 * @file            plane_rule_table.h
 *
 * @brief           The one table of plane rules. A new plane check is a new
 *                  rule file in public_functions/ and a new row here.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULE_TABLE_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULE_TABLE_H

#include "configuration/types/types.h"
#include "scene_understanding/semantic_constraints/plane_rules/objects/PlaneRuleInputStruct.h"
#include "scene_understanding/semantic_constraints/plane_rules/public_functions/public_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

/*!
 * @brief           Signature every plane rule shares: true keeps the plane's
 *                  label, false rejects it.
 */
using PlaneRule = bool (*)(const PlaneRuleInput      &planeInput_in,
                           const types::SystemParams &systemParams_in);

/*!
 * @brief           Every plane rule, applied in this order by
 *                  arePlaneRulesSatisfied().
 */
inline constexpr PlaneRule PLANE_RULE_TABLE[] = {
    &isGroundTiltAcceptable,
    &isUprightTiltAcceptable,
    &isGroundWithinMainStep,
};

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_RULE_TABLE_H */
