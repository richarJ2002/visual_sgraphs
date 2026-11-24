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
 * @file            plane_label_rule_table.h
 *
 * @brief           The one table of plane label rules. A new label from
 *                  geometry is a new rule file in public_functions/ and a new
 *                  row here.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_LABEL_RULE_TABLE_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_LABEL_RULE_TABLE_H

#include <optional>

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
 * @brief           Signature every plane label rule shares: the label the
 *                  plane's geometry gives it, or no value when the rule does
 *                  not apply.
 */
using PlaneLabelRule = std::optional<geometric::Plane::PlaneVariant> (*)(
    const PlaneRuleInput      &planeInput_in,
    const types::SystemParams &systemParams_in);

/*!
 * @brief           Every plane label rule, tried in this order by
 *                  assignPlaneLabel(); the first with a value decides. The
 *                  upright band (|normal . up| <= max_tilt_wall) and the
 *                  level band (|normal . up| >= sqrt(1 - max_tilt_ground^2))
 *                  are disjoint for any tilts below 0.7.
 */
inline constexpr PlaneLabelRule PLANE_LABEL_RULE_TABLE[] = {
    &labelUprightPlane,
    &labelLevelPlane,
};

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_PLANE_LABEL_RULE_TABLE_H */
