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
 * @file            visibility_rule_table.h
 *
 * @brief           The one table of visibility rules. A new visibility check
 *                  is a new rule file in public_functions/ and a new row here.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_VISIBILITY_RULE_TABLE_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_VISIBILITY_RULE_TABLE_H

#include "configuration/types/types.h"
#include "mapping/geometric/Plane/objects/PlaneClass.h"
#include "scene_understanding/semantic_constraints/visibility_rules/objects/VisibilityRuleInputStruct.h"
#include "scene_understanding/semantic_constraints/visibility_rules/public_functions/public_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

/*!
 * @brief           Signature every visibility rule shares: true refuses the
 *                  candidate wall for the room and may name the wall it is
 *                  seen through.
 */
using VisibilityRule = bool (*)(const VisibilityRuleInput &visibilityInput_in,
                                const types::SystemParams &systemParams_in,
                                geometric::Plane         *&p_blockingWall_out);

/*!
 * @brief           Every visibility rule, applied in this order by
 *                  isSeenThroughAnyWall(): the passage view first, then the
 *                  view through a solid wall.
 */
inline constexpr VisibilityRule VISIBILITY_RULE_TABLE[] = {
    &isSeenThroughPassage,
    &isSeenThroughSolidWall,
};

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_CONSTRAINTS_VISIBILITY_RULE_TABLE_H */
