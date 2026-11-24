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
 * @file            isSeenThroughAnyWall.cc
 *
 * @brief           Implements isSeenThroughAnyWall(): applies the visibility
 *                  rule table.
 */

#include "scene_understanding/semantic_constraints/visibility_rules/public_functions/public_functions.h"

#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rule_table.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

bool isSeenThroughAnyWall(const VisibilityRuleInput &visibilityInput_in,
                          const types::SystemParams &systemParams_in,
                          geometric::Plane         *&p_blockingWall_out)
{
    p_blockingWall_out = nullptr;
    for (const VisibilityRule visibilityRule : VISIBILITY_RULE_TABLE)
    {
        if (visibilityRule(visibilityInput_in,
                           systemParams_in,
                           p_blockingWall_out))
        {
            return true;
        }
    }
    return false;
}

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */
