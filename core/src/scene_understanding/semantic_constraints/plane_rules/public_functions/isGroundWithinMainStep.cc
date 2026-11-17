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
 * @file            isGroundWithinMainStep.cc
 *
 * @brief           Implements the ground step plane rule.
 */

#include "scene_understanding/semantic_constraints/plane_rules/public_functions/public_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

bool isGroundWithinMainStep(const PlaneRuleInput      &planeInput_in,
                            const types::SystemParams &systemParams_in)
{
    if (planeInput_in.planeLabel != geometric::Plane::PlaneVariant::GROUND ||
        !planeInput_in.planeHeight_down_m.has_value() ||
        !planeInput_in.mainGroundHeight_down_m.has_value())
    {
        return true;
    }

    /* Heights grow downwards, so a plane higher than the main ground by
     * more than one step has a smaller height than the step limit. */
    const float stepLimitHeight_down_m =
        *planeInput_in.mainGroundHeight_down_m -
        systemParams_in.semSeg.maxStepElevation;
    return !(*planeInput_in.planeHeight_down_m < stepLimitHeight_down_m);
}

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */
