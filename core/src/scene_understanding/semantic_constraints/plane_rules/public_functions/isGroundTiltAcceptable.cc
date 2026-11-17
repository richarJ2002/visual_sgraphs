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
 * @file            isGroundTiltAcceptable.cc
 *
 * @brief           Implements the ground tilt plane rule.
 */

#include "scene_understanding/semantic_constraints/plane_rules/public_functions/public_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

bool isGroundTiltAcceptable(const PlaneRuleInput      &planeInput_in,
                            const types::SystemParams &systemParams_in)
{
    if (planeInput_in.planeLabel != geometric::Plane::PlaneVariant::GROUND ||
        !planeInput_in.hasOrientation)
    {
        return true;
    }

    /* The tilt of a ground plane is the length of its normal's part across
     * up; the normal's sign is irrelevant. */
    const double upComponent =
        planeInput_in.planeNormal_common.dot(planeInput_in.upDirection_common);
    const double groundTilt = (planeInput_in.planeNormal_common -
                               upComponent * planeInput_in.upDirection_common)
                                  .norm();
    return !(groundTilt > systemParams_in.planeCheckpoint.maxTiltGround);
}

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */
