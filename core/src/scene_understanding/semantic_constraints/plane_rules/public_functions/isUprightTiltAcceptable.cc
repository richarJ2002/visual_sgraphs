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
 * @file            isUprightTiltAcceptable.cc
 *
 * @brief           Implements the upright (wall, door, window) tilt plane rule.
 */

#include "scene_understanding/semantic_constraints/plane_rules/public_functions/public_functions.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

bool isUprightTiltAcceptable(const PlaneRuleInput      &planeInput_in,
                             const types::SystemParams &systemParams_in)
{
    const bool isUprightLabel =
        planeInput_in.planeLabel == geometric::Plane::PlaneVariant::WALL ||
        planeInput_in.planeLabel == geometric::Plane::PlaneVariant::DOOR ||
        planeInput_in.planeLabel == geometric::Plane::PlaneVariant::WINDOW;
    if (!isUprightLabel || !planeInput_in.hasOrientation)
    {
        return true;
    }

    /* The tilt of an upright plane is its normal's part along up; the
     * normal's sign is irrelevant. */
    const double upComponent =
        planeInput_in.planeNormal_common.dot(planeInput_in.upDirection_common);
    const double uprightTilt = std::abs(upComponent);
    return !(uprightTilt > systemParams_in.planeCheckpoint.maxTiltWall);
}

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */
