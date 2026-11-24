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
 * @file            labelLevelPlane.cc
 *
 * @brief           Implements the level plane label rule.
 */

#include "scene_understanding/semantic_constraints/plane_rules/private_functions/private_functions.h"
#include "scene_understanding/semantic_constraints/plane_rules/public_functions/public_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

std::optional<geometric::Plane::PlaneVariant>
    labelLevelPlane(const PlaneRuleInput      &planeInput_in,
                    const types::SystemParams &systemParams_in)
{
    if (!planeInput_in.hasOrientation)
    {
        return std::nullopt;
    }

    /* Written as "within the limit" so a NaN normal gets no label. */
    const double levelTilt = computeLevelTilt(planeInput_in.planeNormal_common,
                                              planeInput_in.upDirection_common);
    if (!(levelTilt <= systemParams_in.planeCheckpoint.maxTiltGround))
    {
        return std::nullopt;
    }

    /*!
     * The normal faces the observer, so a normal along up means the camera
     * looks down onto the plane (ground) and against up means it looks up
     * at it (ceiling).
     */
    const double upComponent =
        planeInput_in.planeNormal_common.dot(planeInput_in.upDirection_common);
    return upComponent > 0.0 ? geometric::Plane::PlaneVariant::GROUND
                             : geometric::Plane::PlaneVariant::CEILING;
}

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */
