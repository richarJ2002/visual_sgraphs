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
 * @file            labelUprightPlane.cc
 *
 * @brief           Implements the upright plane label rule.
 */

#include "scene_understanding/semantic_constraints/plane_rules/public_functions/public_functions.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

std::optional<geometric::Plane::PlaneVariant>
    labelUprightPlane(const PlaneRuleInput      &planeInput_in,
                      const types::SystemParams &systemParams_in)
{
    if (!planeInput_in.hasOrientation)
    {
        return std::nullopt;
    }

    /* Written as "within the limit" so a NaN normal gets no label. */
    const double uprightTilt = std::abs(
        planeInput_in.planeNormal_common.dot(planeInput_in.upDirection_common));
    if (!(uprightTilt <= systemParams_in.planeCheckpoint.maxTiltWall))
    {
        return std::nullopt;
    }

    /*!
     * Door and window come only from the segmenter; geometry cannot tell them
     * from a wall, so an upright plane keeps those guesses.
     */
    if (planeInput_in.planeLabel == geometric::Plane::PlaneVariant::DOOR ||
        planeInput_in.planeLabel == geometric::Plane::PlaneVariant::WINDOW)
    {
        return planeInput_in.planeLabel;
    }
    return geometric::Plane::PlaneVariant::WALL;
}

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */
