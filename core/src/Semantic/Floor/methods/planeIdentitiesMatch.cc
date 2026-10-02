/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            planeIdentitiesMatch.cc
 *
 * @brief           Implements Floor::planeIdentitiesMatch().
 */

#include "Semantic/Floor.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

FloorStatus Floor::planeIdentitiesMatch(const PlaneIdentity &firstIdentity_in,
                                        const PlaneIdentity &secondIdentity_in,
                                        const double maximumNormalAngle_deg_in,
                                        const double maximumOffset_m_in,
                                        double      &normalAngle_deg_out,
                                        double      &offset_m_out,
                                        bool        &isMatch_out)
{
    Eigen::Vector4d firstEquation    = firstIdentity_in.planeEquation_world;
    Eigen::Vector4d secondEquation   = secondIdentity_in.planeEquation_world;
    const double    firstNormalNorm  = firstEquation.head<3>().norm();
    const double    secondNormalNorm = secondEquation.head<3>().norm();

    if (!firstEquation.allFinite() || !secondEquation.allFinite() ||
        firstNormalNorm < 1e-8 || secondNormalNorm < 1e-8)
    {
        normalAngle_deg_out = std::numeric_limits<double>::infinity();
        offset_m_out        = std::numeric_limits<double>::infinity();
        isMatch_out         = false;
        return FloorStatus::FLOOR_STATUS_SUCCESS;
    }

    firstEquation /= firstNormalNorm;
    secondEquation /= secondNormalNorm;

    double normalDot = firstEquation.head<3>().dot(secondEquation.head<3>());
    if (normalDot < 0.0)
    {
        secondEquation = -secondEquation;
        normalDot      = -normalDot;
    }

    normalDot           = std::clamp(normalDot, -1.0, 1.0);
    normalAngle_deg_out = std::acos(normalDot) * 180.0 / std::acos(-1.0);
    offset_m_out        = std::abs(firstEquation(3) - secondEquation(3));

    isMatch_out = normalAngle_deg_out <= maximumNormalAngle_deg_in &&
                  offset_m_out <= maximumOffset_m_in;
    return FloorStatus::FLOOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
