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
 * @file            selectCorrectionForPoint.cc
 *
 * @brief           Implements PoseCorrectionField::selectCorrectionForPoint(),
 *                  declared in
 *                  semantics_manager/objects/PoseCorrectionFieldStruct.h.
 */

#include "scene_understanding/semantics_manager/objects/PoseCorrectionFieldStruct.h"

#include <limits>

namespace vs_graphs
{
namespace core
{

const g2o::Sim3 &PoseCorrectionField::selectCorrectionForPoint(
    const Eigen::Vector3d &point_oldWorld_m_in) const
{
    if (!point_oldWorld_m_in.allFinite())
    {
        return fallbackCorrection_oldWorldToNewWorld;
    }

    const PoseCorrectionAnchor *p_nearestAnchor = nullptr;
    double nearestSquaredDistance_m2 = std::numeric_limits<double>::infinity();

    for (const PoseCorrectionAnchor &anchor : anchors)
    {
        const double squaredDistance_m2 =
            (anchor.cameraCenter_oldWorld_m - point_oldWorld_m_in)
                .squaredNorm();

        if (squaredDistance_m2 < nearestSquaredDistance_m2)
        {
            nearestSquaredDistance_m2 = squaredDistance_m2;
            p_nearestAnchor           = &anchor;
        }
    }

    return p_nearestAnchor != nullptr
               ? p_nearestAnchor->correction_oldWorldToNewWorld
               : fallbackCorrection_oldWorldToNewWorld;
}

} /* namespace core */
} /* namespace vs_graphs */
