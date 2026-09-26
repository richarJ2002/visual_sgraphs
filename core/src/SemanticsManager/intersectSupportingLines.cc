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

#include "SemanticsManager.h"

#include "private_functions.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Intersects the infinite lines supporting two finite wall
 *                  segments.
 *
 * @param[in]       firstSegment_in
 *                  First wall segment.
 *
 * @param[in]       secondSegment_in
 *                  Second wall segment.
 *
 * @param[out]      intersection_World_m_out
 *                  Intersection in horizontal world axes.
 *
 * @param[out]      firstParameter_out
 *                  Parametric coordinate on the first segment.
 *
 * @param[out]      secondParameter_out
 *                  Parametric coordinate on the second segment.
 *
 * @return          False when the supporting lines are parallel.
 */
bool intersectSupportingLines(const FiniteWallSegment2d &firstSegment_in,
                              const FiniteWallSegment2d &secondSegment_in,
                              Eigen::Vector2d &intersection_World_m_out,
                              double          &firstParameter_out,
                              double          &secondParameter_out)
{
    const Eigen::Vector2d firstDirection =
        firstSegment_in.end_World_m - firstSegment_in.start_World_m;
    const Eigen::Vector2d secondDirection =
        secondSegment_in.end_World_m - secondSegment_in.start_World_m;
    const double denominator = crossProduct2d(firstDirection, secondDirection);

    if (!std::isfinite(denominator) || std::abs(denominator) < 1e-8)
    {
        return false;
    }

    const Eigen::Vector2d startOffset =
        secondSegment_in.start_World_m - firstSegment_in.start_World_m;
    firstParameter_out =
        crossProduct2d(startOffset, secondDirection) / denominator;
    secondParameter_out =
        crossProduct2d(startOffset, firstDirection) / denominator;
    intersection_World_m_out =
        firstSegment_in.start_World_m + firstParameter_out * firstDirection;

    return intersection_World_m_out.allFinite() &&
           std::isfinite(firstParameter_out) &&
           std::isfinite(secondParameter_out);
}

} // namespace core
} // namespace vs_graphs
