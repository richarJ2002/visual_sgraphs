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

namespace vs_graphs
{
namespace core
{

/*!
 * @brief Returns the Euclidean distance from a point to a finite segment.
 */
SemanticsManagerStatus
    pointToSegmentDistance_m(const Eigen::Vector2d     &point_World_m_in,
                             const FiniteWallSegment2d &segment_in,
                             double                    &distance_m_out)
{
    const Eigen::Vector2d segmentDirection =
        segment_in.end_World_m - segment_in.start_World_m;
    const double squaredLength = segmentDirection.squaredNorm();

    if (squaredLength < 1e-12)
    {
        distance_m_out = (point_World_m_in - segment_in.start_World_m).norm();
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const double interpolation = std::clamp(
        (point_World_m_in - segment_in.start_World_m).dot(segmentDirection) /
            squaredLength,
        0.0,
        1.0);

    distance_m_out = (point_World_m_in - (segment_in.start_World_m +
                                          interpolation * segmentDirection))
                         .norm();
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
