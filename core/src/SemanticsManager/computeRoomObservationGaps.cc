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

#include <algorithm>
#include <cmath>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief Computes the unsigned area of an ordered horizontal polygon.
 */
/*!
 * @brief Finds the angular sectors (from roomCentroid_Ground_m_in) with no
 *        wall evidence -- the "where is this room still unobserved" signal
 *        (user rule: track and expose incomplete-room state, not just a
 *        pass/fail boundary status).
 *
 *        Deliberately coarser than the corner-closing algorithm above: each
 *        wall is reduced to its 2D midpoint angle from the centroid, not its
 *        true angular extent, trading a small amount of precision (a wide
 *        wall's own angular span isn't subtracted from a neighbouring gap)
 *        for a computation that stays meaningful at any wall count,
 *        including 0 or 1 -- the boundary-loop algorithm's own machinery
 *        only starts producing useful output once minimumWallCount is met.
 */
std::vector<semantic::Room::ObservationGap> computeRoomObservationGaps(
    const std::vector<FiniteWallSegment2d> &wallSegments_in,
    const Eigen::Vector2d                  &roomCentroid_Ground_m_in,
    /* An axis-aligned (or any) rectangle's four wall midpoints sit exactly
     * on its principal axes as seen from the centroid -- always exactly 90
     * deg apart by construction, regardless of aspect ratio. The threshold
     * must clear that deterministic case with margin, or every well-formed
     * rectangular room reports four phantom gaps. */
    double                                  gapThreshold_rad_in)
{
    std::vector<semantic::Room::ObservationGap> gaps;

    if (wallSegments_in.empty())
    {
        gaps.push_back({0.0, 2.0 * M_PI});
        return gaps;
    }

    std::vector<double> midpointAngles_rad;
    midpointAngles_rad.reserve(wallSegments_in.size());
    for (const FiniteWallSegment2d &segment : wallSegments_in)
    {
        const Eigen::Vector2d midpoint_Ground_m =
            0.5 * (segment.start_World_m + segment.end_World_m) -
            roomCentroid_Ground_m_in;
        if (!midpoint_Ground_m.allFinite() ||
            midpoint_Ground_m.squaredNorm() < 1e-12)
        {
            continue;
        }
        midpointAngles_rad.push_back(
            std::atan2(midpoint_Ground_m.y(), midpoint_Ground_m.x()));
    }

    if (midpointAngles_rad.empty())
    {
        gaps.push_back({0.0, 2.0 * M_PI});
        return gaps;
    }

    std::sort(midpointAngles_rad.begin(), midpointAngles_rad.end());

    for (std::size_t index = 0U; index < midpointAngles_rad.size(); ++index)
    {
        const double thisAngle_rad = midpointAngles_rad[index];
        const double nextAngle_rad = (index + 1U < midpointAngles_rad.size())
                                         ? midpointAngles_rad[index + 1U]
                                         : midpointAngles_rad[0] + 2.0 * M_PI;
        const double span_rad      = nextAngle_rad - thisAngle_rad;

        if (span_rad > gapThreshold_rad_in)
        {
            gaps.push_back({thisAngle_rad, span_rad});
        }
    }

    return gaps;
}

} // namespace core
} // namespace vs_graphs
