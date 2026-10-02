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
 * @file            computeRoomObservationGaps.cc
 *
 * @brief           Implements computeRoomObservationGaps(), declared in
 *                  SemanticsManager/private_functions.h.
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
 * @brief Finds the angular sectors (from roomCentroidGround_m_in) with no
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
 *
 * @param[in]  wallSegments_in          Finite wall segments of the room.
 * @param[in]  roomCentroidGround_m_in  Room centroid on the ground plane, in
 *                                      metres.
 * @param[out] roomObservationGaps_out  Sectors wider than the threshold that
 *                                      no wall covers.
 * @param[in]  gapThreshold_rad_in      Smallest angle between neighbouring
 *                                      wall midpoints reported as a gap, in
 *                                      radians.
 * @return SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
SemanticsManagerStatus computeRoomObservationGaps(
    const std::vector<FiniteWallSegment2d>      &wallSegments_in,
    const Eigen::Vector2d                       &roomCentroidGround_m_in,
    /* An axis-aligned (or any) rectangle's four wall midpoints sit exactly
     * on its principal axes as seen from the centroid -- always exactly 90
     * deg apart by construction, regardless of aspect ratio. The threshold
     * must clear that deterministic case with margin, or every well-formed
     * rectangular room reports four phantom gaps. */
    std::vector<semantic::Room::ObservationGap> &roomObservationGaps_out,
    double                                       gapThreshold_rad_in)
{
    std::vector<semantic::Room::ObservationGap> gaps;

    if (wallSegments_in.empty())
    {
        gaps.push_back({0.0, 2.0 * M_PI});
        roomObservationGaps_out = gaps;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<double> midpointAngles_rad;
    midpointAngles_rad.reserve(wallSegments_in.size());
    for (const FiniteWallSegment2d &segment : wallSegments_in)
    {
        const Eigen::Vector2d midpointGround_m =
            0.5 * (segment.start_world_m + segment.end_world_m) -
            roomCentroidGround_m_in;
        if (!midpointGround_m.allFinite() ||
            midpointGround_m.squaredNorm() < 1e-12)
        {
            continue;
        }
        midpointAngles_rad.push_back(
            std::atan2(midpointGround_m.y(), midpointGround_m.x()));
    }

    if (midpointAngles_rad.empty())
    {
        gaps.push_back({0.0, 2.0 * M_PI});
        roomObservationGaps_out = gaps;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::sort(midpointAngles_rad.begin(), midpointAngles_rad.end());

    for (std::size_t angleIndex = 0U; angleIndex < midpointAngles_rad.size();
         ++angleIndex)
    {
        const double thisAngle_rad = midpointAngles_rad[angleIndex];
        const double nextAngle_rad =
            (angleIndex + 1U < midpointAngles_rad.size())
                ? midpointAngles_rad[angleIndex + 1U]
                : midpointAngles_rad[0] + 2.0 * M_PI;
        const double span_rad = nextAngle_rad - thisAngle_rad;

        if (span_rad > gapThreshold_rad_in)
        {
            gaps.push_back({thisAngle_rad, span_rad});
        }
    }

    roomObservationGaps_out = gaps;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
