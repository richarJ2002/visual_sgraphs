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
 * @file            isRoomRecordLessFullGeometry.cc
 *
 * @brief           Implements isRoomRecordLessFullGeometry(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isRoomRecordLessFullGeometry(const RoomRecord &lhs_in,
                                  const RoomRecord &rhs_in)
{
    /* Defer to the topology-only order first; only when it is silent (every
     * topology-only field equal) do the geometric fields below become the
     * tiebreak. This composition -- rather than duplicating the
     * topology-only field chain here -- is what keeps
     * isRoomRecordLessTopologyOnly() the single source of truth for which
     * fields are topology. */
    if (isRoomRecordLessTopologyOnly(lhs_in, rhs_in))
    {
        return true;
    }
    if (isRoomRecordLessTopologyOnly(rhs_in, lhs_in))
    {
        return false;
    }

    if (isVector3dLess(lhs_in.centroid_world_m, rhs_in.centroid_world_m))
    {
        return true;
    }
    if (isVector3dLess(rhs_in.centroid_world_m, lhs_in.centroid_world_m))
    {
        return false;
    }

    if (lhs_in.boundaryCorners_world_m.size() !=
        rhs_in.boundaryCorners_world_m.size())
    {
        return lhs_in.boundaryCorners_world_m.size() <
               rhs_in.boundaryCorners_world_m.size();
    }
    for (std::size_t boundaryCornerIndex = 0U;
         boundaryCornerIndex < lhs_in.boundaryCorners_world_m.size();
         ++boundaryCornerIndex)
    {
        if (isVector3dLess(lhs_in.boundaryCorners_world_m[boundaryCornerIndex],
                           rhs_in.boundaryCorners_world_m[boundaryCornerIndex]))
        {
            return true;
        }
        if (isVector3dLess(rhs_in.boundaryCorners_world_m[boundaryCornerIndex],
                           lhs_in.boundaryCorners_world_m[boundaryCornerIndex]))
        {
            return false;
        }
    }

    if (lhs_in.observationGaps.size() != rhs_in.observationGaps.size())
    {
        return lhs_in.observationGaps.size() < rhs_in.observationGaps.size();
    }
    for (std::size_t boundaryCornerIndex = 0U;
         boundaryCornerIndex < lhs_in.observationGaps.size();
         ++boundaryCornerIndex)
    {
        const Room::ObservationGap &lhsGap =
            lhs_in.observationGaps[boundaryCornerIndex];
        const Room::ObservationGap &rhsGap =
            rhs_in.observationGaps[boundaryCornerIndex];
        if (isDoubleLess(lhsGap.startAngle_rad, rhsGap.startAngle_rad))
        {
            return true;
        }
        if (isDoubleLess(rhsGap.startAngle_rad, lhsGap.startAngle_rad))
        {
            return false;
        }
        if (isDoubleLess(lhsGap.spanAngle_rad, rhsGap.spanAngle_rad))
        {
            return true;
        }
        if (isDoubleLess(rhsGap.spanAngle_rad, lhsGap.spanAngle_rad))
        {
            return false;
        }
    }

    return false;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
