/**
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
 * @file            RoomBoundaryGeometryStatus.h
 *
 * @brief           Declares the outcome of checkRoomBoundaryGeometry()'s
 *                  structural (not semantic) validation of one room's
 *                  boundary polygon.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_ROOM_BOUNDARY_GEOMETRY_STATUS_H
#define SEMANTIC_AXIOM_EVALUATOR_ROOM_BOUNDARY_GEOMETRY_STATUS_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Structural validity of one RoomRecord::boundaryCorners_World_m
 *              polygon, independent of Room::BoundaryStatus and of any
 *              observation-gap/aperture correspondence (a separate,
 *              non-geometric check performed directly by
 *              evaluateAxBound01()).
 */
enum class RoomBoundaryGeometryStatus : std::uint8_t
{
    /*! @brief At least three corners, no zero-length consecutive edge, and
     *  no self-intersecting edge pair in the polygon's own best-fit
     *  plane. */
    VALID = 0U,

    /*! @brief Fewer than three corners. */
    TOO_FEW_CORNERS = 1U,

    /*! @brief Two consecutive corners coincide (a zero-length edge). */
    DEGENERATE_EDGE = 2U,

    /*! @brief Two non-adjacent edges of the polygon intersect. */
    SELF_INTERSECTING = 3U,

    /*! @brief At least one corner has a non-finite (NaN or Infinity)
     *  coordinate; no comparison-based check below is trusted to reject
     *  this on its own. */
    NON_FINITE_CORNER = 4U
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_ROOM_BOUNDARY_GEOMETRY_STATUS_H
