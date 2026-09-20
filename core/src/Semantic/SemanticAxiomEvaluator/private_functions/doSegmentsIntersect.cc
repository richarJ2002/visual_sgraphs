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
 * @file            doSegmentsIntersect.cc
 *
 * @brief           Implements doSegmentsIntersect(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool doSegmentsIntersect(const Eigen::Vector2d &p1_in,
                         const Eigen::Vector2d &q1_in,
                         const Eigen::Vector2d &p2_in,
                         const Eigen::Vector2d &q2_in)
{
    const double orientation1 = orientation2d(p1_in, q1_in, p2_in);
    const double orientation2 = orientation2d(p1_in, q1_in, q2_in);
    const double orientation3 = orientation2d(p2_in, q2_in, p1_in);
    const double orientation4 = orientation2d(p2_in, q2_in, q1_in);

    if (((orientation1 > 0.0) != (orientation2 > 0.0)) &&
        ((orientation3 > 0.0) != (orientation4 > 0.0)) && orientation1 != 0.0 &&
        orientation2 != 0.0 && orientation3 != 0.0 && orientation4 != 0.0)
    {
        return true;
    }

    if (orientation1 == 0.0 && isOnSegmentBoundingBox(p1_in, q1_in, p2_in))
    {
        return true;
    }
    if (orientation2 == 0.0 && isOnSegmentBoundingBox(p1_in, q1_in, q2_in))
    {
        return true;
    }
    if (orientation3 == 0.0 && isOnSegmentBoundingBox(p2_in, q2_in, p1_in))
    {
        return true;
    }
    if (orientation4 == 0.0 && isOnSegmentBoundingBox(p2_in, q2_in, q1_in))
    {
        return true;
    }
    return false;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
