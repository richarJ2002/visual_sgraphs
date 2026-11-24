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
 * @file            PlaneGapStruct.h
 *
 * @brief           Declares PlaneGap, how far one plane lies from another.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_PLANE_GAP_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_PLANE_GAP_STRUCT_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Angle and offset between two planes in the same frame.
 */
struct PlaneGap
{
  public:
    /*!
     * @brief           Angle between the plane normals, degrees in [0, 180];
     *                  infinite when either plane is degenerate.
     */
    double normalAngle_deg{0.0};
    /*!
     * @brief           Distance from the reference point on the first plane to
     *                  the second plane, metres; infinite when either plane is
     *                  degenerate.
     */
    double offset_m{0.0};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_PLANE_GAP_STRUCT_H \
        */
