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
 * @file            WallLoopClosureStruct.h
 *
 * @brief           Declares the result of closing wall segments into a loop.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_WALL_LOOP_CLOSURE_STRUCT_H
#define VS_GRAPHS_SEMANTICS_MANAGER_WALL_LOOP_CLOSURE_STRUCT_H

#include <Eigen/Core>
#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Result of attempting to close a set of wall segments into
 *                  one ordered, non-open loop.
 */
struct WallLoopClosure
{
    /*!
     * @brief           True when the walls do not form a closed loop (the
     *                  default); false when every neighbouring pair of walls
     *                  meets within the allowed corner gap.
     */
    bool hasOpenBoundary = true;

    /*!
     * @brief           Corners of the closed loop in the order of the walls
     *                  around the room, in the world frame projected onto the
     *                  two ground axes, metres; empty when the boundary is
     *                  open.
     */
    std::vector<Eigen::Vector2d> loopCorners_world_m;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_WALL_LOOP_CLOSURE_STRUCT_H */
