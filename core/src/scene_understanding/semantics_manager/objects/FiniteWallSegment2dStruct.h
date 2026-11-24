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
 * @file            FiniteWallSegment2dStruct.h
 *
 * @brief           Declares one finite horizontal wall segment on the ground
 *                  plane.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_FINITE_WALL_SEGMENT_2D_STRUCT_H
#define VS_GRAPHS_SEMANTICS_MANAGER_FINITE_WALL_SEGMENT_2D_STRUCT_H

#include <Eigen/Core>

namespace vs_graphs
{
namespace core
{

namespace geometric
{
class Plane;
} /* namespace geometric */

/*!
 * @brief           Finite horizontal representation of one observed room wall.
 */
struct FiniteWallSegment2d
{
    /*!
     * @brief           Wall the segment was built from; non-owning.
     *
     * @frame           N/A
     *
     * @units           N/A
     */
    geometric::Plane *p_wall = nullptr;

    /*!
     * @brief           First end of the wall on the ground plane.
     *
     * @frame           World, projected onto the two ground axes
     *
     * @units           metres
     */
    Eigen::Vector2d wallStart_world_m = Eigen::Vector2d::Zero();

    /*!
     * @brief           Second end of the wall on the ground plane.
     *
     * @frame           World, projected onto the two ground axes
     *
     * @units           metres
     */
    Eigen::Vector2d wallEnd_world_m = Eigen::Vector2d::Zero();

    /*!
     * @brief           Distance between the two ends.
     *
     * @frame           N/A
     *
     * @units           metres
     */
    double length_m = 0.0;

    /*!
     * @brief           How well supported the wall is: observation count (at
     *                  least 1) times the square root of the length; larger
     *                  wins when walls compete.
     *
     * @frame           N/A
     *
     * @units           square root of metres
     */
    double supportScore = 0.0;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_FINITE_WALL_SEGMENT_2D_STRUCT_H */
