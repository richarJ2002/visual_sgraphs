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
 * @file            RoomWallFootprintStruct.h
 *
 * @brief           Declares the floor-plane box spanned by a room's walls.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_ROOM_WALL_FOOTPRINT_STRUCT_H
#define VS_GRAPHS_SEMANTICS_MANAGER_ROOM_WALL_FOOTPRINT_STRUCT_H

#include <Eigen/Core>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Box on the floor plane that holds every wall-extent corner
 *                  of one room, aligned to the room's dominant wall normal.
 *                  A point p lies at (p . firstAxis_world,
 *                  p . secondAxis_world) in box coordinates.
 */
struct RoomWallFootprint
{
  public:
    /*!
     * @brief           Horizontal normal of the room's longest wall; first box
     *                  axis, unit length.
     *
     * @frame           World
     *
     * @units           N/A
     */
    Eigen::Vector3d firstAxis_world{Eigen::Vector3d::UnitX()};

    /*!
     * @brief           Second box axis: up cross firstAxis_world, unit length.
     *
     * @frame           World
     *
     * @units           N/A
     */
    Eigen::Vector3d secondAxis_world{Eigen::Vector3d::UnitZ()};

    /*!
     * @brief           Box limits along firstAxis_world.
     *
     * @frame           World, projected onto firstAxis_world
     *
     * @units           metres
     */
    double minimumFirst_m{0.0};
    double maximumFirst_m{0.0};

    /*!
     * @brief           Box limits along secondAxis_world.
     *
     * @frame           World, projected onto secondAxis_world
     *
     * @units           metres
     */
    double minimumSecond_m{0.0};
    double maximumSecond_m{0.0};
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_ROOM_WALL_FOOTPRINT_STRUCT_H */
