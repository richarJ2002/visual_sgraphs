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
 * @file            WallSlabQueryStruct.h
 *
 * @brief           Declares one wall-slab raster request sent to Voxblox.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_WALL_SLAB_QUERY_STRUCT_H
#define VS_GRAPHS_SEMANTICS_MANAGER_WALL_SLAB_QUERY_STRUCT_H

#include <Eigen/Core>
#include <cstdint>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           A thin box around one wall plane, rasterised into square
 *                  cells. Cell (column, row) is centred at origin +
 *                  (column + 0.5) * cellSize * alongAxis + (row + 0.5) *
 *                  cellSize * upAxis and sampled along normal within
 *                  +-halfThickness. All vectors are in the SLAM world frame.
 */
struct WallSlabQuery
{
  public:
    /*!
     * @brief           Id of the wall plane the slab was built around.
     */
    int wallId{-1};

    /*!
     * @brief           Wall plane (unit normal, offset) in the world frame when
     *                  the query was built; a profile is used only while the
     *                  wall still matches it.
     */
    Eigen::Vector4d wallEquation_world{Eigen::Vector4d::Zero()};

    /*!
     * @brief           Corner of cell (0, 0) on the wall plane at floor height,
     *                  world frame, metres.
     */
    Eigen::Vector3d origin_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Unit horizontal direction along the wall (columns),
     *                  world frame.
     */
    Eigen::Vector3d alongAxis_world{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Unit in-plane direction pointing up (rows), world
     *                  frame.
     */
    Eigen::Vector3d upAxis_world{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Unit wall normal, world frame.
     */
    Eigen::Vector3d normal_world{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Number of cells along the wall.
     */
    std::uint32_t columnCount{0U};

    /*!
     * @brief           Number of cells up the wall, starting at the floor.
     */
    std::uint32_t rowCount{0U};

    /*!
     * @brief           Edge length of one square cell, metres.
     */
    double cellSize_m{0.0};

    /*!
     * @brief           Half thickness of the slab across the wall, metres.
     */
    double halfThickness_m{0.0};
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_WALL_SLAB_QUERY_STRUCT_H */
