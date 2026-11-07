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
 * @file            WallBoundsStruct.h
 *
 * @brief           Declares WallBounds, the extent of one wall plane in its own
 * in-plane coordinates.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_OBJECTS_WALL_BOUNDS_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_OBJECTS_WALL_BOUNDS_STRUCT_H

#include <Eigen/Core>

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           Extent of one wall plane in its own in-plane (U, V)
 *                  coordinates, kept so rooms can be compared by wall size.
 */
struct WallBounds
{
    /*!
     * @brief           True when the four bounds below were finite and
     *                  span a non-empty rectangle at snapshot time.
     */
    bool   isValid{false};
    /*!
     * @brief           Smallest in-plane U coordinate of the wall points,
     *                  metres.
     */
    double minU_m{0.0};
    /*!
     * @brief           Largest in-plane U coordinate of the wall points,
     *                  metres.
     */
    double maxU_m{0.0};
    /*!
     * @brief           Smallest in-plane V coordinate of the wall points,
     *                  metres.
     */
    double minV_m{0.0};
    /*!
     * @brief           Largest in-plane V coordinate of the wall points,
     *                  metres.
     */
    double maxV_m{0.0};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_OBJECTS_WALL_BOUNDS_STRUCT_H */
