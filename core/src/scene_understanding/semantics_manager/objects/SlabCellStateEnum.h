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
 * @file            SlabCellStateEnum.h
 *
 * @brief           Declares the free-space state of one wall-slab raster cell.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_SLAB_CELL_STATE_ENUM_H
#define VS_GRAPHS_SEMANTICS_MANAGER_SLAB_CELL_STATE_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           What the Voxblox TSDF says about one wall-slab cell. Values
 *                  match voxblox_msgs/PlaneSlabProfile CELL_* and are never
 *                  reordered.
 */
enum class SlabCellState : std::uint8_t
{
    /*!
     * @brief           At least one sample of the cell was never observed and
     *                  none is behind a surface.
     */
    UNKNOWN = 0U,

    /*!
     * @brief           Every sample of the cell is observed empty space.
     */
    FREE = 1U,

    /*!
     * @brief           At least one observed sample is at or behind a surface.
     */
    OCCUPIED = 2U
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_SLAB_CELL_STATE_ENUM_H  */
