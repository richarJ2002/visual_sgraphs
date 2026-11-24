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
 * @file            WallSlabProfileStruct.h
 *
 * @brief           Declares the Voxblox answer to one wall-slab query.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_WALL_SLAB_PROFILE_STRUCT_H
#define VS_GRAPHS_SEMANTICS_MANAGER_WALL_SLAB_PROFILE_STRUCT_H

#include <cstdint>
#include <vector>

#include "scene_understanding/semantics_manager/objects/SlabCellStateEnum.h"
#include "scene_understanding/semantics_manager/objects/WallSlabQueryStruct.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Free/occupied/unknown raster of one wall slab, row-major
 *                  (index = row * columnCount + column).
 */
struct WallSlabProfile
{
  public:
    /*!
     * @brief           The query this profile answers.
     */
    WallSlabQuery query{};

    /*!
     * @brief           State of each cell.
     */
    std::vector<SlabCellState> cellStates{};

    /*!
     * @brief           Smallest observed TSDF distance of each cell, metres;
     *                  NaN when no sample was observed.
     */
    std::vector<float> cellDistances_m{};

    /*!
     * @brief           Node-clock time the request was sent, nanoseconds;
     *                  orders profiles so each is used once.
     */
    std::int64_t profileStamp_ns{0};
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_WALL_SLAB_PROFILE_STRUCT_H */
