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
 * @file            computePointVoxelKey.cc
 *
 * @brief           Implements Map::computePointVoxelKey(), declared in
 *                  map/objects/MapClass.h.
 */

#include "map_state/map/objects/MapClass.h"

#include <cmath>
#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus
    Map::computePointVoxelKey(const Eigen::Vector3f &pointPosition_world_m_in,
                              double                 voxelSize_m_in,
                              std::int64_t          &voxelKey_out)
{
    /*!
     * 21 bits per axis: voxel indices wrap every 2^21 voxels (about 63 km at
     * 0.03 m), far beyond any mapped building.
     */
    constexpr std::int64_t AXIS_BITS = 21;
    constexpr std::int64_t AXIS_MASK = (std::int64_t{1} << AXIS_BITS) - 1;

    const std::int64_t voxelIndexX =
        static_cast<std::int64_t>(
            std::floor(static_cast<double>(pointPosition_world_m_in.x()) /
                       voxelSize_m_in)) &
        AXIS_MASK;
    const std::int64_t voxelIndexY =
        static_cast<std::int64_t>(
            std::floor(static_cast<double>(pointPosition_world_m_in.y()) /
                       voxelSize_m_in)) &
        AXIS_MASK;
    const std::int64_t voxelIndexZ =
        static_cast<std::int64_t>(
            std::floor(static_cast<double>(pointPosition_world_m_in.z()) /
                       voxelSize_m_in)) &
        AXIS_MASK;

    voxelKey_out = (voxelIndexX << (2 * AXIS_BITS)) |
                   (voxelIndexY << AXIS_BITS) | voxelIndexZ;
    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
