/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
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
 * @file            getUpDirection.cc
 *
 * @brief           Implements Map::getUpDirection(), declared in
 *                  map/objects/MapClass.h.
 */

#include "map_state/map/objects/MapClass.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus Map::getUpDirection(Eigen::Vector3d &upDirection_world_out,
                              bool            &isFromImuGravity_out)
{
    std::unique_lock<std::mutex> lock(mapMutex);
    upDirection_world_out = upDirection_world;
    isFromImuGravity_out  = isUpFromImuGravity;
    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
