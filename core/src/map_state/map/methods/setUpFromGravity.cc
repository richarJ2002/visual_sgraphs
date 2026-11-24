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
 * @file            setUpFromGravity.cc
 *
 * @brief           Implements Map::setUpFromGravity(), declared in
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

MapStatus Map::setUpFromGravity(
    const Eigen::Matrix3d &gravityRotation_gravityToWorld_in)
{
    std::unique_lock<std::mutex> lock(mapMutex);

    /* Gravity in world is R * [0, 0, -g], so up is R * +Z. */
    upDirection_world =
        (gravityRotation_gravityToWorld_in * Eigen::Vector3d::UnitZ())
            .normalized();

    isUpFromImuGravity = true;

    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
