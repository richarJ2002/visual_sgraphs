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
 * @file            setPlaneIdentity.cc
 *
 * @brief           Implements Floor::setPlaneIdentity().
 */

#include "Semantic/Floor.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

FloorStatus
    Floor::setPlaneIdentity(const Eigen::Vector4d &planeEquation_world_in,
                            const std::size_t      finiteSupportCount_in,
                            const std::size_t      observationCount_in)
{
    Eigen::Vector4d normalizedEquation_world = planeEquation_world_in;
    const double    normalNorm = normalizedEquation_world.head<3>().norm();

    if (!normalizedEquation_world.allFinite() || !std::isfinite(normalNorm) ||
        normalNorm < 1e-8)
    {
        return FloorStatus::FLOOR_STATUS_INVALID_ARGUMENT;
    }

    normalizedEquation_world /= normalNorm;

    std::lock_guard<std::mutex> lock(geometryMutex);
    planeIdentity = PlaneIdentity{normalizedEquation_world,
                                  finiteSupportCount_in,
                                  observationCount_in};
    return FloorStatus::FLOOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
