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
 * @file            setKnownSideDirection.cc
 *
 * @brief           Implements Passage::setKnownSideDirection(), declared in
 *                  Semantic/Passage.h.
 */

#include "Semantic/Passage.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

PassageStatus
    Passage::setKnownSideDirection(const Eigen::Vector3d &direction_world_in)
{
    const double directionNorm = direction_world_in.norm();
    if (!direction_world_in.allFinite() || !std::isfinite(directionNorm) ||
        directionNorm < 1e-8)
    {
        return PassageStatus::PASSAGE_STATUS_INVALID_ARGUMENT;
    }

    std::lock_guard<std::mutex> lock(geometryMutex);
    knownSideProvenance.direction_world = direction_world_in / directionNorm;
    return PassageStatus::PASSAGE_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
