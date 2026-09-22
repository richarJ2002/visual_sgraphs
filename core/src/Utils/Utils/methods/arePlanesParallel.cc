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
 * @file            arePlanesParallel.cc
 *
 * @brief           Implements Utils::arePlanesParallel(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <algorithm>
#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

bool Utils::arePlanesParallel(
    const vs_graphs::core::geometric::Plane *p_plane1_in,
    const vs_graphs::core::geometric::Plane *p_plane2_in)
{
    // Get the threshold value
    double threshold =
        types::SystemParams::getParams()->roomSeg.wallsParallelismThresh *
        Utils::DEG_TO_RAD;

    // Extract and normalize plane normals
    Eigen::Vector3d normal1 =
        p_plane1_in->getGlobalEquation().normal().normalized();
    Eigen::Vector3d normal2 =
        p_plane2_in->getGlobalEquation().normal().normalized();

    // Compute the dot product (clamped to avoid floating-point domain errors)
    double dotProduct = std::clamp(std::abs(normal1.dot(normal2)), -1.0, 1.0);

    // Compute the angle between normals in radians
    double angle = std::acos(dotProduct);

    // Planes are parallel if their normals are within threshold (0° or 180°)
    return (angle < threshold) || (std::abs(angle - M_PI) < threshold);
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
