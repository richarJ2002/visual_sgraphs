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
 * @file            lineIntersectsPlane.cc
 *
 * @brief           Implements Utils::lineIntersectsPlane(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

Eigen::Vector3d Utils::lineIntersectsPlane(const Eigen::Vector4d &plane_in,
                                           const Eigen::Vector3d &lineStart_in,
                                           const Eigen::Vector3d &lineEnd_in)
{
    // Calculate the direction vector of the line
    Eigen::Vector3d lineDirection = lineEnd_in - lineStart_in;

    // [TODO] - check if the line is parallel to the plane_in

    // Calculate the intersection point
    double t = -(plane_in.head<3>().dot(lineStart_in) + plane_in(3)) /
               plane_in.head<3>().dot(lineDirection);
    return lineStart_in + t * lineDirection;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
