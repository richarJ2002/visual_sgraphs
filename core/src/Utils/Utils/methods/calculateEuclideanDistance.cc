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
 * @file            calculateEuclideanDistance.cc
 *
 * @brief           Implements Utils::calculateEuclideanDistance(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

double Utils::calculateEuclideanDistance(const Eigen::Vector3f &point1_in,
                                         const Eigen::Vector3f &point2_in)
{
    double dx = point1_in.x() - point2_in.x();
    double dy = point1_in.y() - point2_in.y();
    double dz = point1_in.z() - point2_in.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
