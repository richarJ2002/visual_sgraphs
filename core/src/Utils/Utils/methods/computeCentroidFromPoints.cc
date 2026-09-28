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
 * @file            computeCentroidFromPoints.cc
 *
 * @brief           Implements Utils::computeCentroidFromPoints(), declared in
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

UtilsStatus Utils::computeCentroidFromPoints(
    const std::vector<Eigen::Vector3d> &points_in,
    Eigen::Vector3d                    &centroid_out)
{
    // Check if there are points_in in the vector
    if (points_in.empty())
    {
        centroid_out = Eigen::Vector3d(0.0, 0.0, 0.0);
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    // Variables
    Eigen::Vector3d sum(0.0, 0.0, 0.0);

    // Calculate the sum of the points_in
    for (const auto &point : points_in)
        sum += point;

    // Return the centroid of the cluster
    centroid_out = sum / points_in.size();
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
