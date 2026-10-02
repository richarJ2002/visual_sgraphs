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
 * @file            computePolygonArea_m2.cc
 *
 * @brief           Implements computePolygonArea_m2(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "SemanticsManager.h"

#include "private_functions.h"

#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus computePolygonArea_m2(
    const std::vector<Eigen::Vector2d> &polygonVertices_world_m_in,
    double                             &polygonArea_m2_out)
{
    if (polygonVertices_world_m_in.size() < 3U)
    {
        polygonArea_m2_out = 0.0;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    double signedTwiceArea_m2 = 0.0;

    for (std::size_t vertexIndex = 0U;
         vertexIndex < polygonVertices_world_m_in.size();
         ++vertexIndex)
    {
        const Eigen::Vector2d &currentVertex =
            polygonVertices_world_m_in[vertexIndex];
        const Eigen::Vector2d &nextVertex =
            polygonVertices_world_m_in[(vertexIndex + 1U) %
                                       polygonVertices_world_m_in.size()];
        double crossProduct{};
        if (crossProduct2d(currentVertex, nextVertex, crossProduct) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: crossProduct2d returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        signedTwiceArea_m2 += crossProduct;
    }

    polygonArea_m2_out = 0.5 * std::abs(signedTwiceArea_m2);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
