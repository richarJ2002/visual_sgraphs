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
 * @file            intersectSupportingLines.cc
 *
 * @brief           Implements intersectSupportingLines(), declared in
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

SemanticsManagerStatus
    intersectSupportingLines(const FiniteWallSegment2d &firstSegment_in,
                             const FiniteWallSegment2d &secondSegment_in,
                             Eigen::Vector2d &lineIntersection_world_m_out,
                             double          &firstParameter_out,
                             double          &secondParameter_out,
                             bool            &hasIntersection_out)
{
    const Eigen::Vector2d firstDirection =
        firstSegment_in.wallEnd_world_m - firstSegment_in.wallStart_world_m;
    const Eigen::Vector2d secondDirection =
        secondSegment_in.wallEnd_world_m - secondSegment_in.wallStart_world_m;
    double denominator{};
    if (crossProduct2d(firstDirection, secondDirection, denominator) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: crossProduct2d returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (!std::isfinite(denominator) || std::abs(denominator) < 1e-8)
    {
        hasIntersection_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const Eigen::Vector2d startOffset =
        secondSegment_in.wallStart_world_m - firstSegment_in.wallStart_world_m;
    double crossProduct{};
    if (crossProduct2d(startOffset, secondDirection, crossProduct) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: crossProduct2d returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    firstParameter_out = crossProduct / denominator;
    double crossProduct2{};
    if (crossProduct2d(startOffset, firstDirection, crossProduct2) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: crossProduct2d returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    secondParameter_out = crossProduct2 / denominator;
    lineIntersection_world_m_out =
        firstSegment_in.wallStart_world_m + firstParameter_out * firstDirection;

    hasIntersection_out = lineIntersection_world_m_out.allFinite() &&
                          std::isfinite(firstParameter_out) &&
                          std::isfinite(secondParameter_out);
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
