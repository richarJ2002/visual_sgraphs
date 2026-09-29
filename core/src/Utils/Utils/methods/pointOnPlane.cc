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
 * @file            pointOnPlane.cc
 *
 * @brief           Implements Utils::pointOnPlane(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::pointOnPlane(Eigen::Vector4d planeEquation_in,
                                MapPoint       *p_mapPoint_in,
                                bool           &isOnPlane_out)
{
    if (p_mapPoint_in->isBad())
    {
        isOnPlane_out = false;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    // Find the distance of the point from a given plane
    double pointPlaneDistance{};
    if (calculateDistancePointToPlane(
            planeEquation_in,
            p_mapPoint_in->getWorldPos().cast<double>(),
            pointPlaneDistance) != UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calculateDistancePointToPlane returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    // Apply a threshold
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (pointPlaneDistance < p_params->seg.planePointDistThresh)
    {
        isOnPlane_out = true;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    isOnPlane_out = false;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
