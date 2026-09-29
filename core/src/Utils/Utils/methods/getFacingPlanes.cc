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
 * @file            getFacingPlanes.cc
 *
 * @brief           Implements Utils::getFacingPlanes(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::getFacingPlanes(
    const std::vector<vs_graphs::core::geometric::Plane *> &planes_in,
    std::vector<std::pair<vs_graphs::core::geometric::Plane *,
                          vs_graphs::core::geometric::Plane *>>
        &facingPlanes_out)
{
    // Variables
    vs_graphs::core::types::SystemParams *p_sysParams = nullptr;
    if (vs_graphs::core::types::SystemParams::getParams(p_sysParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<std::pair<vs_graphs::core::geometric::Plane *,
                          vs_graphs::core::geometric::Plane *>>
           facingPlanes;
    double minimumValidSpace = p_sysParams->roomSeg.minWallDistanceThresh;

    // Loop through all the planes_in
    for (size_t index1 = 0; index1 < planes_in.size(); ++index1)
    {
        vs_graphs::core::geometric::Plane *p_plane1 = planes_in[index1];
        for (size_t index2 = index1 + 1; index2 < planes_in.size(); ++index2)
        {
            // Variables
            vs_graphs::core::geometric::Plane *p_plane2 = planes_in[index2];
            // Check if the planes_in are facing each other
            bool                               isFacing{};
            if (Utils::arePlanesFacingEachOther(p_plane1, p_plane2, isFacing) !=
                UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: arePlanesFacingEachOther returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (isFacing)
            {
                bool arePlanesApartEnough2{};
                if (Utils::arePlanesApartEnough(p_plane1,
                                                p_plane2,
                                                minimumValidSpace,
                                                arePlanesApartEnough2) !=
                    UtilsStatus::UTILS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: arePlanesApartEnough returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (arePlanesApartEnough2)
                    facingPlanes.push_back(std::make_pair(p_plane1, p_plane2));
            }
        }
    }
    facingPlanes_out = facingPlanes;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
