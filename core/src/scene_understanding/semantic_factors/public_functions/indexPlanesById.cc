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
 * @file            indexPlanesById.cc
 *
 * @brief           Implements indexPlanesById(), declared in
 * scene_understanding/semantic_factors/public_functions/public_functions.h.
 */

#include "scene_understanding/semantic_factors/public_functions/public_functions.h"

#include <rclcpp/logging.hpp>

#include "mapping/geometric/Plane/Plane.h"

namespace vs_graphs
{
namespace core
{

void indexPlanesById(
    const std::vector<geometric::Plane *>       &planes_in,
    std::unordered_map<int, geometric::Plane *> &planesById_out)
{
    planesById_out.clear();
    for (geometric::Plane *p_plane : planes_in)
    {
        int planeId{};
        if (p_plane->getId(planeId) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        planesById_out.emplace(planeId, p_plane);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
