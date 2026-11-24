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
 * @file            rebuildPointDensity.cc
 *
 * @brief           Implements Map::rebuildPointDensity(), declared in
 *                  map/objects/MapClass.h.
 */

#include "map_state/map/objects/MapClass.h"
#include "mapping/geometric/Plane/Plane.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace map
{

MapStatus Map::rebuildPointDensity(void)
{
    {
        std::lock_guard<std::mutex> lock(pointDensityMutex);
        pointCountByVoxel.clear();
        voxelKeysByPlane.clear();
        ++densityRevision;
    }

    /* No map lock is held here: each plane takes its own locks first and the
     * density lock last. */
    std::vector<geometric::Plane *> mapPlanes{};
    if (getAllPlanes(mapPlanes) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *p_mapPlane : mapPlanes)
    {
        if (p_mapPlane != nullptr &&
            p_mapPlane->reapplyDensityCap() !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reapplyDensityCap returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return MapStatus::MAP_STATUS_SUCCESS;
}

} /* namespace map */
} /* namespace core */
} /* namespace vs_graphs */
