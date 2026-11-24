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
 * @file            findMapPlaneById.cc
 *
 * @brief           Implements findMapPlaneById(), declared in common.h.
 */

#include "common/common.h"

namespace vs_graphs::common
{

vs_graphs::core::geometric::Plane *
    findMapPlaneById(vs_graphs::core::map::Map *p_map_in, int planeId_in)
{
    if (p_map_in == nullptr)
    {
        return nullptr;
    }

    /* A wall can leave the map's plane set while a room still lists it, so
     * the room wall index is searched first. */
    vs_graphs::core::geometric::Plane *p_plane = nullptr;
    if (p_map_in->getRoomWallPlaneById(planeId_in, p_plane) !=
        vs_graphs::core::map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomWallPlaneById returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_plane == nullptr &&
        p_map_in->getPlaneById(planeId_in, p_plane) !=
            vs_graphs::core::map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneById returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    return p_plane;
}

} /* namespace vs_graphs::common */
