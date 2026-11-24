/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
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
 * @file            getUpDirection.cc
 *
 * @brief           Implements Atlas::getUpDirection(), declared in
 *                  atlas/objects/AtlasClass.h.
 */

#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::getUpDirection(Eigen::Vector3d &upDirection_world_out,
                                  bool            &isFromImuGravity_out)
{
    std::unique_lock<std::mutex> lock(atlasMutex);

    /* Plain RGB-D keeps the map-start camera's -Y and never reads IMU state. */
    upDirection_world_out = Eigen::Vector3d(0.0, -1.0, 0.0);
    isFromImuGravity_out  = false;
    bool activeMapIsInertial{};
    if (p_activeMap != nullptr &&
        p_activeMap->isInertial(activeMapIsInertial) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isInertial returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (activeMapIsInertial &&
        p_activeMap->getUpDirection(upDirection_world_out,
                                    isFromImuGravity_out) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getUpDirection returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
