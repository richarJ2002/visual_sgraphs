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
 * @file            semanticsmanager.cc
 *
 * @brief           Implements the SemanticsManager constructor, declared in
 *                  SemanticsManager.h.
 */

#include "SemanticsManager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManager::SemanticsManager(Atlas *p_atlas_in)
{
    /* Store the address of the atlas map */
    p_atlas = p_atlas_in;

    /* Get the system parameters */
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_sysParams = p_params;

    /* Configure the room-tracking state machine. */
    semantic::RoomTrackerConfig trackerConfiguration;
    trackerConfiguration.crossing_dwell_s =
        static_cast<double>(p_sysParams->roomTracking.crossingDwell_s);
    trackerConfiguration.crossing_confidence =
        static_cast<double>(p_sysParams->roomTracking.crossingConfidence);
    trackerConfiguration.lost_timeout_s =
        static_cast<double>(p_sysParams->roomTracking.lostTimeout_s);
    trackerConfiguration.reacquire_timeout_s =
        static_cast<double>(p_sysParams->roomTracking.reacquireTimeout_s);
    trackerConfiguration.reacquire_retry_interval_s =
        static_cast<double>(p_sysParams->roomTracking.reacquireRetryInterval_s);
    trackerConfiguration.reacquire_max_retries =
        p_sysParams->roomTracking.reacquireMaxRetries;
    trackerConfiguration.reacquire_min_planes =
        p_sysParams->roomTracking.reacquireMinPlanes;
    roomTracker = semantic::RoomTracker(trackerConfiguration);
}

} // namespace core
} // namespace vs_graphs
