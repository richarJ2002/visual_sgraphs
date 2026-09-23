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

#include "SemanticsManager.h"

namespace vs_graphs
{
namespace core
{

SemanticsManager::SemanticsManager(Atlas *pAtlas)
{
    /* Store the address of the atlas map */
    p_atlas = pAtlas;

    /* Get the system parameters */
    p_sysParams = types::SystemParams::getParams();

    /* Configure the room-tracking state machine. */
    semantic::RoomTrackerConfig trackerConfig;
    trackerConfig.crossing_dwell_s =
        static_cast<double>(p_sysParams->roomTracking.crossingDwell_s);
    trackerConfig.crossing_confidence =
        static_cast<double>(p_sysParams->roomTracking.crossingConfidence);
    trackerConfig.lost_timeout_s =
        static_cast<double>(p_sysParams->roomTracking.lostTimeout_s);
    trackerConfig.reacquire_timeout_s =
        static_cast<double>(p_sysParams->roomTracking.reacquireTimeout_s);
    trackerConfig.reacquire_retry_interval_s =
        static_cast<double>(p_sysParams->roomTracking.reacquireRetryInterval_s);
    trackerConfig.reacquire_max_retries =
        p_sysParams->roomTracking.reacquireMaxRetries;
    trackerConfig.reacquire_min_planes =
        p_sysParams->roomTracking.reacquireMinPlanes;
    roomTracker_ = semantic::RoomTracker(trackerConfig);
}

} // namespace core
} // namespace vs_graphs
