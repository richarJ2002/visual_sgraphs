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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

const std::vector<semantic::TransitionEvent> &
    SemanticsManager::getRoomTrackerEventHistoryForTest() const
{
    const std::vector<semantic::TransitionEvent> *p_roomTrackerEventHistory =
        nullptr;
    if (roomTracker.getEventHistory(p_roomTrackerEventHistory) !=
        semantic::RoomTrackerStatus::ROOM_TRACKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getEventHistory returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    return (*p_roomTrackerEventHistory);
}

} // namespace core
} // namespace vs_graphs
