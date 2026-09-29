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

#include "GeoSemHelpers.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

GeoSemHelpersStatus GeoSemHelpers::checkIfMarkerIsDoorway(
    const int                                     &markerId_in,
    std::vector<vs_graphs::core::semantic::Room *> envRooms_in,
    std::pair<bool, std::string>                  &doorwayMatch_out)
{
    bool        isDoorway = true;
    std::string name      = "";
    // Loop over all markers attached to doorways
    for (const auto &room : envRooms_in)
    {
        int roomMetaMarkerId{};
        if (room->getMetaMarkerId(roomMetaMarkerId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMetaMarkerId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (roomMetaMarkerId == markerId_in)
        {
            isDoorway = false;
            std::string roomName{};
            if (room->getName(roomName) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getName returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            name = roomName;
            break; // No need to continue searching if found
        }
    }
    // Returning
    doorwayMatch_out = std::make_pair(isDoorway, name);
    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
