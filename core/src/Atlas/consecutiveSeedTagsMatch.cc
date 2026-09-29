/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "Atlas.h"

#include "private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Room-prior seed: the old final room and the new starting
 *               room must carry the same non-empty tag. A silent mismatch
 *               means no prior link (e.g. loop closure between
 *               non-consecutive maps): not this path's job.
 */
AtlasStatus consecutiveSeedTagsMatch(Map  *p_oldMap_in,
                                     Map  *p_currentMap_in,
                                     bool &isMatch_out)
{
    if (p_oldMap_in == nullptr || p_currentMap_in == nullptr)
    {
        isMatch_out = false;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
    semantic::Room *p_oldFinalRoom = nullptr;
    if (p_oldMap_in->getFinalRoom(p_oldFinalRoom) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getFinalRoom returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Room *p_newStartRoom = nullptr;
    if (p_currentMap_in->getStartingRoom(p_newStartRoom) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getStartingRoom returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    bool oldFinalRoomHasRoomTag{};
    if ((p_oldFinalRoom != nullptr && p_newStartRoom != nullptr) &&
        p_oldFinalRoom->hasRoomTag(oldFinalRoomHasRoomTag) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasRoomTag returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool newStartRoomHasRoomTag{};
    if ((p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
         oldFinalRoomHasRoomTag) &&
        p_newStartRoom->hasRoomTag(newStartRoomHasRoomTag) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasRoomTag returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::string oldFinalRoomRoomTag{};
    if ((p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
         oldFinalRoomHasRoomTag && newStartRoomHasRoomTag) &&
        p_oldFinalRoom->getRoomTag(oldFinalRoomRoomTag) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomTag returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::string oldFinalRoomRoomTag2{};
    if ((p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
         oldFinalRoomHasRoomTag && newStartRoomHasRoomTag &&
         !oldFinalRoomRoomTag.empty()) &&
        p_oldFinalRoom->getRoomTag(oldFinalRoomRoomTag2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomTag returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::string newStartRoomRoomTag{};
    if ((p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
         oldFinalRoomHasRoomTag && newStartRoomHasRoomTag &&
         !oldFinalRoomRoomTag.empty()) &&
        p_newStartRoom->getRoomTag(newStartRoomRoomTag) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomTag returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    isMatch_out = p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
                  oldFinalRoomHasRoomTag && newStartRoomHasRoomTag &&
                  !oldFinalRoomRoomTag.empty() &&
                  oldFinalRoomRoomTag2 == newStartRoomRoomTag;
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
