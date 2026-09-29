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

#include "LoopClosing.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

LoopClosingStatus collapseMergedFloors(Map *p_survivingMap_inout)
{
    if (p_survivingMap_inout == nullptr)
    {
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    std::vector<semantic::Floor *> allFloors{};
    if (p_survivingMap_inout->getAllFloors(allFloors) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (allFloors.size() <= 1U)
    {
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    semantic::Floor *p_keeperFloor = nullptr;
    if (semantic::Floor::selectBestObservedFloor(allFloors, p_keeperFloor) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_keeperFloor == nullptr)
    {
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
    }

    for (semantic::Floor *p_duplicateFloor : allFloors)
    {
        if (p_duplicateFloor == nullptr || p_duplicateFloor == p_keeperFloor)
        {
            continue;
        }

        std::vector<vs_graphs::core::semantic::Room *> duplicateFloorRooms{};
        if (p_duplicateFloor->getRooms(duplicateFloorRooms) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRooms returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Room *p_room : duplicateFloorRooms)
        {
            bool roomIsBad{};
            if ((p_room != nullptr) &&
                p_room->isBad(roomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room != nullptr && !roomIsBad)
            {
                if (p_keeperFloor->addRoom(p_room) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addRoom returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
        }

        if (p_survivingMap_inout->eraseMapFloor(p_duplicateFloor) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseMapFloor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        int duplicateFloorId{};
        if (p_duplicateFloor->getId(duplicateFloorId) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int keeperFloorId{};
        if (p_keeperFloor->getId(keeperFloorId) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[LoopClosing] Fused duplicate semantic::Floor#"
                  << duplicateFloorId << " into semantic::Floor#"
                  << keeperFloorId
                  << " and retained the better-observed plane identity."
                  << std::endl;
    }

    std::vector<semantic::Room *> survivingMapAllDetectedMapRooms{};
    if (p_survivingMap_inout->getAllDetectedMapRooms(
            survivingMapAllDetectedMapRooms) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllDetectedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    for (semantic::Room *p_room : survivingMapAllDetectedMapRooms)
    {
        bool roomIsBad2{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad2)
        {
            if (p_keeperFloor->addRoom(p_room) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addRoom returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
