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

/*!
 * @file            transferFloors.cc
 *
 * @brief           Implements transferFloors(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantic/floor/floor.h"
#include "scene_understanding/semantic/floor/objects/FloorStatusEnum.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void transferFloors(map::Map                             *p_currentMap_inout,
                    map::Map                             *p_mergeMap_inout,
                    const std::vector<semantic::Floor *> &floors_in)
{
    for (semantic::Floor *p_floor : floors_in)
    {
        if (p_floor == nullptr)
        {
            continue;
        }

        if (p_mergeMap_inout->eraseMapFloor(p_floor) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseMapFloor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Floor               *p_retainedFloor = nullptr;
        std::vector<semantic::Floor *> currentMapAllFloors{};
        if (p_currentMap_inout->getAllFloors(currentMapAllFloors) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllFloors returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Floor *p_existingFloor : currentMapAllFloors)
        {
            int existingFloorId{};
            if ((p_existingFloor != nullptr) &&
                p_existingFloor->getId(existingFloorId) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            int floorId{};
            if ((p_existingFloor != nullptr) &&
                p_floor->getId(floorId) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_existingFloor != nullptr && existingFloorId == floorId)
            {
                p_retainedFloor = p_existingFloor;
                break;
            }
        }
        if (p_retainedFloor != nullptr)
        {
            if (mergeFloorEvidenceAndRooms(p_retainedFloor, p_floor) !=
                loop_closing::LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: mergeFloorEvidenceAndRooms returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            continue;
        }
        if (p_floor->setMap(p_currentMap_inout) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap_inout->addMapFloor(p_floor) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapFloor returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    if (collapseMergedFloors(p_currentMap_inout) !=
        loop_closing::LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collapseMergedFloors returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
