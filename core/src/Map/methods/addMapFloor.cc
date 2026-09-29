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

#include "Map.h"
#include "Semantic/Floor.h"
#include "Semantic/FloorStatus.h"

#include <algorithm>
#include <iterator>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void Map::addMapFloor(semantic::Floor *p_floor_inout)
{
    if (p_floor_inout == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mapMutex);

    for (auto floorIterator = floorIndex.begin();
         floorIterator != floorIndex.end();)
    {
        int floor_inoutId{};
        if ((floorIterator->second == p_floor_inout) &&
            p_floor_inout->getId(floor_inoutId) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        floorIterator = floorIterator->second == p_floor_inout &&
                                floorIterator->first != floor_inoutId
                            ? floorIndex.erase(floorIterator)
                            : std::next(floorIterator);
    }

    int floor_inoutId2{};
    if (p_floor_inout->getId(floor_inoutId2) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const auto existingFloorIterator = floorIndex.find(floor_inoutId2);

    int floor_inoutId3{};
    if (p_floor_inout->getId(floor_inoutId3) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (floor_inoutId3 < 0 || (existingFloorIterator != floorIndex.end() &&
                               existingFloorIterator->second != p_floor_inout))
    {
        while (floorIndex.count(nextAvailableFloorId) > 0)
        {
            ++nextAvailableFloorId;
        }

        const int replacementFloorId = nextAvailableFloorId++;

        int floor_inoutId4{};
        if (p_floor_inout->getId(floor_inoutId4) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cerr << "[Map] semantic::Floor ID collision for " << floor_inoutId4
                  << "; reassigned to " << replacementFloorId << "."
                  << std::endl;

        if (p_floor_inout->setId(replacementFloorId) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }
    else
    {
        int floor_inoutId5{};
        if (p_floor_inout->getId(floor_inoutId5) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        nextAvailableFloorId =
            std::max(nextAvailableFloorId, floor_inoutId5 + 1);
    }

    floors.insert(p_floor_inout);
    int floor_inoutId6{};
    if (p_floor_inout->getId(floor_inoutId6) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    floorIndex.insert_or_assign(floor_inoutId6, p_floor_inout);
}

} // namespace core
} // namespace vs_graphs
