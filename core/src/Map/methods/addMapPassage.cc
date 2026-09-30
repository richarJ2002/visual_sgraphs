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
 * @file            addMapPassage.cc
 *
 * @brief           Implements Map::addMapPassage(), declared in Map.h.
 */

#include "Map.h"
#include "Semantic/Passage.h"
#include "Semantic/PassageStatus.h"

#include <algorithm>
#include <iterator>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MapStatus vs_graphs::core::Map::addMapPassage(
    vs_graphs::core::semantic::Passage *p_passage_inout)
{
    if (p_passage_inout == nullptr)
    {
        return MapStatus::MAP_STATUS_SUCCESS;
    }

    std::unique_lock<std::mutex> lock(mapMutex);

    int passage_inoutId{};
    if (p_passage_inout->getId(passage_inoutId) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const std::unordered_map<int, semantic::Passage *>::iterator
        existingPassage = passageIndex.find(passage_inoutId);
    int passage_inoutId2{};
    if (p_passage_inout->getId(passage_inoutId2) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (passage_inoutId2 < 0 || (existingPassage != passageIndex.end() &&
                                 existingPassage->second != p_passage_inout))
    {
        int passage_inoutId3{};
        if (p_passage_inout->getId(passage_inoutId3) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cerr << "[Map] semantic::Passage ID collision for "
                  << passage_inoutId3
                  << "; caller must resolve it before destination insertion."
                  << std::endl;
        return MapStatus::MAP_STATUS_SUCCESS;
    }

    passages.insert(p_passage_inout);
    int passage_inoutId4{};
    if (p_passage_inout->getId(passage_inoutId4) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    passageIndex.insert_or_assign(passage_inoutId4, p_passage_inout);

    return MapStatus::MAP_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
