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
 * @file            getAllMaps.cc
 *
 * @brief           Implements Atlas::getAllMaps(), declared in
 * atlas/objects/AtlasClass.h, with CompFunctor::operator()().
 */

#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::getAllMaps(std::vector<map::Map *> &allMaps_out)
{
    std::unique_lock<std::mutex> lock(atlasMutex);
    /* maps is keyed by id, so this copy is already in ascending id order. */
    std::vector<map::Map *>      mapList{};
    mapList.reserve(maps.size());
    for (const std::pair<const unsigned long, std::unique_ptr<map::Map>>
             &mapEntry : maps)
    {
        mapList.push_back(mapEntry.second.get());
    }
    allMaps_out = mapList;
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
