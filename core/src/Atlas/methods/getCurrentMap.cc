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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

AtlasStatus Atlas::getCurrentMap(Map *&p_currentMap_out)
{
    std::unique_lock<std::mutex> atlasLock(atlasMutex);

    if (!p_activeMap)
    {
        if (createNewMapWhileAtlasLocked() != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: createNewMapWhileAtlasLocked returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }

    for (;;)
    {
        bool activeMapIsBad{};
        if ((p_activeMap != nullptr) &&
            p_activeMap->isBad(activeMapIsBad) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!(p_activeMap != nullptr && activeMapIsBad))
        {
            break;
        }
        /* Allow ChangeMap() to install the merge survivor while waiting. */
        atlasLock.unlock();
        usleep(3000);
        atlasLock.lock();
    }

    if (p_activeMap == nullptr)
    {
        if (createNewMapWhileAtlasLocked() != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: createNewMapWhileAtlasLocked returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }

    p_currentMap_out = p_activeMap;
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
