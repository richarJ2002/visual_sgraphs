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
 * @file            createNewMapWhileAtlasLocked.cc
 *
 * @brief           Implements Atlas::createNewMapWhileAtlasLocked(), declared
 *                  in Atlas.h.
 */

#include "Atlas.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

AtlasStatus Atlas::createNewMapWhileAtlasLocked()
{
    std::cout << "\n[Atlas]" << std::endl;
    std::cout << "- Creating a new map (MapId: " << Map::nextId
              << ", Init KeyFrame: " << lastInitKeyFrameId << ") ..."
              << std::endl;

    if (p_activeMap)
    {
        unsigned long activeMapMaxKeyFrameId{};
        if ((!maps.empty()) &&
            p_activeMap->getMaxKeyFrameId(activeMapMaxKeyFrameId) !=
                MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMaxKeyFrameId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!maps.empty() && lastInitKeyFrameId < activeMapMaxKeyFrameId)
        {
            unsigned long activeMapMaxKeyFrameId2{};
            if (p_activeMap->getMaxKeyFrameId(activeMapMaxKeyFrameId2) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMaxKeyFrameId returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            lastInitKeyFrameId = activeMapMaxKeyFrameId2 + 1;
        }

        /* Snapshot room geometry before the map is stranded so that
         * rooms in the new map can inherit identity tags.            */
        if (exportRoomContextFromCurrentMap() !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: exportRoomContextFromCurrentMap returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        if (p_activeMap->setStoredMap() != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setStoredMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long activeMapId{};
        if (p_activeMap->getId(activeMapId) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "- The created map with MapId #" << activeMapId
                  << " has been stored!" << std::endl;
    }

    Map *p_previousMap = p_activeMap;
    p_activeMap        = new Map(lastInitKeyFrameId);
    if (p_activeMap->setCurrentMap() != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    maps.insert(p_activeMap);
    if (p_previousMap != nullptr)
    {
        /* Mission-chain trace link: the stranded map points at its
         * successor. Same-map clears never pass through here, so the link
         * stays null for them by construction. */
        if (p_previousMap->setFollowingMap(p_activeMap) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setFollowingMap returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    {
        std::lock_guard<std::mutex> contextLock(roomContextMutex);
        isNewMapCreatedPending = true;
    }

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
