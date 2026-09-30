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
 * @file            PreSave.cc
 *
 * @brief           Implements Atlas::preSave(), declared in Atlas.h, with
 *                  CompFunctor::operator()().
 */

#include "Atlas.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

AtlasStatus Atlas::preSave()
{
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
            lastInitKeyFrameId =
                activeMapMaxKeyFrameId2 +
                1; // The init KF is the next of current maximum
        }
    }

    struct CompFunctor
    {
        inline bool operator()(Map *p_firstMap_in, Map *p_secondMap_in)
        {
            unsigned long firstMapId{};
            if (p_firstMap_in->getId(firstMapId) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            unsigned long secondMapId{};
            if (p_secondMap_in->getId(secondMapId) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            return firstMapId < secondMapId;
        }
    };
    std::copy(maps.begin(), maps.end(), std::back_inserter(backupMaps));
    std::sort(backupMaps.begin(), backupMaps.end(), CompFunctor());

    std::set<camera_models::geometriccamera::GeometricCamera *> cameraSet(
        cameras.begin(),
        cameras.end());
    for (Map *p_map : backupMaps)
    {
        bool mapIsBad{};
        if (!(!p_map) &&
            p_map->isBad(mapIsBad) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_map || mapIsBad)
            continue;

        std::vector<KeyFrame *> mapAllKeyFrames{};
        if (p_map->getAllKeyFrames(mapAllKeyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (mapAllKeyFrames.size() == 0)
        {
            // Empty map, erase before of save it.
            if (setMapBad(p_map) != AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMapBad returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            continue;
        }
        if (p_map->preSave(cameraSet) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: preSave returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }
    if (removeBadMaps() != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: removeBadMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
