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
 * @file            setSemanticsManager.cc
 *
 * @brief           Implements Atlas::setSemanticsManager(), declared in
 *                  atlas/objects/AtlasClass.h.
 */

#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::setSemanticsManager(SemanticsManager *p_semanticsManager_in)
{
    std::unique_lock<std::mutex> lock(atlasMutex);
    p_semanticsManager = p_semanticsManager_in;

    /* Maps created or loaded before the manager existed have no registry
     * entry yet: the tree gets one scene graph per map in maps, and the current
     * map becomes the active map. */
    for (map::Map *p_map : maps)
    {
        if (p_semanticsManager->addSceneGraph(*p_map) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addSceneGraph returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
    }
    unsigned long activeMapId{};
    if ((p_activeMap != nullptr) &&
        ((p_activeMap->getId(activeMapId) !=
          map::MapStatus::MAP_STATUS_SUCCESS) ||
         (p_semanticsManager->setActiveMap(activeMapId) !=
          SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId/setActiveMap returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
