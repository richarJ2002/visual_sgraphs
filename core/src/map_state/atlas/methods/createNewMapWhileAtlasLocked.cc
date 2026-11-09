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

#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::createNewMapWhileAtlasLocked()
{
    /* Output map */
    std::cout << "\n[Atlas]" << std::endl;
    std::cout << "- Creating a new map (MapId: " << map::Map::nextId
              << ", Init keyframe::KeyFrame: " << lastInitKeyFrameId << ") ..."
              << std::endl;

    /* If we currently have an active map, then clean up */
    if (p_activeMap)
    {
        /* Declare variable to store the max keyframe ID of active map */
        unsigned long activeMapMaxKeyFrameId{};

        /* If map is not empty, extract the max keyframe ID */
        if (p_activeMap->getMaxKeyFrameId(activeMapMaxKeyFrameId) !=
                map::MapStatus::MAP_STATUS_SUCCESS &&
            (!maps.empty()))
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMaxKeyFrameId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Make sure that next keyframe is greater then latest keyframe ID */
        if (lastInitKeyFrameId < activeMapMaxKeyFrameId)
        {
            lastInitKeyFrameId = activeMapMaxKeyFrameId + 1;
        }

        /* Change flag in map to indicate that it is no longer in use */
        if (p_activeMap->setStoredMap() != map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setStoredMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Decalre variable to hold the new active map id */
        unsigned long activeMapId{};

        /* Extract the active map id */
        if (p_activeMap->getId(activeMapId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "- The created map with MapId #" << activeMapId
                  << " has been stored!" << std::endl;
    }

    /* Store the address of the current active map */
    map::Map *p_previousMap = p_activeMap;

    /* Create new map (which is initialized as current active map) */
    p_activeMap = new map::Map(lastInitKeyFrameId);

    /* Set the new map as the current active map */
    if (p_activeMap->setCurrentMap() != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Insert the new active map into the atlas map list */
    maps.insert(p_activeMap);

    /* The new map gets its empty scene graph and becomes the semantics
     * manager's active map, so the registry always has one scene graph per map
     * in maps; the map's rooms, passages and floors live only in the graph. */

    /* Declare variable to hold the id of the new active map */
    unsigned long newMapId{};

    /*!
     * 3 things are done here:
     *      - Get id of the new active map
     *      - Add the scene graph to the semantics manager's registry
     *      - Set the active map in the semantics manager
     *
     * Before the semantics manager is attached the maps are not registered
     * here; Atlas::setSemanticsManager() registers them.
     */
    if ((p_activeMap->getId(newMapId) != map::MapStatus::MAP_STATUS_SUCCESS) ||
        ((p_semanticsManager != nullptr) &&
         ((p_semanticsManager->addSceneGraph(*p_activeMap) !=
           SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS) ||
          (p_semanticsManager->setActiveMap(newMapId) !=
           SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS))))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId/addSceneGraph/setActiveMap returned "
                     "a failure status although they cannot fail; continuing "
                     "as before.",
                     __func__);
    }

    /* If the prveious map exists */
    if (p_previousMap != nullptr)
    {
        /* Link the new active map to the previous map to create a link */
        if (p_previousMap->setFollowingMap(p_activeMap) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
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

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
