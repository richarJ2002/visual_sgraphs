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
 * @file            refreshRoomContextCache.cc
 *
 * @brief           Implements Atlas::refreshRoomContextCache(), declared in
 *                  atlas/objects/AtlasClass.h.
 */

#include "../private_functions/private_functions.h"
#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/atlas/objects/RoomContextExportStruct.h"
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

AtlasStatus Atlas::refreshRoomContextCache()
{
    /* Called with the graph locks held (lock order: graph locks before
     * atlasMutex), so the active map is read under atlasMutex for a moment. */
    map::Map *p_cacheMap = nullptr;
    {
        std::lock_guard<std::mutex> atlasLock(atlasMutex);
        p_cacheMap = p_activeMap;
    }
    if ((p_cacheMap == nullptr) || (p_semanticsManager == nullptr))
        return AtlasStatus::ATLAS_STATUS_SUCCESS;

    unsigned long mapId{};
    if (p_cacheMap->getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    int departureRoomId{};
    if (getCurrentSemanticRoomIdentity(departureRoomId) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentSemanticRoomIdentity returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    GraphLockScope graphLockScope{};
    if (p_semanticsManager->lockGraphs({mapId},
                                       GraphLockMode::GRAPH_LOCK_MODE_SHARED,
                                       graphLockScope) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: lockGraphs found no scene graph for map %lu; the "
                     "room context cache is not refreshed.",
                     __func__,
                     mapId);
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
    RoomContextExport roomContext{};
    captureMapRoomContext(*p_semanticsManager,
                          *p_cacheMap,
                          departureRoomId,
                          roomContext);

    std::lock_guard<std::mutex> contextLock(roomContextMutex);
    cachedRoomContext = std::move(roomContext);
    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
