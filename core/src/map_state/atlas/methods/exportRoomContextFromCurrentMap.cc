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
 * @file            exportRoomContextFromCurrentMap.cc
 *
 * @brief           Implements Atlas::exportRoomContextFromCurrentMap(),
 *                  declared in atlas/objects/AtlasClass.h.
 */

#include "../private_functions/private_functions.h"
#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/atlas/objects/RoomContextExportStruct.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::exportRoomContextFromCurrentMap()
{
    /* Called without atlasMutex (lock order: graph locks before atlasMutex),
     * so the active map is read under it for a moment. */
    map::Map *p_exportMap = nullptr;
    {
        std::lock_guard<std::mutex> atlasLock(atlasMutex);
        p_exportMap = p_activeMap;
    }
    if (!p_exportMap)
        return AtlasStatus::ATLAS_STATUS_SUCCESS;

    unsigned long mapId{};
    if (p_exportMap->getId(mapId) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* The room the UAV was following when this map was stranded, for the
     * mission-chain trace. */
    int departureRoomId{};
    if (getCurrentSemanticRoomIdentity(departureRoomId) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentSemanticRoomIdentity returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    /* This runs on the tracking thread when a map is lost, which never waits
     * for a graph lock. While the semantic cycle holds the graph, the context
     * the last cycle cached is exported instead of a fresh one. */
    RoomContextExport            roomContext{};
    GraphLockScope               graphLockScope{};
    const SemanticsManagerStatus lockStatus = p_semanticsManager->lockGraphs(
        {mapId},
        GraphLockMode::GRAPH_LOCK_MODE_TRY_SHARED,
        graphLockScope);
    if (lockStatus == SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        captureMapRoomContext(*p_semanticsManager,
                              *p_exportMap,
                              departureRoomId,
                              roomContext);
    }
    else if (lockStatus ==
             SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_BUSY)
    {
        std::lock_guard<std::mutex> contextLock(roomContextMutex);
        if (cachedRoomContext.isCaptured && cachedRoomContext.mapId == mapId)
        {
            roomContext = cachedRoomContext;
        }
        RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                    "[Atlas] The semantic cycle holds the graph of map %lu; "
                    "exporting the room context cached at its last cycle "
                    "(%zu rooms).",
                    mapId,
                    roomContext.roomSnapshots.size());
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: lockGraphs found no scene graph for map %lu; "
                     "continuing without its rooms.",
                     __func__,
                     mapId);
    }
    if (roomContext.roomSnapshots.empty())
        return AtlasStatus::ATLAS_STATUS_SUCCESS;

    const std::size_t exportedRoomCount = roomContext.roomSnapshots.size();
    {
        std::lock_guard<std::mutex> contextLock(roomContextMutex);
        roomContextHistory[mapId] = std::move(roomContext.roomSnapshots);
    }

    /* The anchor rooms are recorded as ids; whoever needs the rooms resolves
     * them through the map's graph, which can fail. */
    if (roomContext.finalRoomId != map::Map::NO_LINKED_ROOM_ID)
    {
        RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                    "[Atlas] Departure anchor: final_room=%d "
                    "key_frame=%ld passage_in_view=%d "
                    "room_beyond_passage=%d",
                    roomContext.finalRoomId,
                    roomContext.lastKeyFrameId,
                    roomContext.passageInViewId,
                    roomContext.roomBeyondPassageId);
        if (p_exportMap->setFinalRoomIds(roomContext.finalRoomId,
                                         roomContext.roomBeyondPassageId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setFinalRoomIds returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    std::cout << "[Atlas] Exported room context: " << exportedRoomCount
              << " rooms (mapId: " << mapId << ")" << std::endl;

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
