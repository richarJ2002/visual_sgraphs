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
 * @file            transferRooms.cc
 *
 * @brief           Implements transferRooms(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/scene_graph/Room/Room.h"
#include "scene_understanding/scene_graph/Room/objects/RoomStatusEnum.h"
#include "scene_understanding/scene_graph/SceneGraph/SceneGraph.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void transferRooms(atlas::Atlas                        *p_atlas_inout,
                   map::Map                            *p_currentMap_inout,
                   map::Map                            *p_mergeMap_inout,
                   const std::vector<semantic::Room *> &detectedRooms_in,
                   const std::vector<semantic::Room *> &markerBasedRooms_in,
                   bool skipBadMarkerBasedRooms_in)
{
    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    unsigned long            currentMapId{};
    unsigned long            mergeMapId{};
    if ((p_atlas_inout->getSceneGraph(p_sceneGraph) !=
         atlas::AtlasStatus::ATLAS_STATUS_SUCCESS) ||
        (p_currentMap_inout->getId(currentMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS) ||
        (p_mergeMap_inout->getId(mergeMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneGraph/getId returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }

    for (vs_graphs::core::semantic::Room *p_currentDetectedRoom :
         detectedRooms_in)
    {
        /* Skip invalid rooms */
        bool currentDetectedRoomIsBad{};
        if (!(p_currentDetectedRoom == nullptr) &&
            p_currentDetectedRoom->isBad(currentDetectedRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentDetectedRoom == nullptr || currentDetectedRoomIsBad)
        {
            continue;
        }

        /* Set the map of the room in the current map */
        if (p_currentDetectedRoom->setMap(p_currentMap_inout) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Hand the room from the merged map to the current map */
        if (p_sceneGraph->moveRoom(mergeMapId,
                                   currentMapId,
                                   p_currentDetectedRoom,
                                   scene_graph::RoomList::ROOM_LIST_DETECTED) !=
            scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: moveRoom found no scene map for maps %lu and "
                         "%lu or no such room; continuing as before.",
                         __func__,
                         mergeMapId,
                         currentMapId);
        }
    }

    for (vs_graphs::core::semantic::Room *pRoom : markerBasedRooms_in)
    {
        if (!pRoom)
            continue;
        if (skipBadMarkerBasedRooms_in)
        {
            bool markerBasedRoomIsBad{};
            if (pRoom->isBad(markerBasedRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (markerBasedRoomIsBad)
            {
                continue;
            }
        }

        if (pRoom->setMap(p_currentMap_inout) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_sceneGraph->moveRoom(
                mergeMapId,
                currentMapId,
                pRoom,
                scene_graph::RoomList::ROOM_LIST_CANDIDATE) !=
            scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: moveRoom found no scene map for maps %lu and "
                         "%lu or no such room; continuing as before.",
                         __func__,
                         mergeMapId,
                         currentMapId);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
