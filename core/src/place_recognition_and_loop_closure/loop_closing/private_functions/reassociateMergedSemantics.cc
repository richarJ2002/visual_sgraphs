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
 * @file            reassociateMergedSemantics.cc
 *
 * @brief           Implements reassociateMergedSemantics(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"

#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

void reassociateMergedSemantics(
    atlas::Atlas                         *p_atlas_inout,
    map::Map                             *p_currentMap_inout,
    const types::SystemParams            *p_systemParams_in,
    const loop_closing::MergeMapContents &mergeMapContents_in)
{
    /* Fuse only after every semantic relationship is visible. */
    if (p_systemParams_in->semSeg.reassociate.enabled)
    {
        if (utils::utils::Utils::reAssociateSemanticPlanes(p_atlas_inout) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: reAssociateSemanticPlanes returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }

    std::vector<semantic::Room *> importedRooms =
        mergeMapContents_in.detectedRooms;
    importedRooms.insert(importedRooms.end(),
                         mergeMapContents_in.markerBasedRooms.begin(),
                         mergeMapContents_in.markerBasedRooms.end());

    /* Rooms the surviving map had before the merge are the fusion targets. */
    std::vector<semantic::Room *> mapAllRooms{};
    if (p_currentMap_inout->getAllRooms(mapAllRooms) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> retainedRooms{};
    for (semantic::Room *p_room : mapAllRooms)
    {
        if (std::find(importedRooms.begin(), importedRooms.end(), p_room) ==
            importedRooms.end())
        {
            retainedRooms.push_back(p_room);
        }
    }

    /* Room fusion is a merge invariant, not an optional geometry
     * re-association feature. Walls, floors and rooms are paired by geometry;
     * only the anchor rooms are paired by their tag. */
    semantic::SemanticVerifyConfig configuration{};
    if (semantic::SemanticVerify::configFromSystemParams(configuration) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: configFromSystemParams returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const std::vector<std::pair<semantic::Room *, semantic::Room *>>
        duplicateRoomPairs =
            semantic::SemanticVerify::pairRoomsInSharedFrame(importedRooms,
                                                             retainedRooms,
                                                             configuration);
    std::cout << "[SemanticMergeFusion] imported_rooms=" << importedRooms.size()
              << " retained_rooms=" << retainedRooms.size()
              << " duplicate_room_pairs=" << duplicateRoomPairs.size()
              << std::endl;
    if (utils::utils::Utils::fuseDuplicateRoomsAfterMerge(p_currentMap_inout,
                                                          duplicateRoomPairs) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: fuseDuplicateRoomsAfterMerge returned a failure status "
            "although it cannot fail; continuing as before.",
            __func__);
    }

    if (p_systemParams_in->semSeg.reassociate.enabled)
    {
        if (utils::utils::Utils::reAssociateRooms(p_atlas_inout) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reAssociateRooms returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (utils::utils::Utils::reAssociatePassages(p_atlas_inout) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reAssociatePassages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
