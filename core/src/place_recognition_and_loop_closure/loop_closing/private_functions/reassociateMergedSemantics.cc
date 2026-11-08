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
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/scene_graph/scene_graph/SceneGraph.h"

#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyStatusEnum.h"
#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include <algorithm>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <string>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

void reassociateMergedSemantics(
    atlas::Atlas                         *p_atlas_inout,
    map::Map                             *p_currentMap_inout,
    map::Map                             *p_mergeMap_in,
    const types::SystemParams            *p_systemParams_in,
    const loop_closing::MergeMapContents &mergeMapContents_in,
    semantic::Room                       *p_retainedAnchorRoom_in,
    semantic::Room                       *p_importedAnchorRoom_in)
{
    /* The gate chose the anchor pair from the link before the surviving map
     * takes the link over; fusing the anchor rooms below then retargets the
     * taken-over room pointers. */
    if (p_currentMap_inout->adoptMergedMapLinks(p_mergeMap_in) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: adoptMergedMapLinks returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<map::Map *> atlasAllMaps{};
    if (p_atlas_inout->getAllMaps(atlasAllMaps) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (map::Map *p_map : atlasAllMaps)
    {
        map::Map *p_followingMap = nullptr;
        if (p_map->getFollowingMap(p_followingMap) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFollowingMap returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_map != p_currentMap_inout && p_followingMap == p_mergeMap_in &&
            p_map->setFollowingMap(p_currentMap_inout) !=
                map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setFollowingMap returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

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
    scene_graph::SceneGraph *p_sceneGraph = nullptr;
    unsigned long            currentMapId{};
    if ((p_atlas_inout->getSceneGraph(p_sceneGraph) !=
         atlas::AtlasStatus::ATLAS_STATUS_SUCCESS) ||
        (p_currentMap_inout->getId(currentMapId) !=
         map::MapStatus::MAP_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSceneGraph/getId returned a failure status "
                     "although they cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> mapAllRooms{};
    if (p_sceneGraph->getMapRoomsInIdOrder(currentMapId, mapAllRooms) !=
        scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: the scene graph has no scene map for map %lu; "
                     "continuing without its rooms.",
                     __func__,
                     currentMapId);
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
     * only the anchor rooms are paired by the map chain link. */
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
        duplicateRoomPairs = semantic::SemanticVerify::pairRoomsInSharedFrame(
            importedRooms,
            retainedRooms,
            p_importedAnchorRoom_in,
            p_retainedAnchorRoom_in,
            configuration);
    std::cout << "[SemanticMergeFusion] imported_rooms=" << importedRooms.size()
              << " retained_rooms=" << retainedRooms.size()
              << " duplicate_room_pairs=" << duplicateRoomPairs.size()
              << std::endl;
    if (utils::utils::Utils::fuseDuplicateRoomsAfterMerge(p_atlas_inout,
                                                          p_currentMap_inout,
                                                          duplicateRoomPairs) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: fuseDuplicateRoomsAfterMerge returned a failure status "
            "although it cannot fail; continuing as before.",
            __func__);
    }

    /* Identities are primary keys from the Atlas counters: every imported
     * room and passage the merge keeps is re-keyed, so no absorbed-map id can
     * alias a surviving one (RViz markers, axioms and the archive key by id).
     * Fused duplicates were retired above and keep no id; retained entities,
     * which the live room tracker refers to, keep theirs. Rooms, floors and
     * walls hold each other by pointer, so only the ids, the room names and
     * the passage index change. */
    for (semantic::Room *p_importedRoom : importedRooms)
    {
        bool importedRoomIsBad{};
        if (!(p_importedRoom == nullptr) &&
            p_importedRoom->isBad(importedRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_importedRoom == nullptr || importedRoomIsBad)
        {
            continue;
        }

        /* The scene graph keys rooms by id: re-insert under the new key,
         * into the list the room came from. */
        const bool isDetectedRoom =
            std::find(mergeMapContents_in.detectedRooms.begin(),
                      mergeMapContents_in.detectedRooms.end(),
                      p_importedRoom) !=
            mergeMapContents_in.detectedRooms.end();
        int importedRoomId{};
        int freshRoomId{};
        if (p_importedRoom->getId(importedRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            p_atlas_inout->reserveRoomIdentity(freshRoomId) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS ||
            p_importedRoom->setId(freshRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            p_sceneGraph->moveRoom(
                currentMapId,
                currentMapId,
                p_importedRoom,
                isDetectedRoom ? scene_graph::RoomList::ROOM_LIST_DETECTED
                               : scene_graph::RoomList::ROOM_LIST_CANDIDATE) !=
                scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: re-keying a room returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Names carry the identity after a kind prefix ("Room#7"). */
        std::string importedRoomName{};
        if (p_importedRoom->getName(importedRoomName) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getName returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        const std::string::size_type identitySeparatorIndex =
            importedRoomName.find('#');
        if (identitySeparatorIndex != std::string::npos &&
            p_importedRoom->setName(
                importedRoomName.substr(0, identitySeparatorIndex + 1) +
                std::to_string(freshRoomId)) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setName returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemanticMergeFusion] Re-keyed imported Room#"
                  << importedRoomId << " to Room#" << freshRoomId << "."
                  << std::endl;
    }

    for (semantic::Passage *p_importedPassage : mergeMapContents_in.passages)
    {
        bool importedPassageIsBad{};
        if (!(p_importedPassage == nullptr) &&
            p_importedPassage->isBad(importedPassageIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_importedPassage == nullptr || importedPassageIsBad)
        {
            continue;
        }

        /* The passage index is keyed by id: re-insert under the new key. */
        int importedPassageId{};
        int freshPassageId{};
        if (p_importedPassage->getId(importedPassageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_atlas_inout->reservePassageIdentity(freshPassageId) !=
                atlas::AtlasStatus::ATLAS_STATUS_SUCCESS ||
            p_importedPassage->setId(freshPassageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_sceneGraph->movePassage(currentMapId,
                                      currentMapId,
                                      p_importedPassage) !=
                scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: re-keying a passage returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemanticMergeFusion] Re-keyed imported Passage#"
                  << importedPassageId << " to Passage#" << freshPassageId
                  << "." << std::endl;
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
