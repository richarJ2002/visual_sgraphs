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
 * @file            transferPassages.cc
 *
 * @brief           Implements transferPassages(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantic/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantic/passage/passage.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{

void transferPassages(map::Map *p_currentMap_inout,
                      map::Map *p_mergeMap_inout,
                      const std::vector<semantic::Passage *> &passages_in,
                      const std::vector<semantic::Room *>    &detectedRooms_in,
                      const std::vector<semantic::Room *>  &markerBasedRooms_in,
                      const semantic::SemanticVerifyConfig &configuration_in)
{
    /*
     * Pair before anything moves: the surviving map's own passages are the
     * only candidates for a duplicate.
     */
    std::vector<semantic::Passage *> currentMapAllPassages{};
    if (p_currentMap_inout->getAllPassages(currentMapAllPassages) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const std::vector<std::pair<semantic::Passage *, semantic::Passage *>>
        duplicatePairs = semantic::SemanticVerify::pairPassagesInSharedFrame(
            passages_in,
            currentMapAllPassages,
            configuration_in);

    for (vs_graphs::core::semantic::Passage *p_passage : passages_in)
    {
        /* Skip invalid rooms */
        if (p_passage == nullptr)
        {
            continue;
        }

        /* A retained passage is the one found by geometry, never by id. */
        semantic::Passage *p_retainedPassage = nullptr;
        for (const std::pair<semantic::Passage *, semantic::Passage *>
                 &duplicatePair : duplicatePairs)
        {
            if (duplicatePair.first == p_passage)
            {
                p_retainedPassage = duplicatePair.second;
                break;
            }
        }

        if (p_mergeMap_inout->eraseMapPassage(p_passage) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseMapPassage returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_retainedPassage != nullptr)
        {
            bool retainedPassageWasGeometryReplaced{};
            if (p_retainedPassage->mergeFromDuplicate(
                    p_passage,
                    retainedPassageWasGeometryReplaced) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                retainedPassageWasGeometryReplaced = false;
                RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                            "%s: mergeFromDuplicate rejected its input; "
                            "continuing as before.",
                            __func__);
            }
            for (semantic::Room *p_room : detectedRooms_in)
            {
                if (p_room != nullptr)
                {
                    bool roomWasAssociationReplaced{};
                    if (p_room->replacePassageAssociation(
                            p_passage,
                            p_retainedPassage,
                            roomWasAssociationReplaced) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        roomWasAssociationReplaced = false;
                        RCLCPP_WARN(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: replacePassageAssociation rejected its "
                            "input; continuing as before.",
                            __func__);
                    }
                }
            }
            for (semantic::Room *p_room : markerBasedRooms_in)
            {
                if (p_room != nullptr)
                {
                    bool roomWasAssociationReplaced2{};
                    if (p_room->replacePassageAssociation(
                            p_passage,
                            p_retainedPassage,
                            roomWasAssociationReplaced2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        roomWasAssociationReplaced2 = false;
                        RCLCPP_WARN(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: replacePassageAssociation rejected its "
                            "input; continuing as before.",
                            __func__);
                    }
                }
            }
            if (p_passage->setBad() !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            continue;
        }

        if (p_passage->setMap(p_currentMap_inout) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_currentMap_inout->addMapPassage(p_passage) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPassage returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
