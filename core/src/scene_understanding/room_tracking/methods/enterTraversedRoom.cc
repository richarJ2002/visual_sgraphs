/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            enterTraversedRoom.cc
 *
 * @brief           Implements SemanticsManager::enterTraversedRoom(), declared
 * in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <string>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::enterTraversedRoom(unsigned long      activeMapId_in,
                                         map::Map          *p_activeMap_in,
                                         semantic::Passage *p_passage_in,
                                         semantic::Room    *p_reachedRoom_inout)
{
    std::vector<semantic::Room *> activeRooms{};
    if (getMapRoomsInIdOrder(activeMapId_in, activeRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapRoomsInIdOrder returned a failure "
                     "status; continuing as before.",
                     __func__);
    }
    bool reachedRoomIsBad{};
    if ((p_reachedRoom_inout != nullptr) &&
        p_reachedRoom_inout->isBad(reachedRoomIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    map::Map *p_reachedRoomMap = nullptr;
    if ((p_reachedRoom_inout != nullptr && !reachedRoomIsBad) &&
        p_reachedRoom_inout->getMap(p_reachedRoomMap) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    const bool reachedRoomIsLive =
        p_reachedRoom_inout != nullptr && !reachedRoomIsBad &&
        p_reachedRoomMap == p_activeMap_in &&
        std::find(activeRooms.begin(),
                  activeRooms.end(),
                  p_reachedRoom_inout) != activeRooms.end();
    if (reachedRoomIsLive)
    {
        semantic::Room::RoomVariant reachedRoomRoomVariant{};
        if (p_reachedRoom_inout->getRoomVariant(reachedRoomRoomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (reachedRoomRoomVariant == semantic::Room::RoomVariant::UNDEFINED)
        {
            if (promoteCandidateRoom(activeMapId_in, p_reachedRoom_inout) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: promoteCandidateRoom returned a failure "
                             "status; continuing as before.",
                             __func__);
            }
            if (p_reachedRoom_inout->setRoomVariant(
                    semantic::Room::RoomVariant::ROOM) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setRoomVariant returned a "
                             "failure status although it cannot "
                             "fail; continuing as before.",
                             __func__);
            }
            int reachedRoomId2{};
            if (p_reachedRoom_inout->getId(reachedRoomId2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_reachedRoom_inout->setName("Room#" +
                                             std::to_string(reachedRoomId2)) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setName returned a failure "
                             "status although it cannot fail; "
                             "continuing as before.",
                             __func__);
            }
            if (p_reachedRoom_inout->setBoundaryStatus(
                    semantic::Room::BoundaryStatus::UNOBSERVED) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                /*!
                 * setBoundaryStatus cannot fail; continue as
                 * before.
                 */
            }
            int reachedRoomId3{};
            if (p_reachedRoom_inout->getId(reachedRoomId3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            prospectiveRoomCycles.erase(reachedRoomId3);

            semantic::Floor               *p_floor = nullptr;
            std::vector<semantic::Floor *> activeMapAllFloors{};
            if (getMapFloorsInIdOrder(activeMapId_in, activeMapAllFloors) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapFloorsInIdOrder returned a failure "
                             "status; continuing as before.",
                             __func__);
            }
            if (semantic::Floor::selectBestObservedFloor(activeMapAllFloors,
                                                         p_floor) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                /*!
                 * selectBestObservedFloor cannot fail; continue as
                 * before.
                 */
            }
            if (p_floor != nullptr)
            {
                if (p_floor->addRoom(p_reachedRoom_inout) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: addRoom returned a failure "
                                 "status although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
            }
            int reachedRoomId4{};
            if (p_reachedRoom_inout->getId(reachedRoomId4) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            int passageId{};
            if (p_passage_in->getId(passageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "SG_PIPELINE {\"event\":\"room_promotion\","
                         "\"map_id\":"
                      << activeMapId_in << ",\"room_id\":" << reachedRoomId4
                      << ",\"passage_id\":" << passageId
                      << ",\"reason\":\"PASSAGE_TRAVERSAL\","
                         "\"semantic_cycle\":"
                      << pipelineSemanticCycle << "}" << std::endl;
        }
        int reachedRoomId{};
        if (p_reachedRoom_inout->getId(reachedRoomId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        {
            std::lock_guard<std::mutex> currentRoomLock(currentRoomMutex);
            currentRoomId = reachedRoomId;
        }
        if (p_atlas->setCurrentSemanticRoomIdentity(reachedRoomId) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCurrentSemanticRoomIdentity "
                         "returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        /* Completed passage traversal into this room: entry
         * evidence marks it visited. */
        if (p_reachedRoom_inout->setPreviouslyVisited(true) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPreviouslyVisited returned a "
                         "failure status although it cannot fail; "
                         "continuing as before.",
                         __func__);
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
