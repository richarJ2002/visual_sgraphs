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
 * @file            associatePassagesToRooms.cc
 *
 * @brief           Implements SemanticsManager::associatePassagesToRooms(),
 *                  declared in SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::associatePassagesToRooms(void)
{
    PROFILE_FUNCTION();
    /* Extract all rooms from the current map */
    std::vector<vs_graphs::core::semantic::Room *> allRooms{};
    if (getAllRoomsInIdOrder(allRooms) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRoomsInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }

    /* Extract all passages from the current map */
    std::vector<vs_graphs::core::semantic::Passage *> allPassages{};
    if (getAllPassagesInIdOrder(allPassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassagesInIdOrder returned a failure status; "
                     "continuing as before.",
                     __func__);
    }

    /* Stable room ordering makes equal-distance passage associations
     * repeatable. */
    std::sort(allRooms.begin(),
              allRooms.end(),
              semantic::isEntityIdLess<semantic::Room>);

    /*!
     * Snapshot the previous topology before rebuilding it. The semantic pass
     * runs periodically, so logging every unchanged edge as newly associated
     * would obscure genuine topology changes.
     */
    std::unordered_map<vs_graphs::core::semantic::Room *,
                       std::unordered_set<int>>
        previousPassageIdsByRoom;

    /* Rebuild the topology so stale associations cannot survive a remerge. */
    for (vs_graphs::core::semantic::Room *p_room : allRooms)
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad)
        {
            std::unordered_set<int> &previousPassageIds =
                previousPassageIdsByRoom[p_room];

            std::vector<vs_graphs::core::semantic::Passage *> roomPassages2{};
            if (p_room->getPassages(roomPassages2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (vs_graphs::core::semantic::Passage *p_previousPassage :
                 roomPassages2)
            {
                if (p_previousPassage != nullptr)
                {
                    int previousPassageId{};
                    if (p_previousPassage->getId(previousPassageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    previousPassageIds.insert(previousPassageId);
                }
            }

            if (p_room->clearPassages() !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: clearPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    /* Iterate through every passage */
    for (vs_graphs::core::semantic::Passage *p_passage : allPassages)
    {
        PassageRoomLinkState linkState{};
        if (readPassageLinkGeometry(p_passage, linkState) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: readPassageLinkGeometry returned a failure "
                         "status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!linkState.isPassageLinkable)
        {
            continue;
        }
        if (findPassageSideRooms(allRooms, p_passage, linkState) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findPassageSideRooms returned a failure status "
                         "although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (dropCrossFloorSideRoom(p_passage, linkState) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: dropCrossFloorSideRoom returned a failure status "
                         "although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (linkPassageToSideRooms(allRooms,
                                   previousPassageIdsByRoom,
                                   p_passage,
                                   linkState) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: linkPassageToSideRooms returned a failure status "
                         "although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (createFarSideRoomHypotheses(allRooms,
                                        allPassages,
                                        p_passage,
                                        linkState) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: createFarSideRoomHypotheses returned a failure "
                         "status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (resolveProspectiveRoomPromotion(allRooms, p_passage) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: resolveProspectiveRoomPromotion returned a "
                         "failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Report, but never fabricate, missing passage connectivity. */
    std::unordered_set<int> computedDisconnectedRoomIds;

    for (vs_graphs::core::semantic::Room *p_room : allRooms)
    {
        bool roomIsBad3{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant roomVariant{};
        if (!(p_room == nullptr || roomIsBad3) &&
            p_room->getRoomVariant(roomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<vs_graphs::core::semantic::Passage *> roomPassages3{};
        if (!(p_room == nullptr || roomIsBad3 ||
              roomVariant ==
                  vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED) &&
            p_room->getPassages(roomPassages3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPassages returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad3 ||
            roomVariant ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED ||
            !roomPassages3.empty())
        {
            continue;
        }

        int roomId3{};
        if (p_room->getId(roomId3) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        computedDisconnectedRoomIds.insert(roomId3);

        int roomId4{};
        if (p_room->getId(roomId4) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (disconnectedRoomIds.count(roomId4) == 0U)
        {
            int roomId5{};
            if (p_room->getId(roomId5) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Room#" << roomId5
                      << " is not yet connected by a confirmed passage; "
                         "semantic routing will treat it as disconnected."
                      << std::endl;
        }
    }

    disconnectedRoomIds = std::move(computedDisconnectedRoomIds);

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
