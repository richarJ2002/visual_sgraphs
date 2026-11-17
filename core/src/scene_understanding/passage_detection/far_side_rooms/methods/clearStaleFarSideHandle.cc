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
 * @file            clearStaleFarSideHandle.cc
 *
 * @brief           Implements SemanticsManager::clearStaleFarSideHandle(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
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
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::clearStaleFarSideHandle(
    semantic::Passage    *p_passage_inout,
    PassageRoomLinkState &linkState_inout)
{
    vs_graphs::core::semantic::Room *p_existingProspective = nullptr;
    if (p_passage_inout->getProspectiveRoom(p_existingProspective) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getProspectiveRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    linkState_inout.hadProspectiveHandle = p_existingProspective != nullptr;

    bool existingProspectiveIsBad{};
    if ((p_existingProspective != nullptr) &&
        p_existingProspective->isBad(existingProspectiveIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Room::RoomVariant existingProspectiveRoomVariant{};
    if ((p_existingProspective != nullptr && !existingProspectiveIsBad) &&
        p_existingProspective->getRoomVariant(existingProspectiveRoomVariant) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getRoomVariant returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_existingProspective != nullptr && existingProspectiveIsBad)
    {
        if (p_passage_inout->setProspectiveRoom(nullptr) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int clearedPassageId{};
        int badHandleId{};
        if (p_passage_inout->getId(clearedPassageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_existingProspective->getId(badHandleId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                    "[SemMgr] Passage#%d far-side handle Room#%d cleared: "
                    "the room is bad (fused or deleted).",
                    clearedPassageId,
                    badHandleId);
    }
    else if (p_existingProspective != nullptr &&
             existingProspectiveRoomVariant !=
                 vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
    {
        /*
         * Promotion/replacement keeps the same far-side resolution, but
         * never as a third room: once two other rooms hold both sides the
         * promoted handle is stale (it is cleared below).
         */
        const bool areBothSidesHeldByOthers =
            linkState_inout.p_negativeSideRoom != nullptr &&
            linkState_inout.p_positiveSideRoom != nullptr &&
            linkState_inout.p_negativeSideRoom !=
                linkState_inout.p_positiveSideRoom &&
            p_existingProspective != linkState_inout.p_negativeSideRoom &&
            p_existingProspective != linkState_inout.p_positiveSideRoom;
        if (!areBothSidesHeldByOthers &&
            p_existingProspective->addPassageAssociation(p_passage_inout) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addPassageAssociation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int existingProspectiveId{};
        if (p_existingProspective->getId(existingProspectiveId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        prospectiveRoomCycles.erase(existingProspectiveId);
    }

    semantic::Room *p_currentFarSideHandle = nullptr;
    if (p_passage_inout->getProspectiveRoom(p_currentFarSideHandle) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getProspectiveRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    /*!
     * A sole confirmed room that won the side opposite the known side is
     * this passage's far-side room, the same state as "resolved to
     * opposite confirmed" below. A seed fused into that room leaves the
     * handle on it; clearing it here would create a new seed in the same
     * space every cycle.
     */
    linkState_inout.isSoleRoomOnFarSide =
        linkState_inout.confirmedAssociatedRoomCount == 1U &&
        linkState_inout.knownSide.p_room !=
            linkState_inout.p_confirmedAssociatedRoom &&
        ((linkState_inout.knownSideSign > 0.0 &&
          linkState_inout.p_confirmedAssociatedRoom ==
              linkState_inout.p_negativeSideRoom) ||
         (linkState_inout.knownSideSign < 0.0 &&
          linkState_inout.p_confirmedAssociatedRoom ==
              linkState_inout.p_positiveSideRoom));
    const bool isHandleTheKnownSideRoom =
        linkState_inout.confirmedAssociatedRoomCount == 1U &&
        p_currentFarSideHandle == linkState_inout.p_confirmedAssociatedRoom &&
        !linkState_inout.isSoleRoomOnFarSide;
    if (isHandleTheKnownSideRoom ||
        (linkState_inout.confirmedAssociatedRoomCount == 2U &&
         p_currentFarSideHandle != nullptr &&
         p_currentFarSideHandle != linkState_inout.p_negativeSideRoom &&
         p_currentFarSideHandle != linkState_inout.p_positiveSideRoom))
    {
        if (p_passage_inout->setProspectiveRoom(nullptr) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int clearedPassageId{};
        int staleHandleId{};
        if (p_passage_inout->getId(clearedPassageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_currentFarSideHandle->getId(staleHandleId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                    "[SemMgr] Passage#%d far-side handle Room#%d cleared: "
                    "%s.",
                    clearedPassageId,
                    staleHandleId,
                    isHandleTheKnownSideRoom
                        ? "it is the only confirmed room and not on the "
                          "far side"
                        : "two other rooms hold both sides");
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
