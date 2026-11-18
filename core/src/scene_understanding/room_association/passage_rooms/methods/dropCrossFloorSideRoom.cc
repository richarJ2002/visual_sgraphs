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
 * @file            dropCrossFloorSideRoom.cc
 *
 * @brief           Implements SemanticsManager::dropCrossFloorSideRoom(),
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

SemanticsManagerStatus SemanticsManager::dropCrossFloorSideRoom(
    semantic::Passage    *p_passage_inout,
    PassageRoomLinkState &linkState_inout)
{
    /*!
     * @brief           Axiom: both rooms a passage links must be on the
     *                  same floor, except through a vertical passage /
     *                  staircase (not implemented yet -- see the user's own
     *                  carve-out). A same-passage, different-floor match is
     *                  therefore not new information, it is a matching
     *                  error: since no vertical-passage mechanism exists to
     *                  produce a genuine one, one of the two sides must be
     *                  wrong. Keep whichever side is closer to the passage
     *                  (the stronger match) and drop the farther one back
     *                  to unresolved for this cycle -- it can still recover
     *                  in a later cycle, e.g. once its own floor identity
     *                  is corrected, or a different room wins that side
     *                  instead.
     */
    if (linkState_inout.p_negativeSideRoom != nullptr &&
        linkState_inout.p_positiveSideRoom != nullptr)
    {
        vs_graphs::core::semantic::Floor *p_negativeFloor = nullptr;
        if (linkState_inout.p_negativeSideRoom->getFloor(p_negativeFloor) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFloor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        vs_graphs::core::semantic::Floor *p_positiveFloor = nullptr;
        if (linkState_inout.p_positiveSideRoom->getFloor(p_positiveFloor) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFloor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        bool negativeFloorHasPlaneIdentity{};
        if ((p_negativeFloor != nullptr && p_positiveFloor != nullptr) &&
            p_negativeFloor->hasPlaneIdentity(negativeFloorHasPlaneIdentity) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasPlaneIdentity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool positiveFloorHasPlaneIdentity{};
        if ((p_negativeFloor != nullptr && p_positiveFloor != nullptr &&
             negativeFloorHasPlaneIdentity) &&
            p_positiveFloor->hasPlaneIdentity(positiveFloorHasPlaneIdentity) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasPlaneIdentity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int negativeFloorId{};
        if ((p_negativeFloor != nullptr && p_positiveFloor != nullptr &&
             negativeFloorHasPlaneIdentity && positiveFloorHasPlaneIdentity) &&
            p_negativeFloor->getId(negativeFloorId) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        int positiveFloorId{};
        if ((p_negativeFloor != nullptr && p_positiveFloor != nullptr &&
             negativeFloorHasPlaneIdentity && positiveFloorHasPlaneIdentity) &&
            p_positiveFloor->getId(positiveFloorId) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_negativeFloor != nullptr && p_positiveFloor != nullptr &&
            negativeFloorHasPlaneIdentity && positiveFloorHasPlaneIdentity &&
            negativeFloorId != positiveFloorId)
        {
            const bool negativeIsFarther =
                linkState_inout.negativeRoomDistance_m >=
                linkState_inout.positiveRoomDistance_m;
            vs_graphs::core::semantic::Room *p_droppedRoom =
                negativeIsFarther ? linkState_inout.p_negativeSideRoom
                                  : linkState_inout.p_positiveSideRoom;

            int passageId4{};
            if (p_passage_inout->getId(passageId4) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            int negativeSideRoomId{};
            if (linkState_inout.p_negativeSideRoom->getId(negativeSideRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            int negativeFloorId2{};
            if (p_negativeFloor->getId(negativeFloorId2) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            int positiveSideRoomId{};
            if (linkState_inout.p_positiveSideRoom->getId(positiveSideRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            int positiveFloorId2{};
            if (p_positiveFloor->getId(positiveFloorId2) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            int droppedRoomId{};
            if (p_droppedRoom->getId(droppedRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Passage#" << passageId4 << " matched Room#"
                      << negativeSideRoomId << " (Floor#" << negativeFloorId2
                      << ") and Room#" << positiveSideRoomId << " (Floor#"
                      << positiveFloorId2
                      << ") on different floors -- no vertical passage "
                         "mechanism exists, so this is a matching "
                         "error, not a real staircase; dropping the "
                         "farther match Room#"
                      << droppedRoomId << " for this cycle." << std::endl;

            if (negativeIsFarther)
            {
                linkState_inout.p_negativeSideRoom             = nullptr;
                linkState_inout.p_negativeExactSupportingOwner = nullptr;
            }
            else
            {
                linkState_inout.p_positiveSideRoom             = nullptr;
                linkState_inout.p_positiveExactSupportingOwner = nullptr;
            }
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
