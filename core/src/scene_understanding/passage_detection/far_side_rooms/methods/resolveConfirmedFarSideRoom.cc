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
 * @file            resolveConfirmedFarSideRoom.cc
 *
 * @brief           Implements SemanticsManager::resolveConfirmedFarSideRoom(),
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

SemanticsManagerStatus SemanticsManager::resolveConfirmedFarSideRoom(
    semantic::Passage    *p_passage_inout,
    PassageRoomLinkState &linkState_inout)
{
    if (linkState_inout.confirmedAssociatedRoomCount == 2U)
    {
        semantic::Room *p_farSideConfirmedRoom = nullptr;
        bool            knownSideHasDirection11{};
        if (linkState_inout.knownSide.hasDirection(knownSideHasDirection11) !=
            semantic::KnownSideProvenanceStatus::
                KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasDirection returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (linkState_inout.knownSide.p_room ==
            linkState_inout.p_negativeSideRoom)
        {
            p_farSideConfirmedRoom = linkState_inout.p_positiveSideRoom;
        }
        else if (linkState_inout.knownSide.p_room ==
                 linkState_inout.p_positiveSideRoom)
        {
            p_farSideConfirmedRoom = linkState_inout.p_negativeSideRoom;
        }
        else if (knownSideHasDirection11)
        {
            bool isNegativeRoomOnKnownSide = false;
            if (isRoomOnKnownSide(linkState_inout.p_negativeSideRoom,
                                  linkState_inout.passageEquation_world,
                                  linkState_inout.knownSide,
                                  linkState_inout.knownSideSign,
                                  isNegativeRoomOnKnownSide) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isRoomOnKnownSide returned a failure status "
                             "although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            p_farSideConfirmedRoom = isNegativeRoomOnKnownSide
                                         ? linkState_inout.p_positiveSideRoom
                                         : linkState_inout.p_negativeSideRoom;
        }
        else if (linkState_inout.p_negativeExactSupportingOwner != nullptr)
        {
            if (p_passage_inout->setKnownSideRoom(
                    linkState_inout.p_negativeExactSupportingOwner) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setKnownSideRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage_inout->setKnownSideDirection(
                    -linkState_inout.passageEquation_world.head<3>()) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                            "%s: setKnownSideDirection rejected its input; "
                            "continuing as before.",
                            __func__);
            }
            p_farSideConfirmedRoom = linkState_inout.p_positiveSideRoom;
        }
        else if (linkState_inout.p_positiveExactSupportingOwner != nullptr)
        {
            if (p_passage_inout->setKnownSideRoom(
                    linkState_inout.p_positiveExactSupportingOwner) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setKnownSideRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage_inout->setKnownSideDirection(
                    linkState_inout.passageEquation_world.head<3>()) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                            "%s: setKnownSideDirection rejected its input; "
                            "continuing as before.",
                            __func__);
            }
            p_farSideConfirmedRoom = linkState_inout.p_negativeSideRoom;
        }
        if (p_farSideConfirmedRoom != nullptr)
        {
            semantic::Room *p_previousFarSideHandle = nullptr;
            if (p_passage_inout->getProspectiveRoom(p_previousFarSideHandle) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage_inout->setProspectiveRoom(p_farSideConfirmedRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_farSideConfirmedRoom->addPassageAssociation(
                    p_passage_inout) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addPassageAssociation returned a failure "
                             "status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_previousFarSideHandle != p_farSideConfirmedRoom)
            {
                int passageId12{};
                if (p_passage_inout->getId(passageId12) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getId returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                int farSideConfirmedRoomId{};
                if (p_farSideConfirmedRoom->getId(farSideConfirmedRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getId returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                std::cout << "[SemMgr] Passage#" << passageId12
                          << " resolved to opposite confirmed Room#"
                          << farSideConfirmedRoomId
                          << " with both sides observed." << std::endl;
            }
        }
    }

    bool passageHasProspectiveRoom{};
    if (p_passage_inout->hasProspectiveRoom(passageHasProspectiveRoom) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasProspectiveRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!passageHasProspectiveRoom &&
        linkState_inout.confirmedAssociatedRoomCount == 1 &&
        linkState_inout.p_undefinedAssociatedRoom != nullptr)
    {
        if (p_passage_inout->setProspectiveRoom(
                linkState_inout.p_undefinedAssociatedRoom) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
