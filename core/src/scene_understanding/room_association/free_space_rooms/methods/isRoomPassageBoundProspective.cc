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
 * @file            isRoomPassageBoundProspective.cc
 *
 * @brief           Implements
 * SemanticsManager::isRoomPassageBoundProspective(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/objects/SemanticsManagerClass.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::isRoomPassageBoundProspective(semantic::Room *p_room_in,
                                                    bool &isPassageBound_out)
{
    isPassageBound_out = false;

    std::vector<vs_graphs::core::semantic::Passage *> activePassages{};
    if (getAllPassagesInIdOrder(activePassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassagesInIdOrder returned a failure "
                     "status; continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::semantic::Passage *p_passage : activePassages)
    {
        vs_graphs::core::semantic::Room *p_passageProspectiveRoom = nullptr;
        if ((p_passage != nullptr) &&
            p_passage->getProspectiveRoom(p_passageProspectiveRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant roomVariant{};
        if ((p_passage != nullptr && p_passageProspectiveRoom == p_room_in) &&
            p_room_in->getRoomVariant(roomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getRoomVariant returned a failure status although "
                "it cannot fail; continuing as before.",
                __func__);
        }
        if (p_passage != nullptr && p_passageProspectiveRoom == p_room_in &&
            roomVariant ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            isPassageBound_out = true;
            break;
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
