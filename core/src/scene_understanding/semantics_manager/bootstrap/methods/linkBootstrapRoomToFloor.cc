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
 * @file            linkBootstrapRoomToFloor.cc
 *
 * @brief           Implements SemanticsManager::linkBootstrapRoomToFloor(),
 *                  declared in
 * semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <optional>
#include <rclcpp/logging.hpp>

#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/semantics_manager/bootstrap/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

semantic::Floor *
    SemanticsManager::linkBootstrapRoomToFloor(map::Map     *p_activeMap_inout,
                                               unsigned long activeMapId_in,
                                               int           roomId_in,
                                               bool isCurrentRoomLive_in,
                                               semantic::Room *p_room_inout)
{
    /* Building and storey come before the room is linked into them. */
    semantic::Floor *p_canonicalFloor = ensureBootstrapFloor(activeMapId_in);
    if (p_canonicalFloor == nullptr)
    {
        printBootstrapRecord("FLOOR_CREATION_FAILED",
                             activeMapId_in,
                             roomId_in,
                             std::nullopt,
                             std::nullopt,
                             pipelineSemanticCycle);
        return nullptr;
    }
    if (p_canonicalFloor->addRoom(p_room_inout) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addRoom returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (!isCurrentRoomLive_in)
    {
        adoptBootstrapRoom(p_activeMap_inout, p_room_inout);
    }
    return p_canonicalFloor;
}

} /* namespace core */
} /* namespace vs_graphs */
