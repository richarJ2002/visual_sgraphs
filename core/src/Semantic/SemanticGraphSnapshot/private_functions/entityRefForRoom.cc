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
 * @file            entityRefForRoom.cc
 *
 * @brief           Implements entityRefForRoom(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

#include "Map.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticGraphSnapshotStatus entityRefForRoom(Room      *p_room_in,
                                             EntityRef &entityRef_out)
{
    EntityRef reference;
    if (p_room_in == nullptr)
    {
        entityRef_out = reference;
        return SemanticGraphSnapshotStatus::
            SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
    }
    int room_inId{};
    if (p_room_in->getId(room_inId) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    reference.localId = room_inId;
    bool room_inIsBad{};
    if (p_room_in->isBad(room_inIsBad) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    reference.isLive                    = !room_inIsBad;
    reference.livenessUnavailableReason = UnavailableReason::NONE;

    core::Map *p_map = nullptr;
    if (p_room_in->getMap(p_map) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_map == nullptr)
    {
        reference.reason = UnavailableReason::ENTITY_HAS_NO_MAP;
        entityRef_out    = reference;
        return SemanticGraphSnapshotStatus::
            SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
    }
    int room_inId2{};
    if (p_room_in->getId(room_inId2) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    EntityKey     key2{};
    unsigned long mapId{};
    if (p_map->getId(mapId) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (makeKey(EntityKind::ROOM, mapId, room_inId2, key2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeKey returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    reference.key    = key2;
    reference.reason = UnavailableReason::NONE;
    entityRef_out    = reference;
    return SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
