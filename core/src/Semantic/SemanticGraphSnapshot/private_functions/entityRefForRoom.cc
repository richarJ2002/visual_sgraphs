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

namespace vs_graphs
{
namespace core
{
namespace semantic
{

EntityRef entityRefForRoom(Room *p_room_in)
{
    EntityRef reference;
    if (p_room_in == nullptr)
    {
        return reference;
    }
    int room_inId{};
    if (p_room_in->getId(room_inId) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    reference.localId = room_inId;
    bool room_inIsBad{};
    if (p_room_in->isBad(room_inIsBad) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // isBad cannot fail; continue as before.
    }
    reference.isLive                    = !room_inIsBad;
    reference.livenessUnavailableReason = UnavailableReason::NONE;

    core::Map *p_map = nullptr;
    if (p_room_in->getMap(p_map) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getMap cannot fail; continue as before.
    }
    if (p_map == nullptr)
    {
        reference.reason = UnavailableReason::ENTITY_HAS_NO_MAP;
        return reference;
    }
    int room_inId2{};
    if (p_room_in->getId(room_inId2) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    reference.key    = makeKey(EntityKind::ROOM, p_map->getId(), room_inId2);
    reference.reason = UnavailableReason::NONE;
    return reference;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
