/**
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

namespace ORB_SLAM3
{
namespace semantic
{

EntityRef entityRefForRoom(Room *p_room_in)
{
    EntityRef ref;
    if (p_room_in == nullptr)
    {
        return ref;
    }
    ref.localId                   = p_room_in->getId();
    ref.isLive                    = !p_room_in->isBad();
    ref.livenessUnavailableReason = UnavailableReason::NONE;

    Map *p_map = p_room_in->getMap();
    if (p_map == nullptr)
    {
        ref.reason = UnavailableReason::ENTITY_HAS_NO_MAP;
        return ref;
    }
    ref.key    = makeKey(EntityKind::ROOM, p_map->GetId(), p_room_in->getId());
    ref.reason = UnavailableReason::NONE;
    return ref;
}

} // namespace semantic
} // namespace ORB_SLAM3
