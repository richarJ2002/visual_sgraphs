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
 * @file            entityRefForFloor.cc
 *
 * @brief           Implements entityRefForFloor(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

#include "Map.h"

namespace ORB_SLAM3
{
namespace semantic
{

EntityRef entityRefForFloor(Floor *p_floor_in)
{
    EntityRef ref;
    if (p_floor_in == nullptr)
    {
        return ref;
    }
    /* Floor has no isBad()/setBad() in the current model (confirmed by
     * direct source read of Floor.h): a Floor has no liveness concept to
     * report at all, so isLive/livenessUnavailableReason keep their
     * defaults (isLive absent, reason NULL_REFERENCE from construction)
     * except that the reason must be corrected to
     * NOT_TRACKED_BY_CURRENT_SCHEMA below, since a real (non-null) target
     * exists even though its liveness cannot be read -- unlike
     * entityRefForRoom()'s explicit isBad() read, unknown liveness must
     * never be encoded as true. */
    ref.localId = p_floor_in->getId();
    ref.livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    Map *p_map = p_floor_in->getMap();
    if (p_map == nullptr)
    {
        ref.reason = UnavailableReason::ENTITY_HAS_NO_MAP;
        return ref;
    }
    ref.key = makeKey(EntityKind::FLOOR, p_map->GetId(), p_floor_in->getId());
    ref.reason = UnavailableReason::NONE;
    return ref;
}

} // namespace semantic
} // namespace ORB_SLAM3
