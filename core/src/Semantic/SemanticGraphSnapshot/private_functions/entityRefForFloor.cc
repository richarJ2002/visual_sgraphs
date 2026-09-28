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
 * @file            entityRefForFloor.cc
 *
 * @brief           Implements entityRefForFloor(), declared in
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

SemanticGraphSnapshotStatus entityRefForFloor(Floor     *p_floor_in,
                                              EntityRef &entityRef_out)
{
    EntityRef reference;
    if (p_floor_in == nullptr)
    {
        entityRef_out = reference;
        return SemanticGraphSnapshotStatus::
            SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
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
    int floor_inId{};
    if (p_floor_in->getId(floor_inId) != FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    reference.localId = floor_inId;
    reference.livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    core::Map *p_map = nullptr;
    if (p_floor_in->getMap(p_map) != FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getMap cannot fail; continue as before.
    }
    if (p_map == nullptr)
    {
        reference.reason = UnavailableReason::ENTITY_HAS_NO_MAP;
        entityRef_out    = reference;
        return SemanticGraphSnapshotStatus::
            SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
    }
    int floor_inId2{};
    if (p_floor_in->getId(floor_inId2) != FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    EntityKey key2{};
    if (makeKey(EntityKind::FLOOR, p_map->getId(), floor_inId2, key2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        // makeKey cannot fail; continue as before.
    }
    reference.key    = key2;
    reference.reason = UnavailableReason::NONE;
    entityRef_out    = reference;
    return SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
