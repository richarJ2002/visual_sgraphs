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
 * @file            rawPlaneRef.cc
 *
 * @brief           Implements rawPlaneRef(), declared in
 *                  public_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/public_functions.h"

#include "Geometric/Plane.h"
#include "Map.h"

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticGraphSnapshotStatus rawPlaneRef(geometric::Plane *p_plane_in,
                                        RawPlaneRef      &rawPlaneRef_out)
{
    RawPlaneRef reference;
    if (p_plane_in == nullptr)
    {
        rawPlaneRef_out = reference;
        return SemanticGraphSnapshotStatus::
            SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
    }
    reference.reason = UnavailableReason::NONE;
    int planeGetId{};
    if (p_plane_in->getId(planeGetId) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    reference.planeId = planeGetId;
    bool planeIsBad{};
    if (p_plane_in->isBad(planeIsBad) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // isBad cannot fail; continue as before.
    }
    reference.isLive = !planeIsBad;
    geometric::Plane::PlaneVariant planeType2{};
    if (p_plane_in->getPlaneType(planeType2) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // getPlaneType cannot fail; continue as before.
    }
    reference.planeType = planeType2;
    core::Map *p_map    = nullptr;
    if (p_plane_in->getMap(p_map) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        // getMap cannot fail; continue as before.
    }
    if (p_map != nullptr)
    {
        reference.mapId = p_map->getId();
        if (reference.planeType == geometric::Plane::PlaneVariant::WALL)
        {
            EntityKey key{};
            int       planeGetId2{};
            if (p_plane_in->getId(planeGetId2) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (makeKey(EntityKind::WALL, p_map->getId(), planeGetId2, key) !=
                SemanticGraphSnapshotStatus::
                    SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
            {
                // makeKey cannot fail; continue as before.
            }
            reference.wallKey = key;
        }
    }
    rawPlaneRef_out = reference;
    return SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
