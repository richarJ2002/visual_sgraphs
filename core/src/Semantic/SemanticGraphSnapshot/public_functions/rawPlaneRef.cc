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
    reference.reason    = UnavailableReason::NONE;
    reference.planeId   = p_plane_in->getId();
    reference.isLive    = !p_plane_in->isBad();
    reference.planeType = p_plane_in->getPlaneType();
    core::Map *p_map    = p_plane_in->getMap();
    if (p_map != nullptr)
    {
        reference.mapId = p_map->getId();
        if (reference.planeType == geometric::Plane::PlaneVariant::WALL)
        {
            EntityKey key{};
            if (makeKey(EntityKind::WALL,
                        p_map->getId(),
                        p_plane_in->getId(),
                        key) != SemanticGraphSnapshotStatus::
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
