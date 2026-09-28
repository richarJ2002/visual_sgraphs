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
 * @file            captureFloor.cc
 *
 * @brief           Implements captureFloor(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

#include <algorithm>

#include "Map.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticGraphSnapshotStatus captureFloor(Floor            *p_floor_in,
                                         long unsigned int mapId_in,
                                         FloorRecord      &floorRecord_out)
{
    FloorRecord record;
    int         floor_inId{};
    if (p_floor_in->getId(floor_inId) != FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    EntityKey key2{};
    if (makeKey(EntityKind::FLOOR, mapId_in, floor_inId, key2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        // makeKey cannot fail; continue as before.
    }
    record.key = key2;

    core::Map *p_declaredMap = nullptr;
    if (p_floor_in->getMap(p_declaredMap) != FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getMap cannot fail; continue as before.
    }
    if (p_declaredMap != nullptr)
    {
        record.declaredMapId = p_declaredMap->getId();
    }

    Eigen::Vector3d floor_inCentroid{};
    if (p_floor_in->getCentroid(floor_inCentroid) !=
        FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getCentroid cannot fail; continue as before.
    }
    record.centroid_World_m = floor_inCentroid;
    /* A single getPlaneIdentity() read: calling hasPlaneIdentity() first
     * would lock and release Floor::geometryMutex a second time, so the
     * two calls together are not atomic with each other. The optional
     * already carries "absent" correctly on its own. */
    std::optional<Floor::PlaneIdentity> floor_inPlaneIdentity{};
    if (p_floor_in->getPlaneIdentity(floor_inPlaneIdentity) !=
        FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getPlaneIdentity cannot fail; continue as before.
    }
    record.planeIdentity = floor_inPlaneIdentity;

    std::vector<vs_graphs::core::semantic::Room *> floor_inRooms{};
    if (p_floor_in->getRooms(floor_inRooms) !=
        FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getRooms cannot fail; continue as before.
    }
    for (Room *p_room : floor_inRooms)
    {
        if (appendRoomRef(p_room, record.roomRefs) !=
            SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
        {
            // appendRoomRef cannot fail; continue as before.
        }
    }
    std::sort(record.roomRefs.begin(), record.roomRefs.end(), &isEntityRefLess);

    floorRecord_out = record;
    return SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
