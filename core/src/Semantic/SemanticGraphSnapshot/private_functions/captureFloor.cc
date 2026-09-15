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

FloorRecord captureFloor(Floor *p_floor_in, long unsigned int mapId_in)
{
    FloorRecord record;
    record.key = makeKey(EntityKind::FLOOR, mapId_in, p_floor_in->getId());

    core::Map *p_declaredMap = p_floor_in->getMap();
    if (p_declaredMap != nullptr)
    {
        record.declaredMapId = p_declaredMap->GetId();
    }

    record.centroid_World_m = p_floor_in->getCentroid();
    /* A single getPlaneIdentity() read: calling hasPlaneIdentity() first
     * would lock and release Floor::mMutexGeometry a second time, so the
     * two calls together are not atomic with each other. The optional
     * already carries "absent" correctly on its own. */
    record.planeIdentity = p_floor_in->getPlaneIdentity();

    for (Room *p_room : p_floor_in->getRooms())
    {
        appendRoomRef(p_room, record.roomRefs);
    }
    std::sort(record.roomRefs.begin(), record.roomRefs.end(), &isEntityRefLess);

    return record;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
