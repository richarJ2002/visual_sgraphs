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
 * @file            captureRoom.cc
 *
 * @brief           Implements captureRoom(), declared in
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

RoomRecord captureRoom(Room             *p_room_in,
                       long unsigned int mapId_in,
                       bool              isDetectedMember_in,
                       bool              isMarkerBasedMember_in)
{
    RoomRecord record;
    record.key    = makeKey(EntityKind::ROOM, mapId_in, p_room_in->getId());
    record.isLive = !p_room_in->isBad();
    record.isDetectedMember    = isDetectedMember_in;
    record.isMarkerBasedMember = isMarkerBasedMember_in;

    core::Map *p_declaredMap = p_room_in->getMap();
    if (p_declaredMap != nullptr)
    {
        record.declaredMapId = p_declaredMap->getId();
    }

    record.variant                 = p_room_in->getRoomVariant();
    record.centroid_World_m        = p_room_in->getCentroid();
    record.boundaryStatus          = p_room_in->getBoundaryStatus();
    record.boundaryCorners_World_m = p_room_in->getBoundaryCorners_World_m();
    record.observationGaps         = p_room_in->getObservationGaps();

    for (geometric::Plane *p_wall : p_room_in->getWalls())
    {
        appendWallRef(p_wall, record.wallRefs);
    }
    /* Full value-based total order (see isRawPlaneRefLess()), not merely a
     * stable pass-through of pre-sort order. */
    std::sort(record.wallRefs.begin(),
              record.wallRefs.end(),
              &isRawPlaneRefLess);

    for (Passage *p_passage : p_room_in->getPassages())
    {
        appendPassageRef(p_passage, record.passageRefs);
    }
    std::sort(record.passageRefs.begin(),
              record.passageRefs.end(),
              &isEntityRefLess);

    record.floorRef       = entityRefForFloor(p_room_in->getFloor());
    record.groundPlaneRef = rawPlaneRef(p_room_in->getGroundPlane());

    return record;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
