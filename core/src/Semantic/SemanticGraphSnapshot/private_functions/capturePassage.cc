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
 * @file            capturePassage.cc
 *
 * @brief           Implements capturePassage(), declared in
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

PassageRecord capturePassage(Passage *p_passage_in, long unsigned int mapId_in)
{
    PassageRecord record;
    record.key = makeKey(EntityKind::PASSAGE, mapId_in, p_passage_in->getId());
    record.isLive = !p_passage_in->isBad();

    core::Map *p_declaredMap = p_passage_in->getMap();
    if (p_declaredMap != nullptr)
    {
        record.declaredMapId = p_declaredMap->getId();
    }

    record.passageType      = p_passage_in->getPassageType();
    record.equation_World   = p_passage_in->getGlobalEquation().coeffs();
    record.centroid_World_m = p_passage_in->getCentroid();
    record.width_m          = p_passage_in->getWidth();
    record.height_m         = p_passage_in->getHeight();
    record.isPassable       = p_passage_in->isPassable();

    for (geometric::Plane *p_wall : p_passage_in->getAssociateWalls())
    {
        appendWallRef(p_wall, record.associateWallRefs);
    }
    std::sort(record.associateWallRefs.begin(),
              record.associateWallRefs.end(),
              &isRawPlaneRefLess);

    record.associateDoorRef = rawPlaneRef(p_passage_in->getAssociateDoor());

    const Passage::KnownSideProvenance provenance =
        p_passage_in->getKnownSideProvenance();
    record.knownSideRoomRef = entityRefForRoom(provenance.p_room);
    if (provenance.hasDirection())
    {
        record.knownSideDirection_World = provenance.direction_World;
    }

    record.prospectiveRoomRef =
        entityRefForRoom(p_passage_in->getProspectiveRoom());

    record.traversalKnownToFarCount =
        p_passage_in->getTraversalKnownToFarCount();
    record.traversalFarToKnownCount =
        p_passage_in->getTraversalFarToKnownCount();
    record.traversalUnknownCount = p_passage_in->getTraversalUnknownCount();

    return record;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
