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
 * @file            captureWall.cc
 *
 * @brief           Implements captureWall(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

#include "Map.h"

namespace ORB_SLAM3
{
namespace semantic
{

WallRecord captureWall(
    Plane                                           *p_wall_in,
    long unsigned int                                mapId_in,
    const std::map<Plane *, std::vector<EntityRef>> &wallOwnersByPointer_in)
{
    WallRecord record;
    record.key       = makeKey(EntityKind::WALL, mapId_in, p_wall_in->getId());
    record.isLive    = !p_wall_in->isBad();
    record.planeType = p_wall_in->getPlaneType();

    Map *p_declaredMap = p_wall_in->GetMap();
    if (p_declaredMap != nullptr)
    {
        record.declaredMapId = p_declaredMap->GetId();
    }

    const PlaneGeometryMetadataSnapshot geometry =
        p_wall_in->getGeometryMetadataSnapshot();
    record.equation_World            = geometry.equation_World;
    record.centroid_World_m          = geometry.centroid_World_m;
    record.minPlaneU_m               = geometry.minPlaneU_m;
    record.maxPlaneU_m               = geometry.maxPlaneU_m;
    record.minPlaneV_m               = geometry.minPlaneV_m;
    record.maxPlaneV_m               = geometry.maxPlaneV_m;
    record.finiteSupportCount        = geometry.finiteSupportCount;
    record.observationCount          = geometry.observationCount;
    record.cloudGeneration           = geometry.cloudGeneration;
    record.successfulRefitGeneration = geometry.successfulRefitGeneration;

    record.observationOrigin_World_m = p_wall_in->getObservationOrigin_World();
    record.twinRef                   = rawPlaneRef(p_wall_in->getTwinFace());

    const std::map<Plane *, std::vector<EntityRef>>::const_iterator ownerIt =
        wallOwnersByPointer_in.find(p_wall_in);
    if (ownerIt != wallOwnersByPointer_in.end())
    {
        record.ownerRoomRefs = ownerIt->second;
    }

    return record;
}

} // namespace semantic
} // namespace ORB_SLAM3
