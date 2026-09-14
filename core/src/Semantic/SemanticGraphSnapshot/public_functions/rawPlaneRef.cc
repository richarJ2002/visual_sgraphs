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
 * @file            rawPlaneRef.cc
 *
 * @brief           Implements rawPlaneRef(), declared in
 *                  public_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/public_functions.h"

#include "Geometric/Plane.h"
#include "Map.h"

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

RawPlaneRef rawPlaneRef(Plane *p_plane_in)
{
    RawPlaneRef ref;
    if (p_plane_in == nullptr)
    {
        return ref;
    }
    ref.reason    = UnavailableReason::NONE;
    ref.planeId   = p_plane_in->getId();
    ref.isLive    = !p_plane_in->isBad();
    ref.planeType = p_plane_in->getPlaneType();
    Map *p_map    = p_plane_in->GetMap();
    if (p_map != nullptr)
    {
        ref.mapId = p_map->GetId();
        if (ref.planeType == Plane::planeVariant::WALL)
        {
            ref.wallKey =
                makeKey(EntityKind::WALL, p_map->GetId(), p_plane_in->getId());
        }
    }
    return ref;
}

} // namespace semantic
} // namespace ORB_SLAM3
