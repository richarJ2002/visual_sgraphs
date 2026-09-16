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
 * @file            RawPlaneRef.h
 *
 * @brief           Declares a reference to a Plane that is not itself
 *                  captured as a full WallRecord.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_RAW_PLANE_REF_H
#define SEMANTIC_GRAPH_SNAPSHOT_RAW_PLANE_REF_H

#include <optional>

#include "Geometric/Plane.h"

#include "Semantic/SemanticGraphSnapshot/objects/EntityKey.h"
#include "Semantic/SemanticGraphSnapshot/objects/UnavailableReason.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Raw identity of a referenced Plane, used both for a plane
 *              whose accepted Plane::PlaneVariant is not WALL by design
 *              (e.g. the DOOR plane a Passage names, or the GROUND plane a
 *              Room names) and, via wallKey below, for every wall-shaped
 *              reference (a Plane a caller expected to be WALL-typed: a
 *              wall's twin face, a Room's owned walls, a Passage's
 *              associated walls) instead of a WALL-kind EntityKey built
 *              without checking the actual Plane::PlaneVariant.
 *
 *              Invariant: reason == UnavailableReason::NONE exactly when the
 *              underlying Plane pointer was non-null at capture time, in
 *              which case planeId/isLive/planeType/mapId are populated from
 *              it. A default-constructed RawPlaneRef therefore defaults
 *              reason to NULL_REFERENCE, matching EntityRef's invariant --
 *              an untouched field must never silently read as "a plane is
 *              present." mapId is independently optional even when reason ==
 *              NONE, because the referenced plane, while non-null, may not
 *              itself have been assigned a map. wallKey is populated only
 *              when planeType is genuinely WALL and mapId has a value, so a
 *              Plane that is present, mapped, and live but is NOT actually
 *              WALL-typed (a caller/model contract violation) still reports
 *              its true planeId/mapId/isLive/planeType -- it is never
 *              silently dropped or fabricated into a nonexistent WallRecord
 *              identity.
 */
struct RawPlaneRef
{
  public:
    /*! @brief Present only when reason == UnavailableReason::NONE and the
     *  referenced plane itself had a non-null map at capture time. */
    std::optional<long unsigned int> mapId;

    /*! @brief The referenced plane's Atlas-assigned id; meaningful only
     *  when reason == UnavailableReason::NONE. */
    int planeId{0};

    /*! @brief Inverse of Plane::isBad() at capture time; meaningful only
     *  when reason == UnavailableReason::NONE. */
    bool isLive{true};

    /*! @brief The referenced plane's accepted Plane::PlaneVariant at
     *  capture time; meaningful only when reason ==
     *  UnavailableReason::NONE. */
    geometric::Plane::PlaneVariant planeType{geometric::Plane::PlaneVariant::UNDEFINED};

    /*! @brief UnavailableReason::NONE when a plane was actually referenced;
     *  UnavailableReason::NULL_REFERENCE (the default) when the underlying
     *  pointer was nullptr, which is the ordinary "no such plane yet"
     *  case for most callers of this type. */
    UnavailableReason reason{UnavailableReason::NULL_REFERENCE};

    /*! @brief Present only when reason == UnavailableReason::NONE,
     *  planeType == Plane::PlaneVariant::WALL, and mapId has a value: the
     *  key of the full WallRecord this same snapshot also captures for this
     *  plane. Absent whenever planeType is not WALL (the wrong-type target
     *  is still retained above via planeId/isLive/planeType/mapId, never
     *  fabricated as a WALL key) or the plane has no map (a WallRecord
     *  cannot exist for it either, for the same reason). */
    std::optional<EntityKey> wallKey;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_RAW_PLANE_REF_H
