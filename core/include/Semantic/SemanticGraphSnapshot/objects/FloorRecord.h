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
 * @file            FloorRecord.h
 *
 * @brief           Declares a value-only copy of one floor.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_FLOOR_RECORD_H
#define SEMANTIC_GRAPH_SNAPSHOT_FLOOR_RECORD_H

#include <optional>
#include <vector>

#include <Eigen/Core>

#include "Semantic/Floor.h"

#include "Semantic/SemanticGraphSnapshot/objects/EntityKey.h"
#include "Semantic/SemanticGraphSnapshot/objects/EntityRef.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Value-only copy of one floor. A Floor is not backed by a
 *              Plane object -- it stores its own lightweight
 *              Floor::PlaneIdentity value (equation, evidence counts) with
 *              no pointer to any Geometric/Plane.h instance.
 */
struct FloorRecord
{
  public:
    /*! @brief Map-qualified identity; mapId is the *containing* map used to
     *  enumerate this floor, which may differ from declaredMapId below. */
    EntityKey key;

    /*! @brief The floor's own Floor::getMap() result at capture time, as a
     *  map id, or absent when that call returned nullptr. Captured
     *  independently of key.mapId (the containing map) so a stored-in-A/
     *  declares-B-or-null mismatch is directly diagnosable. */
    std::optional<long unsigned int> declaredMapId;

    /*! @brief Floor::getCentroid() at capture time. */
    Eigen::Vector3d centroid_world_m{Eigen::Vector3d::Zero()};

    /*! @brief A single Floor::getPlaneIdentity() read; absent means the
     *  returned optional was empty (equivalently, Floor::hasPlaneIdentity()
     *  was false at that same instant -- the two are never queried
     *  separately, since each independently locks and releases
     *  Floor::geometryMutex and would not be atomic together). */
    std::optional<Floor::PlaneIdentity> planeIdentity;

    /*! @brief One EntityRef per non-null Floor::getRooms() entry, sorted
     *  deterministically. Unlike Room<->Passage, Floor keeps this
     *  reciprocal with Room::getFloor() by construction
     *  (Floor::addRoom()/setRooms()/replaceRoom() always update both
     *  sides) -- still captured here as plain data; the evaluator, not
     *  this snapshot, is where that invariant gets checked. Unlike the
     *  prior std::vector<EntityKey> representation, a bad or unmapped room
     *  here retains its own liveness/local-identity evidence. */
    std::vector<EntityRef> roomRefs;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_FLOOR_RECORD_H
