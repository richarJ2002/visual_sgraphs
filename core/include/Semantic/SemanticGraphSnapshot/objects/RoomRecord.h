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
 * @file            RoomRecord.h
 *
 * @brief           Declares a value-only copy of one committed or candidate
 *                  room.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_ROOM_RECORD_H
#define SEMANTIC_GRAPH_SNAPSHOT_ROOM_RECORD_H

#include <optional>
#include <vector>

#include <Eigen/Core>

#include "Semantic/Room.h"

#include "Semantic/SemanticGraphSnapshot/objects/EntityKey.h"
#include "Semantic/SemanticGraphSnapshot/objects/EntityRef.h"
#include "Semantic/SemanticGraphSnapshot/objects/RawPlaneRef.h"
#include "Semantic/SemanticGraphSnapshot/objects/UnavailableReason.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Value-only copy of one committed or candidate room.
 */
struct RoomRecord
{
  public:
    /*! @brief Map-qualified identity; mapId is the *containing* map used to
     *  enumerate this room, which may differ from declaredMapId below. */
    EntityKey key;

    /*! @brief Inverse of Room::isBad(): false means the room is retired. */
    bool isLive{true};

    /*! @brief True when this room's pointer was present in
     *  Map::GetAllDetectedMapRooms() at capture time. Independent of
     *  isMarkerBasedMember: the current model allows a Room pointer to be
     *  registered in both collections simultaneously, and a malformed
     *  object present in both must remain diagnosable rather than
     *  collapsed into one flag. */
    bool isDetectedMember{false};

    /*! @brief True when this room's pointer was present in
     *  Map::GetAllMarkerBasedMapRooms() (== Map::GetAllCandidateMapRooms())
     *  at capture time. See isDetectedMember. */
    bool isMarkerBasedMember{false};

    /*! @brief The room's own Room::getMap() result at capture time, as a
     *  map id, or absent when that call returned nullptr. Captured
     *  independently of key.mapId (the containing map) so a stored-in-A/
     *  declares-B-or-null mismatch is directly diagnosable. */
    std::optional<long unsigned int> declaredMapId;

    /*! @brief Room::getRoomVariant() at capture time. */
    Room::roomVariant variant{Room::roomVariant::UNDEFINED};

    /*! @brief Room::getCentroid() at capture time. May reflect a
     *  concurrent bundle-adjustment update (Optimizer.cc calls
     *  Room::setCentroid() outside the semantic-update lock); this is a
     *  pre-existing, mutex-protected, non-racy characteristic shared by
     *  every other centroid consumer in this codebase, not a defect
     *  introduced by capture. */
    Eigen::Vector3d centroid_World_m{Eigen::Vector3d::Zero()};

    /*! @brief Room::getBoundaryStatus() at capture time. */
    Room::BoundaryStatus boundaryStatus{Room::BoundaryStatus::UNOBSERVED};

    /*! @brief As currently stored; Room.h documents these as populated
     *  only while boundaryStatus == COMPLETE, but this snapshot copies
     *  whatever is present without gating on status -- interpretation is
     *  the evaluator's responsibility. */
    std::vector<Eigen::Vector3d> boundaryCorners_World_m;

    /*! @brief Populated every cycle regardless of boundaryStatus. */
    std::vector<Room::ObservationGap> observationGaps;

    /*! @brief One RawPlaneRef per non-null Room::getWalls() entry, sorted
     *  deterministically by (mapId, planeId, planeType); independent of any
     *  reverse (Plane -> owning room) computation -- see
     *  WallRecord::ownerRoomRefs for the inverted view this snapshot also
     *  builds. A genuinely null Room::getWalls() entry (not currently
     *  reachable: Room::setWalls() rejects a null pointer before insertion,
     *  confirmed by direct source read) produces no entry at all -- an
     *  ordinary "nothing there" case, not evidence. Every non-null entry is
     *  retained here regardless of map/liveness/type, including a wrong-type
     *  (planeType != WALL) target, which RawPlaneRef reports truthfully
     *  instead of this snapshot fabricating a WALL identity for it. Use
     *  RawPlaneRef::wallKey to look up the corresponding WallRecord when the
     *  target is genuinely WALL-typed and mapped. */
    std::vector<RawPlaneRef> wallRefs;

    /*! @brief One EntityRef per non-null Room::getPassages() entry (the
     *  Room::doorways side of the relationship), sorted deterministically.
     *  Captured independently of each Passage's own knownSideProvenance/
     *  prospectiveRoom fields -- the two directions are not kept
     *  synchronized by the current model, so the evaluator cross-checks
     *  them from both independently-captured sides. Unlike the prior
     *  std::vector<EntityKey> representation, a bad or unmapped passage
     *  here retains its own liveness/local-identity evidence instead of
     *  losing it the moment it cannot be map-qualified keyed. */
    std::vector<EntityRef> passageRefs;

    /*! @brief Absent means "no floor yet", an ordinary case. */
    EntityRef floorRef;

    /*! @brief The room's ground plane, if any (a GROUND-typed Plane, not
     *  captured as a WallRecord). reason == NULL_REFERENCE means "no
     *  ground plane yet", the ordinary case. */
    RawPlaneRef groundPlaneRef;

    /*! @brief Always NOT_TRACKED_BY_CURRENT_SCHEMA in this slice: Room has
     *  no field recording how/why it was created (confirmed by direct
     *  source read of Room.h/Room.cc -- mRoomTag, mpMatchedContext, and
     *  the meta-marker fields record identity/labelling, not creation
     *  provenance). A later phase that adds a provenance field on Room is
     *  the owner of resolving this to an actual value. */
    UnavailableReason creationProvenanceReason{
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA};
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_ROOM_RECORD_H
