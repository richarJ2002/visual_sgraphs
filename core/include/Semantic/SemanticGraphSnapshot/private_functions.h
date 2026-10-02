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
 * @file            private_functions.h
 *
 * @brief           Declares the internal helpers used by
 *                  captureSemanticGraphSnapshot() (CPP_CODING_STANDARD.md
 *                  Section 5.4). Not exported or included by a public
 *                  header.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_PRIVATE_FUNCTIONS_H
#define SEMANTIC_GRAPH_SNAPSHOT_PRIVATE_FUNCTIONS_H

#include <cstdint>
#include <map>
#include <vector>

#include <Eigen/Core>

#include "Semantic/SemanticGraphSnapshot/SemanticGraphSnapshotStatus.h"
#include "Semantic/SemanticGraphSnapshot/objects.h"
#include "Semantic/SemanticGraphSnapshot/public_functions.h"
#include "Semantic/ValueOrder.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*! @brief Builds a map-qualified EntityKey from its three components. */
[[nodiscard]] SemanticGraphSnapshotStatus makeKey(EntityKind        kind_in,
                                                  long unsigned int mapId_in,
                                                  int               entityId_in,
                                                  EntityKey        &key_out);

/*! @brief Sorts a vector of any record type carrying a public `key` field
 *  in place: primarily by that key, and -- for a genuine EntityKey
 *  collision between two distinct source objects -- by every remaining
 *  captured value field in RecordT's documented canonical order (see the
 *  isValueLessForCollisionTiebreak() overload for RecordT), so the result
 *  no longer depends on the pointer-ordered container the pre-sort input
 *  happened to come from. */
template <typename RecordT>
[[nodiscard]] SemanticGraphSnapshotStatus
    sortByKey(std::vector<RecordT> &records_inout);

/*! @brief Builds an EntityRef to \p p_room_in, or explains why one could
 *  not be built (null pointer, or a non-null room with no map); localId is
 *  populated whenever \p p_room_in is non-null, even when no map is
 *  available to form key. isLive/livenessUnavailableReason are populated
 *  from Room::isBad() whenever localId has a value (Room has isBad()). */
[[nodiscard]] SemanticGraphSnapshotStatus
    entityRefForRoom(Room *p_room_in, EntityRef &entityRef_out);

/*! @brief Same as entityRefForRoom(), for a Floor. Floor has no isBad(), so
 *  isLive is always left absent with livenessUnavailableReason ==
 *  NOT_TRACKED_BY_CURRENT_SCHEMA whenever localId has a value -- unknown
 *  liveness is never encoded as true. */
[[nodiscard]] SemanticGraphSnapshotStatus
    entityRefForFloor(Floor *p_floor_in, EntityRef &entityRef_out);

/*! @brief Same as entityRefForRoom(), for a Passage. Passage has isBad(),
 *  so isLive/livenessUnavailableReason are populated from it whenever
 *  localId has a value, exactly as for Room. */
[[nodiscard]] SemanticGraphSnapshotStatus
    entityRefForPassage(Passage *p_passage_in, EntityRef &entityRef_out);

/*! rawPlaneRef() is declared in the module's public_functions.h (promoted
 *  from here so SemanticsManager can also build a RawPlaneRef for its own
 *  manager-private Plane* evidence). This is also the
 *  sole representation used for every wall-shaped reference (see
 *  appendWallRef()), replacing the removed entityRefForWall(), which
 *  mislabeled every referenced Plane as EntityKind::WALL without checking
 *  its actual PlaneVariant. */

/*! @brief Appends a RawPlaneRef for \p p_wall_in to \p refs_inout when
 *  non-null (regardless of map/liveness/type -- see RoomRecord::wallRefs);
 *  a null \p p_wall_in appends nothing (Room::setWalls() rejects null
 *  before insertion, so this is not currently reachable, but is handled
 *  safely regardless). Never dereferences a null pointer. */
[[nodiscard]] SemanticGraphSnapshotStatus
    appendWallRef(geometric::Plane         *p_wall_in,
                  std::vector<RawPlaneRef> &refs_inout);

/*! @brief Appends entityRefForRoom(\p p_room_in) to \p refs_inout when
 *  non-null (regardless of map/liveness -- see FloorRecord::roomRefs and
 *  WallRecord::ownerRoomRefs); a null \p p_room_in appends nothing. Never
 *  dereferences a null pointer. */
[[nodiscard]] SemanticGraphSnapshotStatus
    appendRoomRef(Room *p_room_in, std::vector<EntityRef> &refs_inout);

/*! @brief Same as appendRoomRef(), for Passage references (see
 *  RoomRecord::passageRefs). */
[[nodiscard]] SemanticGraphSnapshotStatus
    appendPassageRef(Passage *p_passage_in, std::vector<EntityRef> &refs_inout);

/*! doubleTotalOrderKey(), isDoubleLess(), isVector3dLess(), isVector4dLess(),
 *  isEntityRefLess(), and isRawPlaneRefLess() are declared in the shared
 *  public Semantic/ValueOrder.h (included above), not here: both this
 *  module and SemanticCanonicalSerialization need them, so they live behind
 *  a clean non-private boundary instead of one module including the
 *  other's private header. */

/*!
 * @brief       Strict weak "less than" comparing every RoomRecord field
 *              other than \p key, in this fixed canonical order: isLive,
 *              isDetectedMember, isMarkerBasedMember, declaredMapId
 *              presence/value, variant, centroid_World_m,
 *              boundaryStatus, boundaryCorners_World_m (size, then
 *              lexicographic), observationGaps (size, then lexicographic
 *              by startAngle_rad then spanAngle_rad), wallRefs (size, then
 *              lexicographic via isRawPlaneRefLess()), passageRefs (size,
 *              then lexicographic via isEntityRefLess()), floorRef (via
 *              isEntityRefLess()), groundPlaneRef (via
 *              isRawPlaneRefLess()), creationProvenanceReason.
 *
 *              Used only as a tiebreak by sortByKey<RoomRecord>() when
 *              lhs_in.key == rhs_in.key (a genuine EntityKey collision
 *              between two distinct Room objects); never called otherwise.
 *              Two records equal in every one of these fields are, by
 *              definition, indistinguishable value-wise and require no
 *              artificial distinction -- this function returns false for
 *              both orderings in that case, exactly like any other strict
 *              weak ordering over equal keys.
 */
bool isValueLessForCollisionTiebreak(const RoomRecord &lhs_in,
                                     const RoomRecord &rhs_in);

/*!
 * @brief       Same as the RoomRecord overload, for WallRecord: isLive,
 *              declaredMapId presence/value, planeType, equation_World,
 *              centroid_World_m, minPlaneU_m, maxPlaneU_m, minPlaneV_m,
 *              maxPlaneV_m, finiteSupportCount, observationCount,
 *              cloudGeneration, successfulRefitGeneration,
 *              observationOrigin_World_m presence/value,
 *              observationSideConsensusReason, twinRef (via
 *              isRawPlaneRefLess()), ownerRoomRefs (size, then
 *              lexicographic via isEntityRefLess()), quarantineReason,
 *              observationRayEvidenceReason.
 */
bool isValueLessForCollisionTiebreak(const WallRecord &lhs_in,
                                     const WallRecord &rhs_in);

/*!
 * @brief       Same as the RoomRecord overload, for PassageRecord: isLive,
 *              declaredMapId presence/value, passageType, equation_World,
 *              centroid_World_m, width_m, height_m, passable,
 *              associateWallRefs (size, then lexicographic via
 *              isRawPlaneRefLess()), associateDoorRef (via
 *              isRawPlaneRefLess()), knownSideRoomRef (via
 *              isEntityRefLess()), knownSideDirection_World presence/value,
 *              prospectiveRoomRef (via isEntityRefLess()),
 *              traversalKnownToFarCount, traversalFarToKnownCount,
 *              traversalUnknownCount, endpointSlotReason.
 */
bool isValueLessForCollisionTiebreak(const PassageRecord &lhs_in,
                                     const PassageRecord &rhs_in);

/*!
 * @brief       Same as the RoomRecord overload, for FloorRecord:
 *              declaredMapId presence/value, centroid_World_m,
 *              planeIdentity presence/value (equation_World,
 *              finiteSupportCount, observationCount), roomRefs (size, then
 *              lexicographic via isEntityRefLess()).
 */
bool isValueLessForCollisionTiebreak(const FloorRecord &lhs_in,
                                     const FloorRecord &rhs_in);

/*!
 * @brief       Captures one RoomRecord.
 *
 * @param[in]   p_room_in               Room to capture; must not be null.
 * @param[in]   mapId_in                Containing map id (the map this room
 *                                      was enumerated from).
 * @param[in]   isDetectedMember_in     Whether \p p_room_in was present in
 *                                      Map::GetAllDetectedMapRooms() for
 *                                      that map at capture time.
 * @param[in]   isMarkerBasedMember_in  Whether \p p_room_in was present in
 *                                      Map::GetAllMarkerBasedMapRooms() for
 *                                      that map at capture time.
 * @param[out]  roomRecord_out          Captured record.
 */
[[nodiscard]] SemanticGraphSnapshotStatus
    captureRoom(Room             *p_room_in,
                long unsigned int mapId_in,
                bool              isDetectedMember_in,
                bool              isMarkerBasedMember_in,
                RoomRecord       &roomRecord_out);

/*!
 * @brief       Captures one WallRecord.
 *
 * @param[in]   p_wall_in                  Wall to capture; must not be null.
 * @param[in]   mapId_in                   Containing map id.
 * @param[in]   wallOwnersByPointer_in     Pre-computed inversion of every
 *                                         live map's Room -> Wall
 *                                         ownership, keyed by wall pointer;
 *                                         see captureSemanticGraphSnapshot()'s
 *                                         implementation for how it is
 *                                         built.
 * @param[out]  wallRecord_out             Captured record.
 */
[[nodiscard]] SemanticGraphSnapshotStatus
    captureWall(geometric::Plane *p_wall_in,
                long unsigned int mapId_in,
                const std::map<geometric::Plane *, std::vector<EntityRef>>
                           &wallOwnersByPointer_in,
                WallRecord &wallRecord_out);

/*! @brief Captures one PassageRecord. \p p_passage_in must not be null. */
[[nodiscard]] SemanticGraphSnapshotStatus
    capturePassage(Passage          *p_passage_in,
                   long unsigned int mapId_in,
                   PassageRecord    &passageRecord_out);

/*! @brief Captures one FloorRecord. \p p_floor_in must not be null. */
[[nodiscard]] SemanticGraphSnapshotStatus
    captureFloor(Floor            *p_floor_in,
                 long unsigned int mapId_in,
                 FloorRecord      &floorRecord_out);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#include "Semantic/SemanticGraphSnapshot/private_functions/sortByKey.tpp"

#endif // SEMANTIC_GRAPH_SNAPSHOT_PRIVATE_FUNCTIONS_H
