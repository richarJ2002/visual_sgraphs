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
 * @file            private_functions.h
 *
 * @brief           Declares the internal helpers used by
 *                  serializeSnapshotTopologyOnly(),
 *                  serializeSnapshotFullGeometry(),
 *                  serializeEvaluationReport(), and
 *                  serializeMapCompletenessResults()
 *                  (CPP_CODING_STANDARD.md Section 5.4). Not exported or
 *                  included by a public header.
 *
 *                  Reuses the shared public Semantic/ValueOrder.h
 *                  total-order primitives (doubleTotalOrderKey(),
 *                  isDoubleLess(), isVector3dLess(), isVector4dLess(),
 *                  isEntityRefLess(), isRawPlaneRefLess()) so this module
 *                  and SemanticGraphSnapshot agree on exactly one canonical
 *                  order for these shared value types, without either
 *                  module including the other's private header. This
 *                  module owns its own record-collision comparators
 *                  (isRoomRecordLessTopologyOnly()/...FullGeometry(), etc.,
 *                  declared below) rather than reusing
 *                  SemanticGraphSnapshot's private
 *                  isValueLessForCollisionTiebreak() overloads, because
 *                  this module additionally needs a topology-only order
 *                  that is provably independent of every geometric field --
 *                  see serializeMapSnapshot.cc.
 */

#ifndef SEMANTIC_CANONICAL_SERIALIZATION_PRIVATE_FUNCTIONS_H
#define SEMANTIC_CANONICAL_SERIALIZATION_PRIVATE_FUNCTIONS_H

#include <vector>

#include "Thirdparty/nlohmann/json.hpp"

#include "Semantic/ValueOrder.h"

#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"
#include "Semantic/SemanticAxiomEvaluator/objects.h"
#include "Semantic/SemanticCanonicalSerialization/public_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{
/*! @brief Serializes \p value_in as a JSON number for every finite value
 *  (including signed zero, which nlohmann::json's own formatter already
 *  preserves losslessly and deterministically), or as one of the explicit
 *  string sentinels "NaN", "Infinity", "-Infinity" for a non-finite value
 *  -- strict JSON has no non-finite numeric literal, and nlohmann::json's
 *  own default numeric serialization silently collapses every non-finite
 *  double to JSON null, which this module's callers must never do. */
nlohmann::json serializeDouble(double value_in);

/*! @brief Serializes (x, y, z) as a 3-element JSON array via
 *  serializeDouble(). */
nlohmann::json serializeVector3d(const Eigen::Vector3d &value_in);

/*! @brief Serializes (x, y, z, w), Eigen's raw coefficient order for a
 *  4-vector, as a 4-element JSON array via serializeDouble(). */
nlohmann::json serializeVector4d(const Eigen::Vector4d &value_in);

/*! @brief Serializes {kind, mapId, entityId}; kind as its underlying
 *  integer value. */
nlohmann::json serializeEntityKey(const EntityKey &value_in);

/*! @brief Serializes every EntityRef field; key only when present,
 *  omitted (not null) otherwise, matching \p value_in's own has_value()
 *  invariants documented in EntityRef.h. */
nlohmann::json serializeEntityRef(const EntityRef &value_in);

/*! @brief Serializes every RawPlaneRef field, analogous to
 *  serializeEntityRef(). */
nlohmann::json serializeRawPlaneRef(const RawPlaneRef &value_in);

/*! @brief Serializes one RoomRecord. Every geometric field
 *  (centroid_World_m, boundaryCorners_World_m, observationGaps) is omitted
 *  unless \p includeGeometry_in. */
nlohmann::json serializeRoomRecord(const RoomRecord &value_in,
                                   bool              includeGeometry_in);

/*! @brief Serializes one WallRecord. Every geometric field (equation_World,
 *  centroid_World_m, minPlaneU_m/maxPlaneU_m/minPlaneV_m/maxPlaneV_m,
 *  finiteSupportCount, observationCount, cloudGeneration,
 *  successfulRefitGeneration, observationOrigin_World_m) is omitted unless
 *  \p includeGeometry_in. */
nlohmann::json serializeWallRecord(const WallRecord &value_in,
                                   bool              includeGeometry_in);

/*! @brief Serializes one PassageRecord. Every geometric field
 *  (equation_World, centroid_World_m, width_m, height_m,
 *  knownSideDirection_World) is omitted unless \p includeGeometry_in. */
nlohmann::json serializePassageRecord(const PassageRecord &value_in,
                                      bool                 includeGeometry_in);

/*! @brief Serializes one FloorRecord. centroid_World_m and planeIdentity
 *  are omitted unless \p includeGeometry_in. */
nlohmann::json serializeFloorRecord(const FloorRecord &value_in,
                                    bool               includeGeometry_in);

/*! @brief Serializes one MapSnapshot, propagating \p includeGeometry_in to
 *  every contained record. */
nlohmann::json serializeMapSnapshot(const MapSnapshot &value_in,
                                    bool               includeGeometry_in);

/*! @brief Serializes a complete SemanticGraphSnapshot (without the
 *  top-level "schema" field the two public entry points each add),
 *  propagating \p includeGeometry_in to every contained MapSnapshot. */
nlohmann::json serializeSnapshot(const SemanticGraphSnapshot &snapshot_in,
                                 bool includeGeometry_in);

/*! @brief Serializes one Finding, with involvedKeys re-sorted from a
 *  private copy. */
nlohmann::json serializeFinding(const Finding &value_in);

/*! @brief Serializes one AggregateAxiomResult. */
nlohmann::json
    serializeAggregateAxiomResult(const AggregateAxiomResult &value_in);

/*! @brief Serializes one LegacyMapCompletenessResult, with
 *  incompleteRoomIds/danglingPassageIds re-sorted from a private copy
 *  (preserving duplicates: legacy multiplicity/dangling counts are
 *  meaningful evidence, not deduplicated away). */
nlohmann::json serializeLegacyMapCompletenessResult(
    const LegacyMapCompletenessResult &value_in);

/*! @brief Serializes one MapCompletenessResult, with reasons/
 *  relevantEntityKeys re-sorted from a private copy. */
nlohmann::json
    serializeMapCompletenessResult(const MapCompletenessResult &value_in);

/*! @brief Serializes one OpenPassageHypothesisRecord. Every geometric field
 *  (centroid_World_m, openingRadius_m, heightSpan_m) is omitted unless
 *  \p includeGeometry_in. */
nlohmann::json serializeOpenPassageHypothesisRecord(
    const OpenPassageHypothesisRecord &value_in,
    bool                               includeGeometry_in);

/*! @brief Serializes one UnresolvedWallHypothesisRecord. */
nlohmann::json serializeUnresolvedWallHypothesisRecord(
    const UnresolvedWallHypothesisRecord &value_in);

/*! @brief Strict weak "less than" over every Finding field this
 *  serialization module emits (id, axiomCode, result, classification,
 *  reasonCode, involvedKeys lexicographically, then evidence:
 *  observedCount presence/value, expectedCount presence/value,
 *  numericValue presence/value via isDoubleLess()). \p lhs_in and \p
 *  rhs_in must already have involvedKeys sorted ascending (true for every
 *  production Finding; see Finding.h). Used by serializeEvaluationReport()
 *  so that two findings sharing the deterministic \c id (a genuine
 *  collision) but differing evidence still serialize in one fixed order
 *  regardless of input permutation. */
bool isFindingLessTotalOrder(const Finding &lhs_in, const Finding &rhs_in);

/*! @brief Strict weak "less than" over every AggregateAxiomResult field
 *  this module emits (axiomCode, result, classification,
 *  contributingFindingCount), used by serializeEvaluationReport() so two
 *  aggregates sharing axiomCode (a genuine collision) still serialize in
 *  one fixed order. */
bool isAggregateAxiomResultLessTotalOrder(const AggregateAxiomResult &lhs_in,
                                          const AggregateAxiomResult &rhs_in);

/*! @brief Strict weak "less than" over every MapCompletenessResult field
 *  this module emits (mapId, conservativeResult, isComplete, reasons
 *  lexicographically from a sorted copy, relevantEntityKeys
 *  lexicographically from a sorted copy, confirmedRoomCount,
 *  completeRoomCount, prospectiveRoomCount, livePassageCount,
 *  fullyValidPassageCount, legacy fields in serialized order,
 *  legacyAndConservativeDiverge), used by serializeMapCompletenessResults()
 *  so two results sharing mapId (a genuine collision) still serialize in
 *  one fixed order. */
bool isMapCompletenessResultLessTotalOrder(const MapCompletenessResult &lhs_in,
                                           const MapCompletenessResult &rhs_in);

/*! @brief Strict weak "less than" over every RoomRecord field the
 *  topology-only projection emits (key, isLive, isDetectedMember,
 *  isMarkerBasedMember, declaredMapId presence/value, variant,
 *  boundaryStatus, wallRefs (size, then lexicographic via
 *  isRawPlaneRefLess() over a sorted copy), passageRefs (size, then
 *  lexicographic via isEntityRefLess() over a sorted copy), floorRef (via
 *  isEntityRefLess()), groundPlaneRef (via isRawPlaneRefLess()),
 *  creationProvenanceReason). Deliberately never inspects a geometric
 *  field (centroid_World_m, boundaryCorners_World_m, observationGaps), so
 *  a geometry-only perturbation of otherwise key-colliding records can
 *  never reorder or alter the topology-only projection's bytes. */
bool isRoomRecordLessTopologyOnly(const RoomRecord &lhs_in,
                                  const RoomRecord &rhs_in);

/*! @brief Same as isRoomRecordLessTopologyOnly(), additionally comparing
 *  every field the full-geometry projection emits (centroid_World_m,
 *  boundaryCorners_World_m size/lexicographic via isVector3dLess(),
 *  observationGaps size/lexicographic by startAngle_rad then
 *  spanAngle_rad via isDoubleLess()) after the topology-only fields. */
bool isRoomRecordLessFullGeometry(const RoomRecord &lhs_in,
                                  const RoomRecord &rhs_in);

/*! @brief Same as isRoomRecordLessTopologyOnly(), for WallRecord's
 *  topology-only fields (key, isLive, declaredMapId presence/value,
 *  planeType, observationSideConsensusReason, twinRef, ownerRoomRefs
 *  (size, then lexicographic over a sorted copy), quarantineReason,
 *  observationRayEvidenceReason). */
bool isWallRecordLessTopologyOnly(const WallRecord &lhs_in,
                                  const WallRecord &rhs_in);

/*! @brief Same as isRoomRecordLessFullGeometry(), for WallRecord's
 *  geometric fields (equation_World, centroid_World_m, minPlaneU_m,
 *  maxPlaneU_m, minPlaneV_m, maxPlaneV_m, finiteSupportCount,
 *  observationCount, cloudGeneration, successfulRefitGeneration,
 *  observationOrigin_World_m presence/value), appended after the
 *  topology-only fields. */
bool isWallRecordLessFullGeometry(const WallRecord &lhs_in,
                                  const WallRecord &rhs_in);

/*! @brief Same as isRoomRecordLessTopologyOnly(), for PassageRecord's
 *  topology-only fields (key, isLive, declaredMapId presence/value,
 *  passageType, passable, associateWallRefs (size, then lexicographic
 *  over a sorted copy), associateDoorRef, knownSideRoomRef,
 *  prospectiveRoomRef, traversalKnownToFarCount, traversalFarToKnownCount,
 *  traversalUnknownCount, endpointSlotReason). */
bool isPassageRecordLessTopologyOnly(const PassageRecord &lhs_in,
                                     const PassageRecord &rhs_in);

/*! @brief Same as isRoomRecordLessFullGeometry(), for PassageRecord's
 *  geometric fields (equation_World, centroid_World_m, width_m, height_m,
 *  knownSideDirection_World presence/value), appended after the
 *  topology-only fields. */
bool isPassageRecordLessFullGeometry(const PassageRecord &lhs_in,
                                     const PassageRecord &rhs_in);

/*! @brief Same as isRoomRecordLessTopologyOnly(), for FloorRecord's
 *  topology-only fields (key, declaredMapId presence/value, roomRefs
 *  (size, then lexicographic over a sorted copy)). */
bool isFloorRecordLessTopologyOnly(const FloorRecord &lhs_in,
                                   const FloorRecord &rhs_in);

/*! @brief Same as isRoomRecordLessFullGeometry(), for FloorRecord's
 *  geometric fields (centroid_World_m, planeIdentity presence, then
 *  equation_World/finiteSupportCount/observationCount), appended after
 *  the topology-only fields. */
bool isFloorRecordLessFullGeometry(const FloorRecord &lhs_in,
                                   const FloorRecord &rhs_in);

/*! @brief Strict weak "less than" over every field one MapSnapshot's
 *  top-level projection emits (mapId, isCurrentMap, then rooms/walls/
 *  passages/floors compared as size-then-lexicographic multisets using
 *  the matching topology-only or full-geometry record comparator above,
 *  selected by \p includeGeometry_in), used by serializeSnapshot() so two
 *  MapSnapshot entries sharing mapId (a genuine collision) still
 *  serialize in one fixed order for the requested projection. */
bool isMapSnapshotLessTotalOrder(const MapSnapshot &lhs_in,
                                 const MapSnapshot &rhs_in,
                                 bool               includeGeometry_in);

/*! @brief Strict weak "less than" over every OpenPassageHypothesisRecord
 *  field the topology-only projection emits (supportingWallRef via
 *  isRawPlaneRefLess(), confirmationCount, missedUpdateCount,
 *  lastConfirmedSkeletonFingerprint). Deliberately never inspects a
 *  geometric field (centroid_World_m, openingRadius_m, heightSpan_m), so a
 *  geometry-only perturbation of otherwise colliding records can never
 *  reorder or alter the topology-only projection's bytes. */
bool isOpenPassageHypothesisRecordLessTopologyOnly(
    const OpenPassageHypothesisRecord &lhs_in,
    const OpenPassageHypothesisRecord &rhs_in);

/*! @brief Same as isOpenPassageHypothesisRecordLessTopologyOnly(),
 *  additionally comparing every field the full-geometry projection emits
 *  (centroid_World_m via isVector3dLess(), openingRadius_m, heightSpan_m
 *  via isDoubleLess()) after the topology-only fields. */
bool isOpenPassageHypothesisRecordLessFullGeometry(
    const OpenPassageHypothesisRecord &lhs_in,
    const OpenPassageHypothesisRecord &rhs_in);

/*! @brief Strict weak "less than" over every UnresolvedWallHypothesisRecord
 *  field (wallRef via isRawPlaneRefLess(), unresolvedCycles,
 *  cloudPointCount, observationCount). This record carries no geometric
 *  field, so one total order already serves both the topology-only and
 *  full-geometry projections identically. */
bool isUnresolvedWallHypothesisRecordLessTotalOrder(
    const UnresolvedWallHypothesisRecord &lhs_in,
    const UnresolvedWallHypothesisRecord &rhs_in);

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_CANONICAL_SERIALIZATION_PRIVATE_FUNCTIONS_H
