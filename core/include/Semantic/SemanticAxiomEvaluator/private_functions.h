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
 * @brief           Declares the internal helpers used by evaluateState(),
 *                  evaluateTransition(), and evaluateMapCompleteness()
 *                  (CPP_CODING_STANDARD.md Section 5.4). Not exported or
 *                  included by a public header.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_PRIVATE_FUNCTIONS_H
#define SEMANTIC_AXIOM_EVALUATOR_PRIVATE_FUNCTIONS_H

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "Semantic/SemanticGraphSnapshot/objects.h"
#include "Semantic/ValueOrder.h"

#include "Semantic/SemanticAxiomEvaluator/objects.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*! @brief Section 5's fixed "Class" column value for \p axiomCode_in. */
AxiomClass axiomClassFor(AxiomCode axiomCode_in);

/*! @brief Constructs one AxiomCapabilityEntry row, deriving \c classification
 *  from \p axiomCode_in via axiomClassFor() so it can never drift from
 *  Section 5's own "Class" column. Used only by axiomCapabilityTable(). */
AxiomCapabilityEntry makeAxiomCapabilityEntry(AxiomCode         axiomCode_in,
                                              CapabilityLevel   capability_in,
                                              MissingProofOwner owner_in);

/*! @brief Constructs one Finding with a deterministic \c id derived only
 *  from \p axiomCode_in, \p reasonCode_in, and \p involvedKeys_in (sorted
 *  and deduplicated in place before the id is built) -- see makeFinding.cc
 *  for the exact textual encoding. The sole production constructor for
 *  Finding::id; every per-axiom evaluator uses this instead of building a
 *  Finding by hand. */
Finding makeFinding(AxiomCode              axiomCode_in,
                    AxiomResult            result_in,
                    ReasonCode             reasonCode_in,
                    std::vector<EntityKey> involvedKeys_in,
                    FindingEvidence        evidence_in = FindingEvidence{});

/*! @brief Sorts \p findings_inout ascending by Finding::id in place. Two
 *  Findings can never compare equal under this order unless they are
 *  identical in every field the id is derived from (axiomCode, reasonCode,
 *  involvedKeys), since id is a complete, collision-free encoding of
 *  exactly those three fields for the bounded value ranges this module
 *  produces -- see makeFinding.cc. */
void sortFindings(std::vector<Finding> &findings_inout);

/*! @brief Groups \p findings_in by AxiomCode and applies FAIL > UNKNOWN >
 *  PASS precedence to produce exactly sixteen AggregateAxiomResult entries,
 *  sorted by AxiomCode, one per Section-5 code regardless of how many (if
 *  any) findings contributed. */
std::vector<AggregateAxiomResult>
    aggregateFindings(const std::vector<Finding> &findings_in);

/*! @brief Returns the first record in \p records_in (sorted ascending by
 *  RecordT::key, as every MapSnapshot record vector is documented to be)
 *  whose key equals \p key_in, or nullptr when none does. Non-owning:
 *  the returned pointer is valid exactly as long as \p records_in is. */
template <typename RecordT>
const RecordT *findRecordByKey(const std::vector<RecordT> &records_in,
                               const EntityKey            &key_in);

/*! @brief Resolves \p ref_in (an EntityRef expected to name a Room) against
 *  \p snapshot_in, comparing its key's mapId to \p expectedMapId_in. See
 *  ResolvedRoomEndpoint.h for exactly what each output field means and
 *  resolveRoomEndpoint.cc for why isLive/isConfirmedRoomVariant (not a
 *  not-yet-existing authoritative endpoint-slot field) are this slice's
 *  documented proxy for "real"/"confirmed". */
ResolvedRoomEndpoint
    resolveRoomEndpoint(const EntityRef             &ref_in,
                        long unsigned int            expectedMapId_in,
                        const SemanticGraphSnapshot &snapshot_in);

/*! @brief True when \p endpoint_in is found in the snapshot, live, and
 *  ROOM-variant -- this module's documented proxy for a "real"/"confirmed"
 *  passage endpoint (see resolveRoomEndpoint.cc's Doxygen). */
bool isRealPassageEndpoint(const ResolvedRoomEndpoint &endpoint_in);

/*! @brief True when \p endpoint_in's forward reference is present and
 *  carries at least one independently observable contradiction: wrong
 *  EntityKind, an unresolvable (mapless) target, ambiguous duplicate
 *  identity, a source EntityRef key/reason invariant violation, a located
 *  target record whose own declaredMapId disagrees with the map it was
 *  found in, or a known (available and false) bad liveness value.
 *  Deliberately excludes cross-map placement (see isCrossMap): AX-PASS-02/
 *  03/04 and AX-FLOOR-01's passage branch each validate cross-map placement
 *  explicitly with their own dedicated reason code. Shared by every
 *  AX-PASS-02/03/04 and AX-FLOOR-01 passage evaluator so a known-invalid
 *  forward reference is rejected identically everywhere, rather than only
 *  in cardinality's own checks -- an endpoint that is merely absent, or
 *  whose liveness is genuinely unavailable rather than known false, is not
 *  "known invalid" by this predicate (see resolveRoomEndpoint.cc's
 *  isLiveAvailable Doxygen). 2026-09-07 second proof-closure repair;
 *  isReasonInconsistent added by the Checkpoint-A residual repair. */
bool isKnownInvalidPassageEndpointReference(
    const ResolvedRoomEndpoint &endpoint_in);

/*! @brief True when the live RoomRecord keyed \p roomKey_in (in the map
 *  named by \p roomKey_in.mapId) has a passageRefs entry naming
 *  \p passageKey_in; false when that room cannot be located at all. */
bool roomListsPassageBack(const SemanticGraphSnapshot &snapshot_in,
                          const EntityKey             &roomKey_in,
                          const EntityKey             &passageKey_in);

/*! @brief Counts how many RoomRecord entries in the map named by
 *  \p key_in.mapId share the exact key \p key_in; 0 when that map is not
 *  present in \p snapshot_in. Used to detect duplicate-identity ambiguity. */
std::size_t countRoomRecordsWithKey(const SemanticGraphSnapshot &snapshot_in,
                                    const EntityKey             &key_in);

/*! @brief Counts how many FloorRecord entries across every MapSnapshot in
 *  \p snapshot_in whose own mapId equals \p key_in.mapId share the exact
 *  key \p key_in; 0 when no such map is present. Used to detect
 *  duplicate-identity ambiguity. 2026-09-07 second proof-closure repair:
 *  now snapshot-wide (like countRoomRecordsWithKey/countWallRecordsWithKey)
 *  rather than scoped to one caller-chosen MapSnapshot, so a duplicate
 *  MapSnapshot::mapId cannot hide a same-key floor duplicated across the
 *  two map snapshots. */
std::size_t countFloorRecordsWithKey(const SemanticGraphSnapshot &snapshot_in,
                                     const EntityKey             &key_in);

/*! @brief Counts how many WallRecord entries across every MapSnapshot in
 *  \p snapshot_in whose own mapId equals \p key_in.mapId share the exact
 *  key \p key_in; 0 when no such map is present. Used to detect
 *  duplicate-identity ambiguity. 2026-09-07 residual proof-closure repair;
 *  made snapshot-wide (summed across every matching-mapId MapSnapshot,
 *  not only the first) by the 2026-09-07 second proof-closure repair. */
std::size_t countWallRecordsWithKey(const SemanticGraphSnapshot &snapshot_in,
                                    const EntityKey             &key_in);

/*! @brief Counts how many PassageRecord entries across every MapSnapshot in
 *  \p snapshot_in whose own mapId equals \p key_in.mapId share the exact
 *  key \p key_in; 0 when no such map is present. Used to detect
 *  duplicate-identity ambiguity. 2026-09-07 residual proof-closure repair;
 *  made snapshot-wide by the 2026-09-07 second proof-closure repair (see
 *  countFloorRecordsWithKey's own Doxygen). */
std::size_t countPassageRecordsWithKey(const SemanticGraphSnapshot &snapshot_in,
                                       const EntityKey             &key_in);

/*! @brief Counts how many MapSnapshot entries in \p snapshot_in.maps share
 *  the exact mapId \p mapId_in: a duplicate-map-identity preflight so no
 *  first-matching-map lookup anywhere in this module can silently prefer
 *  one of two ambiguous MapSnapshot entries over the other. 2026-09-07
 *  second proof-closure repair. */
std::size_t countMapSnapshotsWithId(const SemanticGraphSnapshot &snapshot_in,
                                    long unsigned int            mapId_in);

/*! @brief Scans every RoomRecord (live or retired) in every map of
 *  \p snapshot_in for a passageRefs entry naming \p passage_in, independent
 *  of \p passage_in's own forward knownSideRoomRef/prospectiveRoomRef
 *  fields, and classifies every match -- see ReversePassageEndpointScan.h and
 *  scanReversePassageEndpoints.cc. \p expectedMapId_in is \p passage_in's
 *  own containing/declared map id, used to classify a reverse reference
 *  from a different map as cross-map rather than confirmed. */
ReversePassageEndpointScan
    scanReversePassageEndpoints(const PassageRecord         &passage_in,
                                long unsigned int            expectedMapId_in,
                                const SemanticGraphSnapshot &snapshot_in);

/*! @brief Appends one AX-PASS-02 Finding for \p passage_in: forward
 *  endpoint checks (unresolvable/bad/cross-map/duplicate), the reverse
 *  scan's anomaly checks (unresolvable/bad/cross-map/duplicate-identity
 *  reverse reference, third confirmed endpoint), then cardinality/
 *  reciprocity, in that priority order. Never PASS in this slice -- see
 *  this function's own Doxygen in evaluateOnePassageCardinality.cc. */
void evaluateOnePassageCardinality(const PassageRecord         &passage_in,
                                   const SemanticGraphSnapshot &snapshot_in,
                                   std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-PASS-03 Finding for \p passage_in. */
void evaluateOnePassageSlotState(const PassageRecord         &passage_in,
                                 const SemanticGraphSnapshot &snapshot_in,
                                 std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-PASS-04 Finding for \p passage_in. */
void evaluateOnePassageMapAndFloor(const PassageRecord         &passage_in,
                                   const SemanticGraphSnapshot &snapshot_in,
                                   std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-FLOOR-01 Finding for \p room_in's room-floor
 *  reciprocity, scanning every FloorRecord in \p mapSnapshot_in (not only
 *  the one \p room_in.floorRef names) for duplicate identity, duplicate
 *  reverse membership, and another floor also claiming \p room_in.
 *  \p snapshot_in is used only for the snapshot-wide
 *  countFloorRecordsWithKey() duplicate-identity check (2026-09-07 second
 *  proof-closure repair: that helper is now snapshot-wide, see its own
 *  Doxygen). */
void evaluateOneRoomFloorReciprocity(const RoomRecord            &room_in,
                                     const SemanticGraphSnapshot &snapshot_in,
                                     const MapSnapshot    &mapSnapshot_in,
                                     std::vector<Finding> &findings_inout);

/*! @brief Appends one AX-FLOOR-01 Finding for \p passage_in's endpoint
 *  floor identity. */
void evaluateOnePassageFloorIdentity(const PassageRecord         &passage_in,
                                     const SemanticGraphSnapshot &snapshot_in,
                                     std::vector<Finding> &findings_inout);

/*! @brief The aggregate AxiomResult (FAIL/UNKNOWN/PASS) of \p endpoint_in's
 *  own room/floor relationship, found via evaluateOneRoomFloorReciprocity()
 *  against \p snapshot_in. \p endpoint_in must already be a real
 *  (isFoundInSnapshot && isLive && isConfirmedRoomVariant) endpoint; PASS is
 *  returned (vacuously, "no known problem") when the room cannot itself be
 *  located, matching the conservative default elsewhere in this module.
 *  Checkpoint-A residual repair: replaces the lossy
 *  hasFailingRoomFloorReciprocity() bool (FAIL-or-not) so
 *  evaluatePassageFloorAgreement() can also propagate a canonical UNKNOWN,
 *  not only FAIL. */
AxiomResult
    canonicalRoomFloorResultFor(const ResolvedRoomEndpoint  &endpoint_in,
                                const SemanticGraphSnapshot &snapshot_in);

/*! @brief Compares \p knownSide_in and \p prospective_in's own floorKey,
 *  restricted to endpoints that are themselves real (isFoundInSnapshot &&
 *  isLive && isConfirmedRoomVariant). An equal floorKey on both sides is
 *  additionally resolved against \p snapshot_in's own FloorRecord
 *  collections: a dangling key naming no actual FloorRecord is
 *  EVIDENCE_UNAVAILABLE, not AGREE. */
PassageFloorAgreement
    evaluatePassageFloorAgreement(const ResolvedRoomEndpoint  &knownSide_in,
                                  const ResolvedRoomEndpoint  &prospective_in,
                                  const SemanticGraphSnapshot &snapshot_in);

/*! @brief True when any corner has a non-finite (NaN or Infinity)
 *  coordinate. */
bool hasNonFiniteCoordinate(const std::vector<Eigen::Vector3d> &corners_in);

/*! @brief Newell's-method best-fit polygon normal (unnormalized); see
 *  checkRoomBoundaryGeometry.cc for why a fallback is sometimes needed. */
Eigen::Vector3d newellNormal(const std::vector<Eigen::Vector3d> &corners_in);

/*! @brief Cross product of the first non-collinear consecutive corner
 *  triple, used when newellNormal() degenerates to zero. */
Eigen::Vector3d
    planeNormalFallback(const std::vector<Eigen::Vector3d> &corners_in);

/*! @brief Signed 2-D orientation of the ordered triple (p, q, r). */
double orientation2d(const Eigen::Vector2d &p_in,
                     const Eigen::Vector2d &q_in,
                     const Eigen::Vector2d &r_in);

/*! @brief True when r_in, known collinear with segment p_in-q_in, lies on
 *  that segment's closed bounding box. */
bool isOnSegmentBoundingBox(const Eigen::Vector2d &p_in,
                            const Eigen::Vector2d &q_in,
                            const Eigen::Vector2d &r_in);

/*! @brief True when closed segments p1_in-q1_in and p2_in-q2_in intersect
 *  (a proper crossing or any touching, including collinear overlap). */
bool doSegmentsIntersect(const Eigen::Vector2d &p1_in,
                         const Eigen::Vector2d &q1_in,
                         const Eigen::Vector2d &p2_in,
                         const Eigen::Vector2d &q2_in);

/*! @brief Structurally validates \p corners_in as a simple closed polygon:
 *  every corner finite, at least three corners, no zero-length consecutive
 *  edge, and no self-intersecting non-adjacent edge pair in the polygon's
 *  own best-fit plane (Newell's method). Purely geometric; does not read
 *  Room::BoundaryStatus or any observation gap. */
RoomBoundaryGeometryStatus
    checkRoomBoundaryGeometry(const std::vector<Eigen::Vector3d> &corners_in);

/*! @brief Typed validity of \p wallRef_in as boundary support evidence for
 *  \p room_in: VALID when present, WALL-typed, live, in \p room_in's own
 *  map, uniquely resolved to one WallRecord in \p snapshot_in, and
 *  reciprocally owned by \p room_in; INVALID for a proven contradiction
 *  (wrong type, retired, cross-map, ambiguous identity, non-reciprocal);
 *  UNAVAILABLE for an ordinary evidence gap. 2026-09-07 residual
 *  proof-closure repair: replaces a lossy boolean so a known contradiction
 *  is never indistinguishable from merely unavailable evidence. */
RoomBoundaryWallEvidenceStatus
    isValidBoundaryWallEvidence(const RawPlaneRef           &wallRef_in,
                                const RoomRecord            &room_in,
                                const SemanticGraphSnapshot &snapshot_in);

/*! @brief Appends one AX-BOUND-01 Finding for \p room_in. Never PASS in
 *  this slice -- see this function's own Doxygen in
 *  evaluateOneRoomBoundary.cc. */
void evaluateOneRoomBoundary(const RoomRecord            &room_in,
                             const SemanticGraphSnapshot &snapshot_in,
                             std::vector<Finding>        &findings_inout);

/*! @brief True when any entry of \p findings_in has result == \p result_in. */
bool anyFindingIs(const std::vector<Finding> &findings_in,
                  AxiomResult                 result_in);

/*! @brief Appends every entry of \p findings_in whose result == \p result_in
 *  to \p relevantKeys_inout's involvedKeys, without sorting/deduplicating
 *  (the caller does that once at the end). */
void appendKeysFromFindings(const std::vector<Finding> &findings_in,
                            AxiomResult                 result_in,
                            std::vector<EntityKey>     &relevantKeys_inout);

/*! @brief Appends AX-FRAME-01's fixed UNKNOWN placeholder Finding: a single
 *  static snapshot cannot prove frame equivariance -- see
 *  evaluateTransition(). Owned by Phase 2. */
void evaluateAxFrame01(const SemanticGraphSnapshot &snapshot_in,
                       std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-WALL-01 Finding for \p wall_in, proving a single
 *  owner's liveness, unique identity, ROOM variant, declared-map agreement,
 *  and reciprocal wallRefs link -- see evaluateOneWall.cc. */
void evaluateOneWall(const WallRecord            &wall_in,
                     const SemanticGraphSnapshot &snapshot_in,
                     std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-WALL-01 Finding per live WallRecord in every map of
 *  \p snapshot_in. */
void evaluateAxWall01(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout);

/*! @brief Appends AX-WALL-02's fixed UNKNOWN placeholder Finding: no
 *  current schema field records observation-ray/aperture-crossing
 *  evidence. Owned by Phase 4. */
void evaluateAxWall02(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout);

/*! @brief Returns the WallRecord in \p snapshot_in keyed exactly \p key_in,
 *  or nullptr when the map named by \p key_in.mapId is absent or has no
 *  such wall. */
const WallRecord *
    findWallByKeyInSnapshot(const SemanticGraphSnapshot &snapshot_in,
                            const EntityKey             &key_in);

/*! @brief Appends one AX-WALL-03 Finding for \p wall_in. */
void evaluateOneWallTwin(const WallRecord            &wall_in,
                         const SemanticGraphSnapshot &snapshot_in,
                         std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-WALL-03 Finding per live WallRecord in every map of
 *  \p snapshot_in. */
void evaluateAxWall03(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-PASS-01 Finding for \p passage_in's own aperture/
 *  skeleton provenance: PASSAGE_PROVENANCE_NOT_PASSABLE/FAIL when not
 *  passable, otherwise PASSAGE_PROVENANCE_FULL_CHAIN_UNVERIFIABLE/UNKNOWN.
 *  Shared by evaluateAxPass01() and computeConservativeMapCompleteness() so
 *  completeness consumes the identical leaf rather than re-deriving its own
 *  passable() check. 2026-09-07 second proof-closure repair (extracted from
 *  evaluateAxPass01.cc). */
void evaluateOnePassageProvenance(const PassageRecord  &passage_in,
                                  std::vector<Finding> &findings_inout);

/*! @brief Appends one AX-PASS-01 Finding per live PassageRecord, via
 *  evaluateOnePassageProvenance(). */
void evaluateAxPass01(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-PASS-02 Finding per live room in \p mapSnapshot_in
 *  that has at least one passageRefs entry carrying a local id with no key
 *  at all (PASSAGE_CARDINALITY_ROOM_HAS_MALFORMED_REFERENCE/FAIL), scoped
 *  to the room alone. Never attributes such a malformed reference to a
 *  specific PassageRecord merely because a bare local id happens to equal
 *  that passage's own entityId -- local ids are unique only within one map
 *  and are not by themselves a map-qualified identity (see EntityKey.h).
 *  2026-09-07 second proof-closure repair. */
void evaluateRoomMalformedPassageReferences(
    const MapSnapshot    &mapSnapshot_in,
    std::vector<Finding> &findings_inout);

/*! @brief Appends one AX-PASS-02 Finding per live PassageRecord. */
void evaluateAxPass02(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-PASS-03 Finding per live PassageRecord. */
void evaluateAxPass03(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-PASS-04 Finding per live PassageRecord. */
void evaluateAxPass04(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-ROOM-01 Finding for \p room_in: always UNKNOWN/
 *  ROOM_CREATION_PROVENANCE_UNAVAILABLE in this slice, since
 *  RoomRecord::creationProvenanceReason is always
 *  NOT_TRACKED_BY_CURRENT_SCHEMA (owned by Phase 3). Shared by
 *  evaluateAxRoom01() and computeConservativeMapCompleteness() so
 *  completeness consumes the identical leaf rather than re-deriving its own
 *  per-room loop, mirroring evaluateOnePassageProvenance(). Checkpoint-A
 *  residual repair. */
void evaluateOneRoomCreationProvenance(const RoomRecord     &room_in,
                                       std::vector<Finding> &findings_inout);

/*! @brief Appends one AX-ROOM-01 Finding per live, confirmed (ROOM-variant)
 *  RoomRecord in every map of \p snapshot_in, via
 *  evaluateOneRoomCreationProvenance(). Owned by Phase 3. */
void evaluateAxRoom01(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout);

/*! @brief Appends AX-ROOM-02's fixed UNKNOWN placeholder Finding: no
 *  current schema field records independent far-side promotion evidence.
 *  Owned by Phase 3. */
void evaluateAxRoom02(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-BOUND-01 Finding per live, RoomRecord::variant ==
 *  ROOM RoomRecord in every map of \p snapshot_in. */
void evaluateAxBound01(const SemanticGraphSnapshot &snapshot_in,
                       std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-FLOOR-01 Finding per live, confirmed RoomRecord
 *  (room-floor reciprocity) plus one per live PassageRecord (endpoint floor
 *  agreement). */
void evaluateAxFloor01(const SemanticGraphSnapshot &snapshot_in,
                       std::vector<Finding>        &findings_inout);

/*! @brief Appends AX-LIFE-01's fixed UNKNOWN placeholder Finding: no
 *  current schema field records quarantine provenance, and detecting
 *  silent erasure additionally requires transition history. Owned by
 *  Phase 4/5. */
void evaluateAxLife01(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout);

/*! @brief Appends AX-TXN-01's fixed UNKNOWN placeholder Finding: a single
 *  static snapshot cannot prove transaction determinism/idempotence -- see
 *  evaluateTransition(). Owned by Phase 7. */
void evaluateAxTxn01(const SemanticGraphSnapshot &snapshot_in,
                     std::vector<Finding>        &findings_inout);

/*! @brief Appends one AX-COMP-01 Finding per entry of \p completeness_in
 *  (or exactly one COMPLETENESS_NO_MAP_PRESENT Finding when it is empty). */
void evaluateAxComp01(const std::vector<MapCompletenessResult> &completeness_in,
                      std::vector<Finding>                     &findings_inout);

/*! @brief Appends AX-MERGE-01's fixed UNKNOWN placeholder Finding: no
 *  map-merge preservation/postcondition logic is implemented in this
 *  slice. Owned by Phase 8. */
void evaluateAxMerge01(const SemanticGraphSnapshot &snapshot_in,
                       std::vector<Finding>        &findings_inout);

/*! @brief Exactly reproduces SemanticsManager::Run()'s current per-map
 *  completeness calculation for \p mapSnapshot_in -- see
 *  LegacyMapCompletenessResult.h. Takes the whole \p snapshot_in, not only
 *  \p mapSnapshot_in, because the legacy code dereferences a passage's
 *  known-side/prospective room pointers directly with no map check at all;
 *  reproducing that exactly requires resolving those references regardless
 *  of which map they land in. */
LegacyMapCompletenessResult
    computeLegacyMapCompleteness(const SemanticGraphSnapshot &snapshot_in,
                                 const MapSnapshot           &mapSnapshot_in);

/*! @brief Computes the conservative semantic-completeness result for
 *  \p mapSnapshot_in (every field of MapCompletenessResult except \c legacy
 *  and \c legacyAndConservativeDiverge, which evaluateMapCompleteness.cc
 *  fills in afterward). Does not call evaluateState(): it independently
 *  re-derives the minimal hard-contradiction signals it needs from
 *  \p snapshot_in using the same shared helpers the per-axiom evaluators
 *  use, to avoid a circular dependency (evaluateState() itself calls
 *  evaluateMapCompleteness() to build its AX-COMP-01 Finding). */
MapCompletenessResult
    computeConservativeMapCompleteness(const SemanticGraphSnapshot &snapshot_in,
                                       const MapSnapshot &mapSnapshot_in);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#include "Semantic/SemanticAxiomEvaluator/private_functions/findRecordByKey.tpp"

#endif // SEMANTIC_AXIOM_EVALUATOR_PRIVATE_FUNCTIONS_H
