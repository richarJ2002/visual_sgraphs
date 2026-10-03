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
 * @file            ReasonCode.h
 *
 * @brief           Declares every stable reason code a Finding can carry.
 *                  Grouped by the axiom code that uses it; a code is never
 *                  reused across axioms so a bare ReasonCode is already
 *                  unambiguous evidence of which axiom produced a Finding.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_REASON_CODE_H
#define SEMANTIC_AXIOM_EVALUATOR_REASON_CODE_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Stable, prose-free reason identifying why one Finding holds
 *                  its AxiomResult. Part of a Finding's deterministic identity
 *                  (see Finding.h); never derived from wall time, pointer
 *                  addresses, or unordered iteration.
 */
enum class ReasonCode : std::uint8_t
{
    /* AX-FRAME-01 */
    /*!
     * @brief           evaluateState() cannot prove frame equivariance from one
     *                  static snapshot; only evaluateTransition() can.
     */
    FRAME_TRANSITION_EVALUATION_REQUIRED = 0U,
    /*!
     * @brief           evaluateTransition() does not yet implement
     *                  frame-equivariance detection.
     */
    FRAME_EQUIVARIANCE_NOT_YET_IMPLEMENTED = 1U,

    /* AX-WALL-01 */
    /*!
     * @brief           The wall has zero owners; committed-violation proof
     *                  requires quarantine state this schema does not track.
     */
    WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE = 2U,
    /*!
     * @brief           Exactly one owner, live, in the wall's own map.
     */
    WALL_OWNERSHIP_SINGLE_VALID_OWNER = 3U,
    /*!
     * @brief           More than one owner room.
     */
    WALL_OWNERSHIP_MULTIPLE_OWNERS = 4U,
    /*!
     * @brief           An owner reference resolves to a bad (retired) room.
     */
    WALL_OWNERSHIP_OWNER_BAD = 5U,
    /*!
     * @brief           An owner reference could not be resolved to a key at
     *                  all.
     */
    WALL_OWNERSHIP_OWNER_UNRESOLVABLE = 6U,
    /*!
     * @brief           An owner is in a different map than the wall.
     */
    WALL_OWNERSHIP_OWNER_CROSS_MAP = 7U,

    /* AX-WALL-02 */
    /*!
     * @brief           No current schema field records individual
     *                  observation rays or aperture-crossing order.
     */
    WALL_OBSERVATION_RAY_EVIDENCE_UNAVAILABLE = 8U,

    /* AX-WALL-03 */
    /*!
     * @brief           No twin identified; the ordinary case, not a violation.
     */
    WALL_TWIN_ABSENT = 9U,
    /*!
     * @brief           A wall's twin reference names itself.
     */
    WALL_TWIN_SELF = 10U,
    /*!
     * @brief           The twin link is not symmetric (twin's own twin is not
     *                  this wall).
     */
    WALL_TWIN_ASYMMETRIC = 11U,
    /*!
     * @brief           The twin resolves to a bad (retired) plane.
     */
    WALL_TWIN_BAD = 12U,
    /*!
     * @brief           The twin is in a different map.
     */
    WALL_TWIN_CROSS_MAP = 13U,
    /*!
     * @brief           The twin's own planeType is not WALL.
     */
    WALL_TWIN_WRONG_TYPE = 14U,
    /*!
     * @brief           The twin is present but has no map, so
     *                  same-map/cross-map cannot be verified.
     */
    WALL_TWIN_MAP_UNAVAILABLE = 15U,
    /*!
     * @brief           The twin shares a live owner room with this wall, which
     *                  the axiom forbids.
     */
    WALL_TWIN_SHARED_OWNER_FORBIDDEN = 16U,
    /*!
     * @brief           The twin is symmetric, same-map, distinct-owner, and
     *                  structurally valid; full geometric-plausibility
     *                  threshold evidence is not tracked by this schema.
     */
    WALL_TWIN_STRUCTURALLY_VALID_GEOMETRY_UNVERIFIED = 17U,

    /* AX-PASS-01 */
    /*!
     * @brief           A live (published) passage is not marked passable, so it
     *                  lacks the skeleton-crossing evidence the axiom requires.
     */
    PASSAGE_PROVENANCE_NOT_PASSABLE = 18U,
    /*!
     * @brief           passable() is a derived boolean, not a retained
     *                  chain-of-custody provenance record; full provenance
     *                  is unverifiable in this schema.
     */
    PASSAGE_PROVENANCE_FULL_CHAIN_UNVERIFIABLE = 19U,

    /* AX-PASS-02 */
    /*!
     * @brief           Neither slot resolves to a live, confirmed
     *                  (ROOM-variant) room.
     */
    PASSAGE_CARDINALITY_NO_CONFIRMED_ENDPOINT = 20U,
    /*!
     * @brief           Both slots resolve to the same room.
     */
    PASSAGE_CARDINALITY_DUPLICATE_ENDPOINT = 21U,
    /*!
     * @brief           A slot resolves to a bad (retired) room.
     */
    PASSAGE_CARDINALITY_ENDPOINT_BAD = 22U,
    /*!
     * @brief           A slot's reference could not be resolved to a key at
     *                  all.
     */
    PASSAGE_CARDINALITY_ENDPOINT_UNRESOLVABLE = 23U,
    /*!
     * @brief           A slot resolves to a room in a different map.
     */
    PASSAGE_CARDINALITY_ENDPOINT_CROSS_MAP = 24U,
    /*!
     * @brief           A live, confirmed, same-map endpoint room's own
     *                  passageRefs does not name this passage back.
     */
    PASSAGE_CARDINALITY_NON_RECIPROCAL = 25U,
    /*!
     * @brief           At least one distinct live confirmed reciprocal
     *                  endpoint, no
     *                  duplicate/bad/unresolvable/cross-map/non-reciprocal
     *                  reference.
     */
    PASSAGE_CARDINALITY_VALID = 26U,

    /* AX-PASS-03 */
    /*!
     * @brief           The known/near side resolves to a live room whose
     *                  variant is still UNDEFINED (not actually confirmed),
     *                  contradicting "known".
     */
    PASSAGE_SLOT_KNOWN_SIDE_NOT_CONFIRMED = 27U,
    /*!
     * @brief           The known/prospective slot combination matches the axiom
     *                  (one confirmed with at most one prospective handle, or
     *                  two confirmed with neither prospective).
     */
    PASSAGE_SLOT_STATE_VALID = 28U,

    /* AX-PASS-04 */
    /*!
     * @brief           A real endpoint room is in a different map than the
     *                  passage.
     */
    PASSAGE_FLOOR_ENDPOINT_CROSS_MAP = 29U,
    /*!
     * @brief           Two real endpoints resolve to different floors.
     */
    PASSAGE_FLOOR_DISAGREEMENT = 30U,
    /*!
     * @brief           A real endpoint has no floor link yet, so floor
     *                  agreement cannot be verified.
     */
    PASSAGE_FLOOR_EVIDENCE_UNAVAILABLE = 31U,
    /*!
     * @brief           Every real endpoint is in the passage's map and, when
     *                  two exist, they share one floor.
     */
    PASSAGE_FLOOR_AGREEMENT_VALID = 32U,

    /* AX-ROOM-01 */
    /*!
     * @brief           No current schema field records room creation
     *                  provenance (bootstrap-vs-passage-far-side, origin
     *                  passage/side).
     */
    ROOM_CREATION_PROVENANCE_UNAVAILABLE = 33U,

    /* AX-ROOM-02 */
    /*!
     * @brief           No current schema field records independent
     *                  far-side free-space partitioning/observation-owned
     *                  wall support; validated traversal counts alone are
     *                  documented as never sufficient.
     */
    ROOM_FAR_SIDE_EVIDENCE_UNAVAILABLE = 34U,

    /* AX-BOUND-01 */
    /*!
     * @brief           A COMPLETE room has fewer than three boundary corners.
     */
    ROOM_BOUNDARY_TOO_FEW_CORNERS = 35U,
    /*!
     * @brief           A COMPLETE room's boundary has a zero-length
     *                  (degenerate) edge.
     */
    ROOM_BOUNDARY_DEGENERATE_EDGE = 36U,
    /*!
     * @brief           A COMPLETE room's boundary polygon self-intersects.
     */
    ROOM_BOUNDARY_SELF_INTERSECTING = 37U,
    /*!
     * @brief           A COMPLETE room has no wall evidence at all.
     */
    ROOM_BOUNDARY_NO_WALL_EVIDENCE = 38U,
    /*!
     * @brief           The model itself reports
     *                  Room::BoundaryStatus::CONFLICTING.
     */
    ROOM_BOUNDARY_CONFLICTING_STATE = 39U,
    /*!
     * @brief           A COMPLETE, structurally valid boundary retains
     *                  observation gaps whose correspondence to a real
     *                  aperture is not verified.
     */
    ROOM_BOUNDARY_GAP_CORRESPONDENCE_UNVERIFIED = 40U,
    /*!
     * @brief           boundaryStatus is UNOBSERVED or INCOMPLETE; the axiom's
     *                  COMPLETE-only contract is not yet triggered for this
     *                  room.
     */
    ROOM_BOUNDARY_NOT_YET_COMPLETE = 41U,
    /*!
     * @brief           A COMPLETE room has a simple, closed,
     *                  finite-wall-evidenced boundary with no observation gaps.
     */
    ROOM_BOUNDARY_STRUCTURALLY_VALID = 42U,

    /* AX-FLOOR-01 */
    /*!
     * @brief           A live, confirmed room has no floor link yet.
     */
    ROOM_FLOOR_UNLINKED = 43U,
    /*!
     * @brief           The room's floor does not list the room back.
     */
    ROOM_FLOOR_NON_RECIPROCAL = 44U,
    /*!
     * @brief           The room's floor is in a different map than the room.
     */
    ROOM_FLOOR_CROSS_MAP = 45U,
    /*!
     * @brief           The room has exactly one same-map, reciprocal, live
     *                  floor link.
     */
    ROOM_FLOOR_RECIPROCAL_VALID = 46U,
    /*!
     * @brief           Two real passage endpoints resolve to different floors
     *                  (shared logic with PASSAGE_FLOOR_DISAGREEMENT, reported
     *                  under this axiom code too because AX-FLOOR-01's own
     *                  contract names it).
     */
    FLOOR_PASSAGE_CROSS_FLOOR = 47U,
    /*!
     * @brief           A real passage endpoint has no floor link yet.
     */
    FLOOR_PASSAGE_EVIDENCE_UNAVAILABLE = 48U,
    /*!
     * @brief           Every real passage endpoint agrees on floor.
     */
    FLOOR_PASSAGE_AGREEMENT_VALID = 49U,

    /* AX-LIFE-01 */
    /*!
     * @brief           No current schema field records quarantine
     *                  provenance; detecting silent erasure additionally
     *                  requires transition history.
     */
    LIFECYCLE_QUARANTINE_PROVENANCE_UNAVAILABLE = 50U,

    /* AX-TXN-01 */
    /*!
     * @brief           evaluateState() cannot prove transaction determinism/
     *                  idempotence from one static snapshot.
     */
    TRANSACTION_EVALUATION_REQUIRES_TRANSITION = 51U,
    /*!
     * @brief           evaluateTransition() does not yet implement
     *                  postcondition re-validation.
     */
    TRANSACTION_POSTCONDITION_NOT_YET_IMPLEMENTED = 52U,

    /* AX-COMP-01 */
    /*!
     * @brief           The map has zero confirmed (live, ROOM-variant) rooms.
     */
    COMPLETENESS_ZERO_CONFIRMED_ROOMS = 53U,
    /*!
     * @brief           The map has at least one live prospective
     *                  (UNDEFINED-variant) room.
     */
    COMPLETENESS_LIVE_PROSPECTIVE_ROOM_PRESENT = 54U,
    /*!
     * @brief           At least one confirmed room is not boundary-COMPLETE.
     */
    COMPLETENESS_ROOM_BOUNDARY_NOT_COMPLETE = 55U,
    /*!
     * @brief           At least one live passage lacks two distinct, live,
     *                  confirmed, same-map, same-floor, reciprocal, COMPLETE
     *                  endpoints.
     */
    COMPLETENESS_PASSAGE_ENDPOINTS_INVALID = 56U,
    /*!
     * @brief           Another observable hard violation (AX-WALL-01/02/03,
     *                  AX-PASS-*, AX-ROOM-*) was found in this map.
     */
    COMPLETENESS_HARD_CONTRADICTION = 57U,
    /*!
     * @brief           No contradiction was found, but required proof for at
     *                  least one clause is unavailable.
     */
    COMPLETENESS_EVIDENCE_UNAVAILABLE = 58U,
    /*!
     * @brief           The snapshot contains no map at all.
     */
    COMPLETENESS_NO_MAP_PRESENT = 59U,
    /*!
     * @brief           At least one confirmed room, all confirmed rooms
     *                  COMPLETE, no live prospectives, every live passage fully
     *                  valid, no contradiction and no unavailable evidence
     *                  found.
     */
    COMPLETENESS_ALL_CLEAR = 60U,

    /* AX-MERGE-01 */
    /*!
     * @brief           No map-merge preservation/postcondition logic is
     *                  implemented.
     */
    MERGE_PRESERVATION_NOT_YET_IMPLEMENTED = 61U,

    /* AX-PASS-02 */
    /*!
     * @brief           A third live, confirmed, same-map room -- distinct from
     *                  both forward-named slots -- lists this passage in its
     *                  own passageRefs.
     */
    PASSAGE_CARDINALITY_THIRD_ENDPOINT = 62U,
    /*!
     * @brief           A room not named by either forward slot lists this
     *                  passage back, but that room is itself retired.
     */
    PASSAGE_CARDINALITY_REVERSE_ENDPOINT_BAD = 63U,
    /*!
     * @brief           A room not named by either forward slot lists this
     *                  passage back, but that room is in a different map than
     *                  the passage.
     */
    PASSAGE_CARDINALITY_REVERSE_ENDPOINT_CROSS_MAP = 64U,
    /*!
     * @brief           A live room in this map has at least one passageRefs
     *                  entry that carries a local id with no key at all (the
     *                  underlying pointer had no map at capture time): a
     *                  malformed reference this schema cannot safely resolve.
     *                  Scoped to the room alone -- never attributed to a
     *                  specific passage merely because a bare integer happens
     *                  to equal that passage's own local id, since local ids
     *                  are unique only within one map and are not by themselves
     *                  a map-qualified identity (see EntityKey.h). Renamed in
     *                  place from
     *                  PASSAGE_CARDINALITY_REVERSE_ENDPOINT_UNRESOLVABLE, which
     *                  incorrectly attributed this same evidence to one
     *                  specific passage by bare local-id equality; the numeric
     *                  value is unchanged.
     */
    PASSAGE_CARDINALITY_ROOM_HAS_MALFORMED_REFERENCE = 65U,
    /*!
     * @brief           More than one RoomRecord in the same map shares the
     *                  exact key of a room this passage's cardinality
     *                  evaluation depends on.
     */
    PASSAGE_CARDINALITY_DUPLICATE_ROOM_IDENTITY = 66U,
    /*!
     * @brief           At least one real, reciprocal, non-contradictory
     *                  endpoint was found and no third/bad/cross-map/
     *                  unresolvable/duplicate reverse reference was found
     *                  either, but PassageRecord::endpointSlotReason is
     *                  NOT_TRACKED_BY_CURRENT_SCHEMA: no authoritative
     *                  DISCOVERY_SIDE/OPPOSITE_SIDE slot proof exists, so
     *                  cardinality remains UNKNOWN rather than PASS.
     */
    PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED = 67U,

    /* AX-WALL-01 */
    /*!
     * @brief           The single owner reference is present and not explicitly
     *                  bad, but EntityRef::isLive itself carries no value
     *                  (livenessUnavailable Reason != NONE): liveness is
     *                  genuinely unproven, not merely unchecked.
     */
    WALL_OWNERSHIP_OWNER_LIVENESS_UNAVAILABLE = 68U,
    /*!
     * @brief           The owner resolves to a located RoomRecord whose variant
     *                  is not ROOM (a prospective handle cannot own a committed
     *                  wall).
     */
    WALL_OWNERSHIP_OWNER_WRONG_VARIANT = 69U,
    /*!
     * @brief           The owner resolves to a located RoomRecord whose own
     *                  declaredMapId disagrees with the containing map used to
     *                  key it.
     */
    WALL_OWNERSHIP_OWNER_DECLARED_MAP_MISMATCH = 70U,
    /*!
     * @brief           The owner resolves to a located, live, ROOM-variant,
     *                  same-map RoomRecord, but that room's own wallRefs does
     *                  not resolve back to this wall.
     */
    WALL_OWNERSHIP_OWNER_NOT_RECIPROCAL = 71U,
    /*!
     * @brief           More than one RoomRecord in the owner's map shares the
     *                  exact key the owner reference names, so which room
     *                  actually owns this wall is ambiguous.
     */
    WALL_OWNERSHIP_OWNER_DUPLICATE_IDENTITY = 72U,
    /*!
     * @brief           The owner reference is present, live, and same-map, but
     *                  no RoomRecord with that exact key was located in any
     *                  captured map (see the analogous missing-enumeration note
     *                  in resolveRoomEndpoint.cc): genuinely missing
     *                  enumeration evidence, not a proven contradiction.
     */
    WALL_OWNERSHIP_OWNER_RECORD_UNAVAILABLE = 73U,

    /* AX-BOUND-01 */
    /*!
     * @brief           A COMPLETE room's boundary polygon has a non-finite (NaN
     *                  or Infinity) corner coordinate.
     */
    ROOM_BOUNDARY_NON_FINITE_CORNER = 74U,
    /*!
     * @brief           A COMPLETE, structurally valid boundary has at
     *                  least one verified live, same-map, reciprocal WALL
     *                  evidence reference, but full edge-to-wall geometric
     *                  correspondence is not implemented.
     */
    ROOM_BOUNDARY_EDGE_SUPPORT_UNVERIFIED = 75U,

    /* AX-FLOOR-01 */
    /*!
     * @brief           A floor's own roomRefs lists the same room key more than
     *                  once.
     */
    ROOM_FLOOR_DUPLICATE_REVERSE_MEMBERSHIP = 76U,
    /*!
     * @brief           More than one distinct floor in the room's map lists
     *                  this room in its roomRefs.
     */
    ROOM_FLOOR_CLAIMED_BY_MULTIPLE_FLOORS = 77U,
    /*!
     * @brief           More than one FloorRecord in the room's map shares the
     *                  exact key the room's floorRef names.
     */
    ROOM_FLOOR_DUPLICATE_IDENTITY = 78U,

    /* AX-COMP-01 */
    /*!
     * @brief           The map has at least one live passage;
     *                  PassageRecord::endpointSlotReason is always
     *                  NOT_TRACKED_BY_CURRENT_SCHEMA, so no live passage
     *                  can contribute positive completeness proof
     *                  regardless of how plausible its endpoints look.
     */
    COMPLETENESS_PASSAGE_SLOT_PROOF_UNAVAILABLE = 79U,

    /* AX-PASS-03/04, AX-FLOOR-01: a non-contradictory passage still
     * cannot positively PASS its own axiom-specific clause while
     * PassageRecord::endpointSlotReason remains NOT_TRACKED_BY_CURRENT_SCHEMA,
     * so each of these three evaluators also emits one of these typed UNKNOWNs
     * alongside any clause-level PASS, capping the aggregate at UNKNOWN via
     * FAIL > UNKNOWN > PASS precedence. */
    /*!
     * @brief           AX-PASS-03's own clause held, but no authoritative
     *                  endpoint slot proof exists for this passage.
     */
    PASSAGE_SLOT_ENDPOINT_PROOF_UNVERIFIED = 80U,
    /*!
     * @brief           AX-PASS-04's own map/floor clause held, but no
     *                  authoritative endpoint slot proof exists for this
     *                  passage.
     */
    PASSAGE_FLOOR_ENDPOINT_PROOF_UNVERIFIED = 81U,
    /*!
     * @brief           AX-FLOOR-01's passage-floor-identity clause held, but no
     *                  authoritative endpoint slot proof exists for this
     *                  passage.
     */
    FLOOR_PASSAGE_ENDPOINT_PROOF_UNVERIFIED = 82U,

    /* AX-PASS-02 */
    /*!
     * @brief           A same-map, live room's passageRefs entry shares this
     *                  passage's map/entity id but a different EntityKind: a
     *                  wrong-kind key masquerading as a reference to this
     *                  passage.
     */
    PASSAGE_CARDINALITY_REVERSE_ENDPOINT_WRONG_KIND = 83U,
    /*!
     * @brief           One live, same-map, ROOM-variant room's own passageRefs
     *                  names this passage more than once: relationship
     *                  multiplicity that key deduplication must not silently
     *                  erase to a single clean reference.
     */
    PASSAGE_CARDINALITY_REVERSE_REFERENCE_DUPLICATED = 84U,
    /*!
     * @brief           A forward (known-side or prospective) endpoint reference
     *                  resolves to a key that more than one distinct RoomRecord
     *                  in the same map shares: which room actually forms the
     *                  endpoint is ambiguous, so it can never supply positive
     *                  cardinality proof.
     */
    PASSAGE_CARDINALITY_FORWARD_ENDPOINT_DUPLICATE_IDENTITY = 85U,

    /* AX-WALL-01 */
    /*!
     * @brief           More than one WallRecord in the wall's own map shares
     *                  its exact key: which wall this evaluation is even about
     *                  is ambiguous.
     */
    WALL_OWNERSHIP_WALL_DUPLICATE_IDENTITY = 86U,
    /*!
     * @brief           The wall's own declaredMapId disagrees with the
     *                  containing map used to enumerate and key it.
     */
    WALL_OWNERSHIP_WALL_DECLARED_MAP_MISMATCH = 87U,
    /*!
     * @brief           The owner resolves to a located RoomRecord whose own
     *                  isLive is false, even though the owning reference's own
     *                  captured liveness reported it as live -- a
     *                  record/reference disagreement.
     */
    WALL_OWNERSHIP_OWNER_RECORD_NOT_LIVE = 88U,

    /* AX-BOUND-01 */
    /*!
     * @brief           At least one RoomRecord::wallRefs entry is provably
     *                  invalid (wrong type, retired, cross-map, ambiguous
     *                  identity, or non-reciprocal) rather than merely
     *                  unavailable; a known contradiction that must not be
     *                  hidden behind another valid reference or the
     *                  edge-support-coverage UNKNOWN.
     */
    ROOM_BOUNDARY_INVALID_WALL_EVIDENCE = 89U,

    /* AX-FLOOR-01 */
    /*!
     * @brief           A floor's roomRefs reverse-claims a room that itself has
     *                  no forward floorRef naming any floor at all: a known
     *                  contradiction, not the ordinary "room has no floor yet"
     *                  case.
     */
    ROOM_FLOOR_REVERSE_CLAIM_WITHOUT_FORWARD_LINK = 90U,
    /*!
     * @brief           The room's own floorRef key is not FLOOR-typed.
     */
    ROOM_FLOOR_WRONG_KIND = 91U,
    /*!
     * @brief           The named floor's own declaredMapId disagrees with its
     *                  containing map.
     */
    ROOM_FLOOR_DECLARED_MAP_MISMATCH = 92U,
    /*!
     * @brief           Two real passage endpoints share an equal floor key, but
     *                  that key names more than one FloorRecord, or the
     *                  resolved floor does not reciprocally list one or both
     *                  endpoint rooms: identity/reciprocity ambiguity distinct
     *                  from a plain missing-evidence or cross-floor
     *                  disagreement.
     */
    PASSAGE_FLOOR_IDENTITY_AMBIGUOUS = 93U,
    /*!
     * @brief           AX-FLOOR-01's own reporting of PASSAGE_FLOOR_IDENTITY_
     *                  AMBIGUOUS's underlying condition.
     */
    FLOOR_PASSAGE_IDENTITY_AMBIGUOUS = 94U,

    /* AX-COMP-01 */
    /*!
     * @brief           The map has at least one confirmed (live,
     *                  ROOM-variant) room;
     *                  RoomRecord::creationProvenanceReason is always
     *                  NOT_TRACKED_BY_CURRENT_SCHEMA, so
     *                  room-creation/bootstrap provenance can never
     *                  positively contribute to completeness.
     */
    COMPLETENESS_ROOM_CREATION_PROVENANCE_UNAVAILABLE = 95U,
    /*!
     * @brief           At least one room, wall, passage, or floor key in this
     *                  map is shared by more than one distinct record: identity
     *                  itself is ambiguous, so no first-match proof anywhere in
     *                  this map can be trusted.
     */
    COMPLETENESS_DUPLICATE_IDENTITY = 96U,

    /* AX-PASS-02 */
    /*!
     * @brief           A forward (known-side or prospective) endpoint
     *                  reference's key has an EntityKind other than ROOM: a
     *                  wrong-kind key masquerading as a room reference.
     */
    PASSAGE_CARDINALITY_ENDPOINT_WRONG_KIND = 97U,
    /*!
     * @brief           A forward endpoint reference resolves to a located
     *                  RoomRecord whose own declaredMapId disagrees with the
     *                  map it was found in.
     */
    PASSAGE_CARDINALITY_ENDPOINT_DECLARED_MAP_MISMATCH = 98U,
    /*!
     * @brief           A forward endpoint reference is present and carries no
     *                  other known contradiction, but its own EntityRef::isLive
     *                  carries no value at all: liveness is genuinely unproven,
     *                  not merely unchecked, so this passage's cardinality
     *                  cannot be certified either way. Distinct from
     *                  PASSAGE_CARDINALITY_ENDPOINT_BAD, which requires an
     *                  explicit known-false liveness value.
     */
    PASSAGE_CARDINALITY_ENDPOINT_LIVENESS_UNAVAILABLE = 99U,
    /*!
     * @brief           At least one same-map room's passageRefs entry names
     *                  this passage by key with no independently-known
     *                  kind/map/identity contradiction, but that entry's own
     *                  EntityRef::isLive carries no value at all:
     *                  reverse-endpoint liveness is genuinely unproven, so this
     *                  passage's cardinality cannot be certified either way.
     */
    PASSAGE_CARDINALITY_REVERSE_ENDPOINT_LIVENESS_UNAVAILABLE = 100U,

    /* AX-PASS-03 */
    /*!
     * @brief           A forward endpoint reference is provably invalid (wrong
     *                  kind, unresolvable, duplicate identity, cross-map,
     *                  declared-map mismatch, or known-bad liveness) -- must
     *                  dominate this clause's ordinary
     *                  PASS/endpoint-proof-UNKNOWN pair rather than being
     *                  silently ignored because it does not resolve to a
     *                  confirmed room at all.
     */
    PASSAGE_SLOT_FORWARD_ENDPOINT_INVALID = 101U,

    /* AX-PASS-04 */
    /*!
     * @brief           Same contradiction as
     *                  PASSAGE_SLOT_FORWARD_ENDPOINT_INVALID, reported under
     *                  AX-PASS-04's own map/floor-agreement clause.
     */
    PASSAGE_FLOOR_FORWARD_ENDPOINT_INVALID = 102U,
    /*!
     * @brief           At least one real endpoint room's own canonical
     *                  evaluateOneRoomFloorReciprocity() result is FAIL
     *                  (PassageFloorAgreement::ENDPOINT_ROOM_FLOOR_INVALID):
     *                  the passage's floor agreement cannot be valid while
     *                  either endpoint room's own room/floor relationship is
     *                  itself contradictory.
     */
    PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_INVALID = 103U,

    /* AX-FLOOR-01 */
    /*!
     * @brief           Same contradiction as
     *                  PASSAGE_SLOT_FORWARD_ENDPOINT_INVALID, reported under
     *                  AX-FLOOR-01's passage-floor-identity clause.
     */
    FLOOR_PASSAGE_FORWARD_ENDPOINT_INVALID = 104U,
    /*!
     * @brief           AX-FLOOR-01's own reporting of
     *                  PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_INVALID's underlying
     *                  condition.
     */
    FLOOR_PASSAGE_ENDPOINT_ROOM_FLOOR_INVALID = 105U,

    /* AX-WALL-01 */
    /*!
     * @brief           The wall's own key.kind is not EntityKind::WALL.
     */
    WALL_OWNERSHIP_WALL_WRONG_KEY_KIND = 106U,
    /*!
     * @brief           The wall's own planeType is not
     *                  Plane::PlaneVariant::WALL.
     */
    WALL_OWNERSHIP_WALL_WRONG_PLANE_TYPE = 107U,
    /*!
     * @brief           The single owner reference's own key.kind is not
     *                  EntityKind::ROOM.
     */
    WALL_OWNERSHIP_OWNER_WRONG_KEY_KIND = 108U,
    /*!
     * @brief           Every other AX-WALL-01 clause is affirmatively
     *                  satisfied, but the wall's own declaredMapId is absent:
     *                  positive ownership proof is capped at UNKNOWN rather
     *                  than PASS, since declared-map agreement could not
     *                  actually be verified either way.
     */
    WALL_OWNERSHIP_WALL_DECLARED_MAP_UNAVAILABLE = 109U,
    /*!
     * @brief           The resolved owner RoomRecord's own wallRefs contains at
     *                  least one entry that shares this wall's raw
     *                  mapId/planeId identity but is otherwise malformed (wrong
     *                  reason, wrong planeType, not live, or a wallKey
     *                  inconsistent with that same mapId/planeId): a known
     *                  contradiction, even when another entry in the same
     *                  wallRefs collection is a well-formed reciprocal
     *                  reference.
     */
    WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY = 110U,
    /*!
     * @brief           The resolved owner RoomRecord's own wallRefs contains
     *                  more than one well-formed reciprocal reference to this
     *                  wall: which reference is the "real" reciprocal link is
     *                  ambiguous.
     */
    WALL_OWNERSHIP_RECIPROCAL_DUPLICATE = 111U,

    /* AX-BOUND-01 */
    /*!
     * @brief           A COMPLETE room's wallRefs collection is nonempty, but
     *                  every entry is
     *                  RoomBoundaryWallEvidenceStatus::UNAVAILABLE (no entry is
     *                  independently known INVALID): an ordinary evidence gap,
     *                  not the ROOM_BOUNDARY_NO_WALL_EVIDENCE contradiction
     *                  reserved for a genuinely empty wallRefs collection.
     */
    ROOM_BOUNDARY_WALL_EVIDENCE_UNAVAILABLE = 112U,

    /* AX-COMP-01 */
    /*!
     * @brief           More than one MapSnapshot in the evaluated snapshot
     *                  shares this map's own mapId: which MapSnapshot is
     *                  authoritative for this map is itself ambiguous, so no
     *                  first-matching-map lookup anywhere in this map's
     *                  completeness evaluation can be trusted.
     */
    COMPLETENESS_DUPLICATE_MAP_IDENTITY = 113U,

    /* AX-FLOOR-01 */
    /*!
     * @brief           The room's own key.kind is not EntityKind::ROOM.
     */
    ROOM_FLOOR_ROOM_WRONG_KIND = 114U,
    /*!
     * @brief           The room's own declaredMapId disagrees with the
     *                  containing map used to enumerate and key it.
     */
    ROOM_FLOOR_ROOM_DECLARED_MAP_MISMATCH = 115U,
    /*!
     * @brief           Every other AX-FLOOR-01 room-floor clause is
     *                  affirmatively satisfied, but the room's own
     *                  declaredMapId is absent: positive reciprocity proof is
     *                  capped at UNKNOWN rather than PASS.
     */
    ROOM_FLOOR_ROOM_DECLARED_MAP_UNAVAILABLE = 116U,

    /* AX-COMP-01 */
    /*!
     * @brief           At least one live room in this map has a passageRefs
     *                  entry the evaluator cannot safely resolve (a local id
     *                  with no key):
     *                  PASSAGE_CARDINALITY_ROOM_HAS_MALFORMED_REFERENCE.
     */
    COMPLETENESS_ROOM_HAS_MALFORMED_PASSAGE_REFERENCE = 117U,

    /* AX-PASS-02 */
    /*!
     * @brief           A forward endpoint's source EntityRef violates its own
     *                  key/reason invariant (key.has_value() with reason !=
     *                  NONE): an adversarial/corrupt snapshot value, not an
     *                  ordinary valid keyed reference.
     */
    PASSAGE_CARDINALITY_ENDPOINT_REASON_INCONSISTENT = 118U,

    /* AX-PASS-03 */
    /*!
     * @brief           A real (found, live, confirmed-variant) forward endpoint
     *                  resolves in a different map than this passage's own
     *                  declared/ containing map.
     */
    PASSAGE_SLOT_ENDPOINT_CROSS_MAP = 119U,

    /* AX-FLOOR-01 */
    /*!
     * @brief           The passage branch's real forward endpoint resolves in a
     *                  different map than this passage's own
     *                  declared/containing map.
     */
    FLOOR_PASSAGE_ENDPOINT_CROSS_MAP = 120U,
    /*!
     * @brief           The uniquely resolved named floor's own declaredMapId is
     *                  unavailable: every other room-floor clause is
     *                  affirmatively satisfied, but this room's positive proof
     *                  is capped at UNKNOWN rather than PASS.
     */
    ROOM_FLOOR_FLOOR_DECLARED_MAP_UNAVAILABLE = 121U,

    /* AX-FLOOR-01 */
    /*!
     * @brief           This room's own containing map id is duplicated across
     *                  more than one MapSnapshot: no first-match FloorRecord
     *                  lookup may supply positive proof.
     */
    ROOM_FLOOR_CONTAINING_MAP_AMBIGUOUS = 122U,
    /*!
     * @brief           The forward floorRef's own EntityRef::reason is not NONE
     *                  despite key.has_value(): an invariant violation.
     */
    ROOM_FLOOR_ROOM_REASON_INCONSISTENT = 123U,
    /*!
     * @brief           A reverse member naming this room has its own reason not
     *                  NONE or known-false liveness: a known contradiction, not
     *                  reciprocity proof.
     */
    ROOM_FLOOR_REVERSE_MEMBER_INVALID = 124U,
    /*!
     * @brief           Every reverse member naming this room has genuinely
     *                  unproven liveness: reciprocity is neither confirmed nor
     *                  contradicted.
     */
    ROOM_FLOOR_REVERSE_MEMBER_LIVENESS_UNAVAILABLE = 125U,

    /* AX-PASS-02 */
    /*!
     * @brief           A forward endpoint's own containing map id is duplicated
     *                  across more than one MapSnapshot.
     */
    PASSAGE_CARDINALITY_ENDPOINT_CONTAINING_MAP_AMBIGUOUS = 126U,

    /* AX-WALL-01 */
    /*!
     * @brief           The single owner's own EntityRef::reason is not NONE
     *                  despite key.has_value(): an invariant violation.
     */
    WALL_OWNERSHIP_OWNER_REASON_INCONSISTENT = 127U,
    /*!
     * @brief           The owner's own containing map id is duplicated across
     *                  more than one MapSnapshot: no first-match RoomRecord
     *                  lookup may supply positive proof.
     */
    WALL_OWNERSHIP_OWNER_CONTAINING_MAP_AMBIGUOUS = 128U,
    /*!
     * @brief           The owner room's own declaredMapId is genuinely absent:
     *                  every other ownership clause is affirmatively satisfied,
     *                  but positive proof is capped at UNKNOWN rather than
     *                  PASS.
     */
    WALL_OWNERSHIP_OWNER_DECLARED_MAP_UNAVAILABLE = 129U,

    /* AX-WALL-03 */
    /*!
     * @brief           The twin reference's own reason claims "absent" but
     *                  carries populated data (mapId/wallKey/a real planeType):
     *                  an invariant violation and a known contradiction, not an
     *                  ordinary absent twin.
     */
    WALL_TWIN_REASON_INCONSISTENT = 130U,
    /*!
     * @brief           The named twin key resolves to more than one distinct
     *                  WallRecord: ambiguous identity.
     */
    WALL_TWIN_DUPLICATE_IDENTITY = 131U,
    /*!
     * @brief           The twin's own containing map id is duplicated across
     *                  more than one MapSnapshot.
     */
    WALL_TWIN_CONTAINING_MAP_AMBIGUOUS = 132U,

    /* AX-PASS-04 */
    /*!
     * @brief           At least one real endpoint room's own canonical
     *                  room-floor result is UNKNOWN (no FAIL, but not a clean
     *                  PASS either).
     */
    PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_UNVERIFIED = 133U,

    /* AX-FLOOR-01 */
    /*!
     * @brief           At least one real endpoint room's own canonical
     *                  room-floor result is UNKNOWN (no FAIL, but not a clean
     *                  PASS either), for the passage branch.
     */
    FLOOR_PASSAGE_ENDPOINT_ROOM_FLOOR_UNVERIFIED = 134U
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_REASON_CODE_H
