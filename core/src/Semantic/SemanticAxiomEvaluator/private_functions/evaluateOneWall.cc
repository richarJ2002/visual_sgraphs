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
 * @file            evaluateOneWall.cc
 *
 * @brief           Implements evaluateOneWall(), declared in
 *                  private_functions.h.
 *
 *                  2026-09-07 proof-correctness repair: a single owner
 *                  reference is no longer accepted merely for being keyed,
 *                  same-map, and not explicitly bad. Full positive proof now
 *                  additionally requires: owner liveness explicitly
 *                  available (not merely absent-and-assumed-fine); a unique
 *                  matching live RoomRecord (no duplicate same-key
 *                  ambiguity); that RoomRecord's own variant is ROOM (a
 *                  prospective handle cannot own a committed wall); that
 *                  RoomRecord's own declaredMapId agreeing with the wall's
 *                  containing map; and that RoomRecord's own wallRefs
 *                  resolving back to this wall (reciprocity). Missing
 *                  liveness or a genuinely unenumerable owner record is
 *                  UNKNOWN; every other new check is a proven contradiction
 *                  (FAIL).
 *
 *                  2026-09-07 residual proof-closure repair: completes the
 *                  wall-record -> owner-ref -> live room-record ->
 *                  reciprocal wall-ref chain. Adds: this WallRecord's own
 *                  duplicate-identity check (which wall this evaluation is
 *                  even about must itself be unambiguous); the wall's own
 *                  declaredMapId consistency with its containing map; the
 *                  resolved owner RoomRecord's own isLive (independent of
 *                  the owning reference's own captured liveness, catching a
 *                  record/reference disagreement no earlier check
 *                  observes); and the reciprocal wallRefs entry's own
 *                  isLive (not merely its wallKey identity). The owner
 *                  key's kind is always EntityKind::ROOM by construction
 *                  (WallRecord::ownerRoomRefs is built exclusively from
 *                  captureSemanticGraphSnapshot.cc's wallOwnersByPointer
 *                  inversion pass, which always assigns
 *                  makeKey(EntityKind::ROOM, ...)), the same structural
 *                  exclusion scanReversePassageEndpoints.cc documents for
 *                  RoomRecord::passageRefs -- validated as data anyway
 *                  below (see the 2026-09-07 second proof-closure repair
 *                  paragraph), since public snapshot records are
 *                  deliberately mutable adversarial inputs.
 *
 *                  2026-09-07 second proof-closure repair: adds this wall's
 *                  own key.kind == WALL and planeType == WALL checks; the
 *                  owner reference's own key.kind == ROOM check; caps
 *                  positive proof at UNKNOWN (rather than PASS) when the
 *                  wall's own declaredMapId is genuinely absent, checked
 *                  only after every other clause is affirmatively
 *                  satisfied; and replaces the first-equal-wallKey
 *                  reciprocal scan with a full scan of every wallRefs entry
 *                  sharing this wall's own raw mapId/planeId identity --
 *                  any such entry that is not simultaneously WALL-typed,
 *                  live, and wallKey-consistent is a known contradiction
 *                  (WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY) even beside an
 *                  otherwise well-formed entry, and more than one
 *                  well-formed entry is itself ambiguous
 *                  (WALL_OWNERSHIP_RECIPROCAL_DUPLICATE).
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <cstddef>
#include <utility>

namespace ORB_SLAM3
{
namespace semantic
{

void evaluateOneWall(const WallRecord            &wall_in,
                     const SemanticGraphSnapshot &snapshot_in,
                     std::vector<Finding>        &findings_inout)
{
    if (wall_in.key.kind != EntityKind::WALL)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_WALL_WRONG_KEY_KIND,
                        {wall_in.key}));
        return;
    }

    if (wall_in.planeType != Plane::planeVariant::WALL)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_WALL_WRONG_PLANE_TYPE,
                        {wall_in.key}));
        return;
    }

    if (countWallRecordsWithKey(snapshot_in, wall_in.key) > 1U)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_WALL_DUPLICATE_IDENTITY,
                        {wall_in.key}));
        return;
    }

    if (wall_in.declaredMapId.has_value() &&
        *wall_in.declaredMapId != wall_in.key.mapId)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_WALL_DECLARED_MAP_MISMATCH,
                        {wall_in.key}));
        return;
    }

    const std::size_t ownerCount = wall_in.ownerRoomRefs.size();

    if (ownerCount == 0U)
    {
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_WALL_01,
            AxiomResult::UNKNOWN,
            ReasonCode::WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE,
            {wall_in.key}));
        return;
    }

    if (ownerCount > 1U)
    {
        std::vector<EntityKey> involvedKeys{wall_in.key};
        for (const EntityRef &owner : wall_in.ownerRoomRefs)
        {
            if (owner.key.has_value())
            {
                involvedKeys.push_back(*owner.key);
            }
        }
        FindingEvidence evidence;
        evidence.observedCount = ownerCount;
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_MULTIPLE_OWNERS,
                        std::move(involvedKeys),
                        evidence));
        return;
    }

    const EntityRef &owner = wall_in.ownerRoomRefs.front();
    if (!owner.key.has_value())
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_UNRESOLVABLE,
                        {wall_in.key}));
        return;
    }

    if (owner.key->kind != EntityKind::ROOM)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_WRONG_KEY_KIND,
                        {wall_in.key, *owner.key}));
        return;
    }

    if (owner.reason != UnavailableReason::NONE)
    {
        /* Checkpoint-A residual repair: EntityRef documents
         * key.has_value() <=> reason == NONE as an invariant; a keyed
         * owner reference whose own reason is not NONE is a known
         * contradiction, not an ordinary valid reference. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_REASON_INCONSISTENT,
                        {wall_in.key, *owner.key}));
        return;
    }

    if (owner.key->mapId != wall_in.key.mapId)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_CROSS_MAP,
                        {wall_in.key, *owner.key}));
        return;
    }

    if (countRoomRecordsWithKey(snapshot_in, *owner.key) > 1U)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_DUPLICATE_IDENTITY,
                        {wall_in.key, *owner.key}));
        return;
    }

    if (countMapSnapshotsWithId(snapshot_in, owner.key->mapId) > 1U)
    {
        /* Checkpoint-A residual repair: which MapSnapshot actually holds
         * the owner room is itself ambiguous when its own containing map id
         * is duplicated -- no first-match RoomRecord lookup below may
         * supply positive proof. */
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_WALL_01,
            AxiomResult::FAIL,
            ReasonCode::WALL_OWNERSHIP_OWNER_CONTAINING_MAP_AMBIGUOUS,
            {wall_in.key, *owner.key}));
        return;
    }

    if (owner.isLive.has_value() && !(*owner.isLive))
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_BAD,
                        {wall_in.key, *owner.key}));
        return;
    }

    const RoomRecord *p_owner = nullptr;
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId != owner.key->mapId)
        {
            continue;
        }
        p_owner = findRecordByKey(mapSnapshot.rooms, *owner.key);
        break;
    }
    if (p_owner == nullptr)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_UNAVAILABLE,
                        {wall_in.key, *owner.key}));
        return;
    }

    if (!p_owner->isLive)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_NOT_LIVE,
                        {wall_in.key, *owner.key}));
        return;
    }

    if (p_owner->variant != Room::roomVariant::ROOM)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_WRONG_VARIANT,
                        {wall_in.key, *owner.key}));
        return;
    }

    if (p_owner->declaredMapId.has_value() &&
        *p_owner->declaredMapId != wall_in.key.mapId)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_MISMATCH,
                        {wall_in.key, *owner.key}));
        return;
    }
    /* Checkpoint-A residual repair (D1): a genuinely absent
     * p_owner->declaredMapId is missing evidence, not a proven mismatch --
     * the UNKNOWN cap for it is deferred to after the PASS below (see
     * wall_in.declaredMapId's own analogous cap), preserving FAIL >
     * UNKNOWN precedence against the reciprocal checks still to come. */

    /* P1.R10 fourth re-audit fix: the source EntityRef's own liveness being
     * genuinely unproven is deferred to a cap appended after the terminal
     * PASS below (see wall_in.declaredMapId's own analogous cap), not an
     * early UNKNOWN return here. The reciprocal-scan FAILs immediately below
     * depend only on p_owner->wallRefs (already resolved above) and
     * wall_in.key -- never on owner.isLive -- so an early return here would
     * mask them too, one step further than the P1.R10 third re-audit fix
     * (declaredMapId mismatch) already closed for the checks above it. */

    /* 2026-09-07 second proof-closure repair: every wallRefs entry that
     * shares this wall's own raw mapId/planeId identity is inspected, not
     * only the first equal-wallKey entry -- a well-formed reciprocal
     * reference has reason == NONE (implied by wallKey.has_value(), see
     * RawPlaneRef.h's invariant), WALL type, live state, and a wallKey that
     * agrees with that same mapId/planeId. Any entry sharing this wall's
     * raw identity but failing that full check is a known contradiction
     * (WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY), dominating even when
     * another entry is well-formed; more than one well-formed entry is
     * itself ambiguous (WALL_OWNERSHIP_RECIPROCAL_DUPLICATE). */
    std::size_t wellFormedReciprocalCount  = 0U;
    bool        anyContradictoryReciprocal = false;
    for (const RawPlaneRef &ownedWallRef : p_owner->wallRefs)
    {
        /* P1.R10 second re-audit fix: relevance is no longer keyed on raw
         * mapId/planeId alone -- an entry whose own wallKey already names
         * this exact wall is about this wall too, even when its raw
         * mapId/planeId field has been adversarially mutated to disagree
         * with that same wallKey (an internal RawPlaneRef inconsistency,
         * not evidence of an unrelated wall). Checking wallKey alone would
         * conversely miss a raw-identity match whose wallKey was never
         * populated (e.g. reason != NONE); either signal alone is enough to
         * pull the entry into this loop's scope. */
        const bool aboutThisWallByRaw =
            ownedWallRef.mapId.has_value() &&
            (*ownedWallRef.mapId == wall_in.key.mapId) &&
            (ownedWallRef.planeId == wall_in.key.entityId);
        const bool aboutThisWallByKey = ownedWallRef.wallKey.has_value() &&
                                        (*ownedWallRef.wallKey == wall_in.key);
        if (!aboutThisWallByRaw && !aboutThisWallByKey)
        {
            continue;
        }
        if (ownedWallRef.reason != UnavailableReason::NONE)
        {
            /* Checkpoint-A residual repair (checkpoint 6): this entry's own
             * mapId/planeId match this wall, but its own reason claims
             * "absent" (RawPlaneRef::reason != NONE) -- an invariant
             * violation and a known contradiction, not an ordinary unrelated
             * reference to skip. Populated data with a non-NONE reason must
             * never be silently masked. */
            anyContradictoryReciprocal = true;
            continue;
        }
        if (aboutThisWallByRaw != aboutThisWallByKey)
        {
            /* Exactly one of the two identity signals names this wall: the
             * entry's own wallKey and its own raw mapId/planeId disagree
             * about which wall it references -- an internal RawPlaneRef
             * inconsistency, a known contradiction. */
            anyContradictoryReciprocal = true;
            continue;
        }
        const bool wellFormed =
            (ownedWallRef.planeType == Plane::planeVariant::WALL) &&
            ownedWallRef.isLive && ownedWallRef.wallKey.has_value() &&
            (*ownedWallRef.wallKey == wall_in.key);
        if (wellFormed)
        {
            ++wellFormedReciprocalCount;
        }
        else
        {
            anyContradictoryReciprocal = true;
        }
    }
    if (anyContradictoryReciprocal)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY,
                        {wall_in.key, *owner.key}));
        return;
    }
    if (wellFormedReciprocalCount > 1U)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_DUPLICATE,
                        {wall_in.key, *owner.key}));
        return;
    }
    if (wellFormedReciprocalCount == 0U)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_NOT_RECIPROCAL,
                        {wall_in.key, *owner.key}));
        return;
    }

    findings_inout.push_back(
        makeFinding(AxiomCode::AX_WALL_01,
                    AxiomResult::PASS,
                    ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER,
                    {wall_in.key, *owner.key}));
    if (!wall_in.declaredMapId.has_value())
    {
        /* Every other clause is affirmatively satisfied, but the wall's own
         * declared map is genuinely absent evidence, not a contradiction:
         * cap positive ownership proof at UNKNOWN rather than PASS. */
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_WALL_01,
            AxiomResult::UNKNOWN,
            ReasonCode::WALL_OWNERSHIP_WALL_DECLARED_MAP_UNAVAILABLE,
            {wall_in.key, *owner.key}));
    }
    if (!p_owner->declaredMapId.has_value())
    {
        /* Checkpoint-A residual repair (D1): the owner room's own declared
         * map is genuinely absent evidence, not a contradiction -- cap
         * positive ownership proof at UNKNOWN rather than PASS, mirroring
         * the wall's own missing-declared-map cap immediately above. */
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_WALL_01,
            AxiomResult::UNKNOWN,
            ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_UNAVAILABLE,
            {wall_in.key, *owner.key}));
    }
    if (!owner.isLive.has_value())
    {
        /* P1.R10 fourth re-audit fix: every other clause -- including every
         * record-level FAIL and the full reciprocal-scan FAIL set above --
         * is affirmatively satisfied, but the source reference's own
         * liveness was never itself proven either way: cap positive
         * ownership proof at UNKNOWN rather than PASS, mirroring the
         * wall's/owner's own missing-declared-map caps immediately above. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::WALL_OWNERSHIP_OWNER_LIVENESS_UNAVAILABLE,
                        {wall_in.key, *owner.key}));
    }
}

} // namespace semantic
} // namespace ORB_SLAM3
