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
 * @file            resolveRoomEndpoint.cc
 *
 * @brief           Implements resolveRoomEndpoint(), declared in
 *                  private_functions.h.
 *
 *                  "Real"/"confirmed" throughout this module's passage-
 *                  endpoint evaluators (AX-PASS-02/03/04) means
 *                  isLive && isConfirmedRoomVariant, read directly from the
 *                  target RoomRecord this function resolves. This is a
 *                  deliberate, documented proxy for the plan's
 *                  "authoritative side slot" concept
 *                  (PassageRecord::endpointSlotReason is always
 *                  UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA in this
 *                  slice, confirmed by direct source read of Passage.h/
 *                  Passage.cc: KnownSideProvenance and getProspectiveRoom()
 *                  record which room is known/prospective, not a named
 *                  DISCOVERY_SIDE/OPPOSITE_SIDE endpoint slot). Evaluating
 *                  cardinality, reciprocity, and map/floor agreement from
 *                  this proxy is not "claiming authoritative endpoint-slot
 *                  PASS from unavailable evidence": every field this
 *                  function reads (EntityRef::key/isLive,
 *                  RoomRecord::isLive/variant/floorRef) is genuinely,
 *                  always populated by capture -- see
 *                  ResolvedRoomEndpoint.h.
 *
 *                  Residual known limitation: RoomRecord::variant is not
 *                  duplicated onto EntityRef, so isConfirmedRoomVariant can
 *                  only be known when the target is actually located via
 *                  enumeration (isFoundInSnapshot); a referenced room this
 *                  snapshot cannot enumerate in any captured map never
 *                  satisfies "real"/"confirmed" here, even when it is
 *                  genuinely live -- the downstream evaluators correctly
 *                  treat that as unproven rather than fabricating a
 *                  variant, but it remains a schema gap distinct from the
 *                  isLive fix below.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

ResolvedRoomEndpoint
    resolveRoomEndpoint(const EntityRef             &ref_in,
                        long unsigned int            expectedMapId_in,
                        const SemanticGraphSnapshot &snapshot_in)
{
    ResolvedRoomEndpoint resolved;

    /* EntityRef::isLive is populated directly from the referenced pointer's
     * isBad() whenever the pointer was non-null at capture time,
     * independent of whether that pointer's own key could be formed or
     * whether this snapshot can enumerate it in any captured map's
     * RoomRecord collection (see EntityRef.h and entityRefForRoom.cc) --
     * read it here unconditionally so a keyed-but-unenumerated or
     * unresolvable-but-non-null reference still reports its true liveness
     * rather than silently reading as an ordinary absent reference.
     * isLiveAvailable is recorded independently (2026-09-07 second
     * proof-closure repair) so a caller can distinguish "genuinely unproven"
     * from "known false" instead of treating isLive's own false default as
     * a known-bad fact. */
    if (ref_in.isLive.has_value())
    {
        resolved.isLive          = *ref_in.isLive;
        resolved.isLiveAvailable = true;
    }

    resolved.referencePresent = ref_in.key.has_value();
    if (!resolved.referencePresent)
    {
        /* A non-null underlying pointer with no key formable
         * (UnavailableReason::ENTITY_HAS_NO_MAP) is the only case with
         * localId present but no key; a genuinely absent reference
         * (UnavailableReason::NULL_REFERENCE) leaves both absent. */
        resolved.referenceUnresolvable = ref_in.localId.has_value();
        return resolved;
    }

    resolved.key         = ref_in.key;
    resolved.isCrossMap  = (ref_in.key->mapId != expectedMapId_in);
    resolved.isWrongKind = (ref_in.key->kind != EntityKind::ROOM);
    /* Checkpoint-A residual repair: EntityRef documents
     * key.has_value() <=> reason == NONE as an invariant, but this
     * snapshot's records are adversarial value inputs -- do not assume
     * capture made them coherent. A keyed reference whose own reason is
     * not NONE is a known contradiction. */
    resolved.isReasonInconsistent = (ref_in.reason != UnavailableReason::NONE);

    if (countMapSnapshotsWithId(snapshot_in, ref_in.key->mapId) > 1U)
    {
        /* Checkpoint-A residual repair: which MapSnapshot is actually
         * authoritative for this map id is itself ambiguous, so no
         * first-match lookup below may supply positive proof. */
        resolved.isContainingMapAmbiguous = true;
        return resolved;
    }

    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId != ref_in.key->mapId)
        {
            continue;
        }
        if (countRoomRecordsWithKey(snapshot_in, *ref_in.key) > 1U)
        {
            /* 2026-09-07 residual proof-closure repair: more than one
             * distinct RoomRecord shares this exact key -- which room
             * actually forms the endpoint is ambiguous, so no first-match
             * lookup below may supply positive proof (see
             * isDuplicateIdentity's own Doxygen). */
            resolved.isDuplicateIdentity = true;
            break;
        }
        const RoomRecord *p_foundRoom =
            findRecordByKey(mapSnapshot.rooms, *ref_in.key);
        if (p_foundRoom != nullptr)
        {
            resolved.isFoundInSnapshot = true;
            /* Checkpoint-A residual repair: do NOT fill resolved.isLive/
             * isLiveAvailable from p_foundRoom->isLive here. EntityRef::
             * isLive is already populated directly from the referenced
             * pointer's isBad() whenever the pointer was non-null (see the
             * read at the top of this function) -- under a well-formed
             * invariant, ref_in.isLive and p_foundRoom->isLive always agree.
             * When they do NOT (ref_in.isLive absent, an adversarial value
             * input this evaluator must not assume away), the found
             * record's own liveness must never manufacture availability the
             * reference itself does not carry: unavailable forward-reference
             * liveness is not overwritten by a present/live room record and
             * must not silently become a confirmed endpoint. */
            resolved.isConfirmedRoomVariant =
                (p_foundRoom->variant == Room::roomVariant::ROOM);
            if (p_foundRoom->floorRef.key.has_value())
            {
                resolved.floorKey = p_foundRoom->floorRef.key;
            }
            /* 2026-09-07 second proof-closure repair: the located record's
             * own declared map must agree with the map it was actually
             * enumerated from (this loop only ever searches the map named
             * by ref_in.key->mapId), otherwise the target record itself is
             * internally inconsistent. */
            resolved.isTargetDeclaredMapMismatch =
                p_foundRoom->declaredMapId.has_value() &&
                (*p_foundRoom->declaredMapId != mapSnapshot.mapId);
        }
        break;
    }

    return resolved;
}

} // namespace semantic
} // namespace ORB_SLAM3
