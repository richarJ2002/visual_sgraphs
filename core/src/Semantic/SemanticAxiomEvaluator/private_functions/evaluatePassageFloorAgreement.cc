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
 * @file            evaluatePassageFloorAgreement.cc
 *
 * @brief           Implements evaluatePassageFloorAgreement(), declared in
 *                  private_functions.h.
 *
 *                  2026-09-07 proof-correctness repair: an equal floorKey on
 *                  both sides is no longer sufficient for AGREE. Both
 *                  endpoints' floor keys are additionally resolved against
 *                  \p snapshot_in's own FloorRecord collections; an equal
 *                  key that names no actual FloorRecord (a dangling key) is
 *                  EVIDENCE_UNAVAILABLE, not AGREE -- "equal dangling floor
 *                  keys are not proof of a live reciprocal floor." Floor has
 *                  no isBad()/liveness concept at all (see EntityRef.h), so
 *                  existence in this snapshot's own floor records is the
 *                  strongest available proof; canonical reciprocity itself
 *                  is AX-FLOOR-01's own room-floor check, not re-verified
 *                  here.
 *
 *                  2026-09-07 residual proof-closure repair: an equal
 *                  floorKey naming more than one distinct FloorRecord in the
 *                  same map, or a resolved FloorRecord whose own roomRefs
 *                  does not reciprocally list one or both endpoint rooms, is
 *                  now AMBIGUOUS -- a known identity/reciprocity
 *                  contradiction, distinct from a plain missing floor link
 *                  (EVIDENCE_UNAVAILABLE). "Equal floor keys alone are
 *                  insufficient: the floor identity must be unique and
 *                  consistent, and the floor must reciprocally list each
 *                  endpoint room."
 *
 *                  2026-09-07 second proof-closure repair: each real
 *                  endpoint room's own canonical
 *                  evaluateOneRoomFloorReciprocity() result is now consulted
 *                  first; a FAIL there (wrong kind, cross-map, duplicate
 *                  identity, duplicate or missing reverse membership, or a
 *                  second claiming floor) yields ENDPOINT_ROOM_FLOOR_INVALID
 *                  before any floorKey-equality comparison is attempted, so
 *                  equal dangling keys or one malformed reverse member on
 *                  either endpoint can no longer become AGREE.
 *
 *                  Checkpoint-A residual repair: the former
 *                  hasFailingRoomFloorReciprocity() helper (already its own
 *                  translation unit) is replaced by
 *                  canonicalRoomFloorResultFor(), which returns the full
 *                  aggregate AxiomResult instead of a lossy bool, so a
 *                  canonical UNKNOWN (not only FAIL) is now propagated via
 *                  the new ENDPOINT_ROOM_FLOOR_UNVERIFIED outcome.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

PassageFloorAgreement
    evaluatePassageFloorAgreement(const ResolvedRoomEndpoint  &knownSide_in,
                                  const ResolvedRoomEndpoint  &prospective_in,
                                  const SemanticGraphSnapshot &snapshot_in)
{
    if (!isRealPassageEndpoint(knownSide_in) ||
        !isRealPassageEndpoint(prospective_in))
    {
        /* Zero or one real endpoint cannot disagree with itself. */
        return PassageFloorAgreement::NOT_APPLICABLE;
    }

    const AxiomResult knownSideRoomFloorResult =
        canonicalRoomFloorResultFor(knownSide_in, snapshot_in);
    const AxiomResult prospectiveRoomFloorResult =
        canonicalRoomFloorResultFor(prospective_in, snapshot_in);
    if (knownSideRoomFloorResult == AxiomResult::FAIL ||
        prospectiveRoomFloorResult == AxiomResult::FAIL)
    {
        return PassageFloorAgreement::ENDPOINT_ROOM_FLOOR_INVALID;
    }
    if (knownSideRoomFloorResult == AxiomResult::UNKNOWN ||
        prospectiveRoomFloorResult == AxiomResult::UNKNOWN)
    {
        /* Checkpoint-A residual repair (checkpoint 10): a real endpoint
         * room's own canonical room-floor proof is itself unavailable (no
         * FAIL, but not a clean PASS either) -- this passage's floor
         * agreement cannot be positively proved either, even though it is
         * also not a proven contradiction. Must dominate any subsequent
         * floorKey-equality comparison, mirroring the FAIL case above. */
        return PassageFloorAgreement::ENDPOINT_ROOM_FLOOR_UNVERIFIED;
    }

    if (!knownSide_in.floorKey.has_value() ||
        !prospective_in.floorKey.has_value())
    {
        return PassageFloorAgreement::EVIDENCE_UNAVAILABLE;
    }

    if (*knownSide_in.floorKey != *prospective_in.floorKey)
    {
        return PassageFloorAgreement::DISAGREE;
    }

    if (countFloorRecordsWithKey(snapshot_in, *knownSide_in.floorKey) > 1U)
    {
        return PassageFloorAgreement::AMBIGUOUS;
    }
    if (countMapSnapshotsWithId(snapshot_in, knownSide_in.floorKey->mapId) > 1U)
    {
        /* Checkpoint-A residual repair: which MapSnapshot actually holds
         * this floor is itself ambiguous when its own containing map id is
         * duplicated. */
        return PassageFloorAgreement::AMBIGUOUS;
    }

    const FloorRecord *p_floor = nullptr;
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId != knownSide_in.floorKey->mapId)
        {
            continue;
        }
        p_floor = findRecordByKey(mapSnapshot.floors, *knownSide_in.floorKey);
        break;
    }
    if (p_floor == nullptr)
    {
        return PassageFloorAgreement::EVIDENCE_UNAVAILABLE;
    }

    bool knownSideReciprocal       = false;
    bool prospectiveSideReciprocal = false;
    for (const EntityRef &memberRef : p_floor->roomRefs)
    {
        if (!memberRef.key.has_value())
        {
            continue;
        }
        if (*memberRef.key == *knownSide_in.key)
        {
            knownSideReciprocal = true;
        }
        if (*memberRef.key == *prospective_in.key)
        {
            prospectiveSideReciprocal = true;
        }
    }
    if (!knownSideReciprocal || !prospectiveSideReciprocal)
    {
        return PassageFloorAgreement::AMBIGUOUS;
    }

    return PassageFloorAgreement::AGREE;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
