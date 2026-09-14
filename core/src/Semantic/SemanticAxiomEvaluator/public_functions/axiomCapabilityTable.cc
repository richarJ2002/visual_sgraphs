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
 * @file            axiomCapabilityTable.cc
 *
 * @brief           Implements axiomCapabilityTable(), declared in
 *                  public_functions.h.
 *
 *                  Fixed capability/ownership assignment for this slice, per
 *                  semantic-axiom-reliability-plan.md and the evaluator-
 *                  foundation-slice instructions' default ownership table:
 *                  - AX-FRAME-01: DEFERRED, Phase 2 (frame/face provenance;
 *                    a static snapshot cannot prove equivariance at all);
 *                  - AX-WALL-01: PARTIAL, Phase 4 (multiple/invalid/bad/
 *                    cross-map owners are fully checkable; a committed
 *                    zero-owner violation requires quarantine proof this
 *                    schema does not track);
 *                  - AX-WALL-02: DEFERRED, Phase 4 (no observation-ray/
 *                    aperture-crossing evidence is tracked at all);
 *                  - AX-WALL-03: PARTIAL, Phase 4 (structural twin
 *                    plausibility is fully checkable; geometric-
 *                    plausibility threshold evidence is not tracked);
 *                  - AX-PASS-01: PARTIAL, Phase 5 (the passable() flag
 *                    makes a non-passable live passage an observable FAIL;
 *                    full chain-of-custody provenance is not retained);
 *                  - AX-PASS-02/03/04: PARTIAL, Phase 3 (2026-09-07
 *                    proof-correctness repair: PassageRecord::
 *                    endpointSlotReason is always
 *                    NOT_TRACKED_BY_CURRENT_SCHEMA -- no authoritative
 *                    DISCOVERY_SIDE/OPPOSITE_SIDE slot exists, so observable
 *                    contradictions -- including a third reverse-listing
 *                    room, a bad/cross-map/unresolvable/duplicate reverse
 *                    reference, cardinality, reciprocity, known-side
 *                    variant confirmation, and map/floor agreement -- are
 *                    fully checkable and FAIL when contradicted, but a
 *                    passage with no contradiction is UNKNOWN, never PASS,
 *                    pending Phase 3's authoritative endpoint slots);
 *                  - AX-ROOM-01/02: DEFERRED, Phase 3 (no schema field
 *                    records creation provenance or independent far-side
 *                    promotion evidence at all);
 *                  - AX-BOUND-01: PARTIAL, Phase 6 (boundary-status,
 *                    polygon-geometry, non-finite-corner, and verified
 *                    live/reciprocal/same-map wall-evidence clauses are
 *                    fully checkable and FAIL when contradicted; full
 *                    edge-to-wall and observation-gap-to-aperture geometric
 *                    correspondence is Phase 6's planar-arrangement
 *                    algorithm, not implemented in this slice, so a
 *                    COMPLETE room with no contradiction is UNKNOWN, never
 *                    PASS);
 *                  - AX-FLOOR-01: PARTIAL, Phase 3 (2026-09-07
 *                    proof-correctness repair: room-floor reciprocity,
 *                    including duplicate-identity and multi-floor-claim
 *                    detection, is fully checkable; the passage
 *                    floor-agreement clause shares AX-PASS-04's
 *                    authoritative-endpoint-slot gap, so it is PARTIAL
 *                    overall);
 *                  - AX-LIFE-01: DEFERRED, Phase 4/5 (no quarantine
 *                    provenance field exists, and detecting silent erasure
 *                    additionally requires transition history);
 *                  - AX-TXN-01: DEFERRED, Phase 7 (a static snapshot cannot
 *                    prove transaction determinism/idempotence, and
 *                    postcondition re-validation is not implemented);
 *                  - AX-COMP-01: PARTIAL, Phase 7 (this slice fully
 *                    implements the shadow conservative calculation itself;
 *                    the missing piece is becoming the production authority
 *                    in place of the legacy calculation, an integration
 *                    step this slice deliberately does not perform -- see
 *                    P1.3's shadow-only scope);
 *                  - AX-MERGE-01: DEFERRED, Phase 8 (no map-merge
 *                    preservation/postcondition logic is implemented).
 */

#include "Semantic/SemanticAxiomEvaluator/public_functions.h"

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

std::vector<AxiomCapabilityEntry> axiomCapabilityTable()
{
    return {makeAxiomCapabilityEntry(AxiomCode::AX_FRAME_01,
                                     CapabilityLevel::DEFERRED,
                                     MissingProofOwner::PHASE_2),
            makeAxiomCapabilityEntry(AxiomCode::AX_WALL_01,
                                     CapabilityLevel::PARTIAL,
                                     MissingProofOwner::PHASE_4),
            makeAxiomCapabilityEntry(AxiomCode::AX_WALL_02,
                                     CapabilityLevel::DEFERRED,
                                     MissingProofOwner::PHASE_4),
            makeAxiomCapabilityEntry(AxiomCode::AX_WALL_03,
                                     CapabilityLevel::PARTIAL,
                                     MissingProofOwner::PHASE_4),
            makeAxiomCapabilityEntry(AxiomCode::AX_PASS_01,
                                     CapabilityLevel::PARTIAL,
                                     MissingProofOwner::PHASE_5),
            makeAxiomCapabilityEntry(AxiomCode::AX_PASS_02,
                                     CapabilityLevel::PARTIAL,
                                     MissingProofOwner::PHASE_3),
            makeAxiomCapabilityEntry(AxiomCode::AX_PASS_03,
                                     CapabilityLevel::PARTIAL,
                                     MissingProofOwner::PHASE_3),
            makeAxiomCapabilityEntry(AxiomCode::AX_PASS_04,
                                     CapabilityLevel::PARTIAL,
                                     MissingProofOwner::PHASE_3),
            makeAxiomCapabilityEntry(AxiomCode::AX_ROOM_01,
                                     CapabilityLevel::DEFERRED,
                                     MissingProofOwner::PHASE_3),
            makeAxiomCapabilityEntry(AxiomCode::AX_ROOM_02,
                                     CapabilityLevel::DEFERRED,
                                     MissingProofOwner::PHASE_3),
            makeAxiomCapabilityEntry(AxiomCode::AX_BOUND_01,
                                     CapabilityLevel::PARTIAL,
                                     MissingProofOwner::PHASE_6),
            makeAxiomCapabilityEntry(AxiomCode::AX_FLOOR_01,
                                     CapabilityLevel::PARTIAL,
                                     MissingProofOwner::PHASE_3),
            makeAxiomCapabilityEntry(AxiomCode::AX_LIFE_01,
                                     CapabilityLevel::DEFERRED,
                                     MissingProofOwner::PHASE_4_OR_5),
            makeAxiomCapabilityEntry(AxiomCode::AX_TXN_01,
                                     CapabilityLevel::DEFERRED,
                                     MissingProofOwner::PHASE_7),
            makeAxiomCapabilityEntry(AxiomCode::AX_COMP_01,
                                     CapabilityLevel::PARTIAL,
                                     MissingProofOwner::PHASE_7),
            makeAxiomCapabilityEntry(AxiomCode::AX_MERGE_01,
                                     CapabilityLevel::DEFERRED,
                                     MissingProofOwner::PHASE_8)};
}

} // namespace semantic
} // namespace ORB_SLAM3
