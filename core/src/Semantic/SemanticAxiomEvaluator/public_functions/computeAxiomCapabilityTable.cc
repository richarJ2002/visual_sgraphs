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
 * @file         computeAxiomCapabilityTable.cc
 *
 * @brief        Implements computeAxiomCapabilityTable(), declared
 *               in public_functions.h.
 *
 *               Fixed capability/ownership assignment:
 *               - AX-FRAME-01: DEFERRED (frame/face provenance; a
 *                 static snapshot cannot prove equivariance at
 *                 all);
 *               - AX-WALL-01: PARTIAL (multiple/invalid/bad/
 *                 cross-map owners are fully checkable; a
 *                 committed zero-owner violation requires
 *                 quarantine proof this schema does not track);
 *               - AX-WALL-02: DEFERRED (no observation-ray/
 *                 aperture-crossing evidence is tracked at all);
 *               - AX-WALL-03: PARTIAL (structural twin plausibility
 *                 is fully checkable; geometric-plausibility
 *                 threshold evidence is not tracked);
 *               - AX-PASS-01: PARTIAL (the passable() flag makes a
 *                 non-passable live passage an observable FAIL;
 *                 full chain-of-custody provenance is not
 *                 retained);
 *               - AX-PASS-02/03/04: PARTIAL (PassageRecord::
 *                 endpointSlotReason is always
 *                 NOT_TRACKED_BY_CURRENT_SCHEMA -- no authoritative
 *                 DISCOVERY_SIDE/OPPOSITE_SIDE slot exists, so
 *                 observable contradictions -- including a third
 *                 reverse-listing room, a bad/cross-map/
 *                 unresolvable/duplicate reverse reference,
 *                 cardinality, reciprocity, known-side variant
 *                 confirmation, and map/floor agreement -- are
 *                 fully checkable and FAIL when contradicted, but
 *                 a passage with no contradiction is UNKNOWN,
 *                 never PASS, pending authoritative endpoint
 *                 slots);
 *               - AX-ROOM-01/02: DEFERRED (no schema field records
 *                 creation provenance or independent far-side
 *                 promotion evidence at all);
 *               - AX-BOUND-01: PARTIAL (boundary-status,
 *                 polygon-geometry, non-finite-corner, and verified
 *                 live/reciprocal/same-map wall-evidence clauses
 *                 are fully checkable and FAIL when contradicted;
 *                 full edge-to-wall and observation-gap-to-aperture
 *                 geometric correspondence is not implemented, so
 *                 a COMPLETE room with no contradiction is UNKNOWN,
 *                 never PASS);
 *               - AX-FLOOR-01: PARTIAL (room-floor reciprocity, including
 *                 duplicate-identity and multi-floor-claim
 *                 detection, is fully checkable; the passage
 *                 floor-agreement clause shares AX-PASS-04's
 *                 authoritative-endpoint-slot gap, so it is PARTIAL
 *                 overall);
 *               - AX-LIFE-01: DEFERRED (no quarantine provenance
 *                 field exists, and detecting silent erasure
 *                 additionally requires transition history);
 *               - AX-TXN-01: DEFERRED (a static snapshot cannot prove
 *                 transaction determinism/idempotence, and
 *                 postcondition re-validation is not implemented);
 *               - AX-COMP-01: PARTIAL (the shadow conservative
 *                 calculation itself is fully implemented here; the
 *                 missing piece is becoming the production
 *                 authority in place of the legacy calculation, an
 *                 integration step not performed here -- see the
 *                 shadow-only scope);
 *               - AX-MERGE-01: DEFERRED (no map-merge
 *                 preservation/postcondition logic is implemented).
 */

#include "Semantic/SemanticAxiomEvaluator/public_functions.h"

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::vector<AxiomCapabilityEntry> computeAxiomCapabilityTable()
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
} // namespace core
} // namespace vs_graphs
