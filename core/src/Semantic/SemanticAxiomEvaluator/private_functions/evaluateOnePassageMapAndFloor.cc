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
 * @file            evaluateOnePassageMapAndFloor.cc
 *
 * @brief           Implements evaluateOnePassageMapAndFloor(), declared in
 *                  private_functions.h.
 *
 *                  The terminal success path also appends a typed
 *                  PASSAGE_FLOOR_ENDPOINT_PROOF_UNVERIFIED UNKNOWN
 *                  alongside the clause-level
 *                  PASSAGE_FLOOR_AGREEMENT_VALID PASS, so the AX-PASS-04
 *                  aggregate can never become PASS while
 *                  PassageRecord::endpointSlotReason remains
 *                  NOT_TRACKED_BY_CURRENT_SCHEMA.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateOnePassageMapAndFloor(const PassageRecord         &passage_in,
                                   const SemanticGraphSnapshot &snapshot_in,
                                   std::vector<Finding>        &findings_inout)
{
    const long unsigned int expectedMapId =
        passage_in.declaredMapId.value_or(passage_in.key.mapId);
    const ResolvedRoomEndpoint knownSide =
        resolveRoomEndpoint(passage_in.knownSideRoomRef,
                            expectedMapId,
                            snapshot_in);
    const ResolvedRoomEndpoint prospective =
        resolveRoomEndpoint(passage_in.prospectiveRoomRef,
                            expectedMapId,
                            snapshot_in);

    std::vector<EntityKey> involvedKeys{passage_in.key};
    if (knownSide.key.has_value())
    {
        involvedKeys.push_back(*knownSide.key);
    }
    if (prospective.key.has_value())
    {
        involvedKeys.push_back(*prospective.key);
    }

    if (isKnownInvalidPassageEndpointReference(knownSide) ||
        isKnownInvalidPassageEndpointReference(prospective))
    {
        /* A provably invalid forward reference (wrong kind, unresolvable,
         * duplicate identity, cross-map, declared-map mismatch, or
         * known-bad liveness) must dominate this clause too, not only
         * AX-PASS-02's own cardinality check. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_FLOOR_FORWARD_ENDPOINT_INVALID,
                        involvedKeys));
        return;
    }

    if ((knownSide.referencePresent && knownSide.isCrossMap) ||
        (prospective.referencePresent && prospective.isCrossMap))
    {
        /* No longer gated on
         * isRealPassageEndpoint -- mirrors AX-PASS-02's own
         * unconditional-on-referencePresent cross-map check, since an
         * unenumerated or unconfirmed cross-map reference is just as much a
         * known contradiction as a confirmed one. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_FLOOR_ENDPOINT_CROSS_MAP,
                        involvedKeys));
        return;
    }

    switch (evaluatePassageFloorAgreement(knownSide, prospective, snapshot_in))
    {
    case PassageFloorAgreement::DISAGREE:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_FLOOR_DISAGREEMENT,
                        involvedKeys));
        return;
    case PassageFloorAgreement::AMBIGUOUS:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_FLOOR_IDENTITY_AMBIGUOUS,
                        involvedKeys));
        return;
    case PassageFloorAgreement::ENDPOINT_ROOM_FLOOR_INVALID:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_INVALID,
                        involvedKeys));
        return;
    case PassageFloorAgreement::EVIDENCE_UNAVAILABLE:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::UNKNOWN,
                        ReasonCode::PASSAGE_FLOOR_EVIDENCE_UNAVAILABLE,
                        involvedKeys));
        return;
    case PassageFloorAgreement::ENDPOINT_ROOM_FLOOR_UNVERIFIED:
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_PASS_04,
            AxiomResult::UNKNOWN,
            ReasonCode::PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_UNVERIFIED,
            involvedKeys));
        return;
    case PassageFloorAgreement::NOT_APPLICABLE:
    case PassageFloorAgreement::AGREE:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::PASS,
                        ReasonCode::PASSAGE_FLOOR_AGREEMENT_VALID,
                        involvedKeys));
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::UNKNOWN,
                        ReasonCode::PASSAGE_FLOOR_ENDPOINT_PROOF_UNVERIFIED,
                        involvedKeys));
        return;
    }
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
