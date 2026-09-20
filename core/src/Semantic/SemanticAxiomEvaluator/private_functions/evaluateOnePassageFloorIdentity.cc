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
 * @file            evaluateOnePassageFloorIdentity.cc
 *
 * @brief           Implements evaluateOnePassageFloorIdentity(), declared in
 *                  private_functions.h.
 *
 *                  The terminal success path also appends a typed
 *                  FLOOR_PASSAGE_ENDPOINT_PROOF_UNVERIFIED UNKNOWN
 *                  alongside the clause-level
 *                  FLOOR_PASSAGE_AGREEMENT_VALID PASS, so the passage
 *                  branch of the AX-FLOOR-01 aggregate can never become
 *                  PASS while PassageRecord::endpointSlotReason remains
 *                  NOT_TRACKED_BY_CURRENT_SCHEMA.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateOnePassageFloorIdentity(const PassageRecord         &passage_in,
                                     const SemanticGraphSnapshot &snapshot_in,
                                     std::vector<Finding> &findings_inout)
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
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::FLOOR_PASSAGE_FORWARD_ENDPOINT_INVALID,
                        involvedKeys));
        return;
    }

    if ((knownSide.referencePresent && knownSide.isCrossMap) ||
        (prospective.referencePresent && prospective.isCrossMap))
    {
        /* Mirrors
         * evaluateOnePassageMapAndFloor.cc's own unconditional-on-
         * referencePresent cross-map check (not gated on
         * isRealPassageEndpoint) -- a reference resolving in a different map
         * than this passage's own declared/containing map must fail this
         * leaf too, not only AX-PASS-02/04's. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::FLOOR_PASSAGE_ENDPOINT_CROSS_MAP,
                        involvedKeys));
        return;
    }

    switch (evaluatePassageFloorAgreement(knownSide, prospective, snapshot_in))
    {
    case PassageFloorAgreement::DISAGREE:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::FLOOR_PASSAGE_CROSS_FLOOR,
                        involvedKeys));
        return;
    case PassageFloorAgreement::AMBIGUOUS:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::FLOOR_PASSAGE_IDENTITY_AMBIGUOUS,
                        involvedKeys));
        return;
    case PassageFloorAgreement::ENDPOINT_ROOM_FLOOR_INVALID:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::FAIL,
                        ReasonCode::FLOOR_PASSAGE_ENDPOINT_ROOM_FLOOR_INVALID,
                        involvedKeys));
        return;
    case PassageFloorAgreement::EVIDENCE_UNAVAILABLE:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::FLOOR_PASSAGE_EVIDENCE_UNAVAILABLE,
                        involvedKeys));
        return;
    case PassageFloorAgreement::ENDPOINT_ROOM_FLOOR_UNVERIFIED:
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_FLOOR_01,
            AxiomResult::UNKNOWN,
            ReasonCode::FLOOR_PASSAGE_ENDPOINT_ROOM_FLOOR_UNVERIFIED,
            involvedKeys));
        return;
    case PassageFloorAgreement::NOT_APPLICABLE:
    case PassageFloorAgreement::AGREE:
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::PASS,
                        ReasonCode::FLOOR_PASSAGE_AGREEMENT_VALID,
                        involvedKeys));
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_FLOOR_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::FLOOR_PASSAGE_ENDPOINT_PROOF_UNVERIFIED,
                        involvedKeys));
        return;
    }
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
