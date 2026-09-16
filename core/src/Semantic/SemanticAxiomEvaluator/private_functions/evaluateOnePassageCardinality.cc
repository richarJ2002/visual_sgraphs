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
 * @file            evaluateOnePassageCardinality.cc
 *
 * @brief           Implements evaluateOnePassageCardinality(), declared in
 *                  private_functions.h.
 *
 *                  An exhaustive reverse-endpoint scan
 *                  (scanReversePassageEndpoints()) complements the
 *                  forward-only checks, and the terminal result is UNKNOWN
 *                  (PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED): see
 *                  PassageRecord::endpointSlotReason, always
 *                  NOT_TRACKED_BY_CURRENT_SCHEMA -- no authoritative
 *                  DISCOVERY_SIDE/OPPOSITE_SIDE slot proof exists in this
 *                  schema, so a passage with one or two apparently valid,
 *                  reciprocal, non-contradictory legacy pointers is
 *                  UNKNOWN, never PASS. Every observable contradiction
 *                  (forward or reverse) still dominates that UNKNOWN
 *                  result via FAIL > UNKNOWN precedence.
 *
 *                  Forward endpoint resolution rejects an ambiguous
 *                  duplicate-identity match; the reverse scan's wrong-kind
 *                  and duplicated-reference anomalies are checked; and the
 *                  terminal cardinality rule computes the union of every
 *                  real (forward or reverse) endpoint key before applying
 *                  the maximum-two-endpoint rule, so a reverse-only room
 *                  that merely fills an otherwise-empty forward slot is
 *                  not misreported as a third endpoint -- only a union
 *                  exceeding two distinct keys is.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateOnePassageCardinality(const PassageRecord         &passage_in,
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

    if (knownSide.referenceUnresolvable || prospective.referenceUnresolvable)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_UNRESOLVABLE,
                        involvedKeys));
        return;
    }

    if (knownSide.isWrongKind || prospective.isWrongKind)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_WRONG_KIND,
                        involvedKeys));
        return;
    }

    if (knownSide.isReasonInconsistent || prospective.isReasonInconsistent)
    {
        /* A keyed forward reference whose own
         * EntityRef::reason is not NONE violates EntityRef's documented
         * invariant and must never flow through as an ordinary valid
         * reference. */
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_PASS_02,
            AxiomResult::FAIL,
            ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_REASON_INCONSISTENT,
            involvedKeys));
        return;
    }

    if (knownSide.isContainingMapAmbiguous ||
        prospective.isContainingMapAmbiguous)
    {
        /* A duplicate MapSnapshot::mapId for
         * the referenced map means no first-match lookup can supply
         * positive proof for this endpoint. */
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_PASS_02,
            AxiomResult::FAIL,
            ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_CONTAINING_MAP_AMBIGUOUS,
            involvedKeys));
        return;
    }

    /* isLive is trustworthy only when isLiveAvailable -- a keyed endpoint
     * with genuinely unproven liveness is not a bad endpoint merely because
     * ResolvedRoomEndpoint::isLive defaults to false; see
     * resolveRoomEndpoint.cc's Doxygen and the dedicated
     * liveness-unavailable check below. */
    const bool knownSideBad = knownSide.referencePresent &&
                              knownSide.isLiveAvailable && !knownSide.isLive;
    const bool prospectiveBad = prospective.referencePresent &&
                                prospective.isLiveAvailable &&
                                !prospective.isLive;
    if (knownSideBad || prospectiveBad)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_BAD,
                        involvedKeys));
        return;
    }

    if ((knownSide.isFoundInSnapshot &&
         knownSide.isTargetDeclaredMapMismatch) ||
        (prospective.isFoundInSnapshot &&
         prospective.isTargetDeclaredMapMismatch))
    {
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_PASS_02,
            AxiomResult::FAIL,
            ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_DECLARED_MAP_MISMATCH,
            involvedKeys));
        return;
    }

    if ((knownSide.referencePresent && knownSide.isCrossMap) ||
        (prospective.referencePresent && prospective.isCrossMap))
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_CROSS_MAP,
                        involvedKeys));
        return;
    }

    if (knownSide.referencePresent && prospective.referencePresent &&
        knownSide.key.has_value() && prospective.key.has_value() &&
        (*knownSide.key == *prospective.key))
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_DUPLICATE_ENDPOINT,
                        involvedKeys));
        return;
    }

    const ReversePassageEndpointScan reverseScan =
        scanReversePassageEndpoints(passage_in, expectedMapId, snapshot_in);

    if (!reverseScan.badReverseRoomKeys.empty())
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           reverseScan.badReverseRoomKeys.begin(),
                           reverseScan.badReverseRoomKeys.end());
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_BAD,
                        anomalyKeys));
        return;
    }
    if (!reverseScan.crossMapReverseRoomKeys.empty())
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           reverseScan.crossMapReverseRoomKeys.begin(),
                           reverseScan.crossMapReverseRoomKeys.end());
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_PASS_02,
            AxiomResult::FAIL,
            ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_CROSS_MAP,
            anomalyKeys));
        return;
    }
    if (!reverseScan.duplicateIdentityRoomKeys.empty())
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           reverseScan.duplicateIdentityRoomKeys.begin(),
                           reverseScan.duplicateIdentityRoomKeys.end());
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_DUPLICATE_ROOM_IDENTITY,
                        anomalyKeys));
        return;
    }
    if (!reverseScan.wrongKindReverseRoomKeys.empty())
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           reverseScan.wrongKindReverseRoomKeys.begin(),
                           reverseScan.wrongKindReverseRoomKeys.end());
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_PASS_02,
            AxiomResult::FAIL,
            ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_WRONG_KIND,
            anomalyKeys));
        return;
    }
    if (!reverseScan.duplicateReferenceRoomKeys.empty())
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           reverseScan.duplicateReferenceRoomKeys.begin(),
                           reverseScan.duplicateReferenceRoomKeys.end());
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_PASS_02,
            AxiomResult::FAIL,
            ReasonCode::PASSAGE_CARDINALITY_REVERSE_REFERENCE_DUPLICATED,
            anomalyKeys));
        return;
    }
    if (knownSide.isDuplicateIdentity || prospective.isDuplicateIdentity)
    {
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_PASS_02,
            AxiomResult::FAIL,
            ReasonCode::PASSAGE_CARDINALITY_FORWARD_ENDPOINT_DUPLICATE_IDENTITY,
            involvedKeys));
        return;
    }

    if ((knownSide.referencePresent && !knownSide.isLiveAvailable) ||
        (prospective.referencePresent && !prospective.isLiveAvailable))
    {
        /* A forward endpoint reference is present with no other known
         * contradiction, but its own liveness is genuinely unproven: this
         * passage's cardinality cannot be certified either way. Checked
         * after every reverse-scan FAIL above so an independently known
         * contradiction still dominates (FAIL > UNKNOWN). */
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_PASS_02,
            AxiomResult::UNKNOWN,
            ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_LIVENESS_UNAVAILABLE,
            involvedKeys));
        return;
    }
    if (!reverseScan.livenessUnavailableReverseRoomKeys.empty())
    {
        /* A same-map room's reverse reference to this passage is otherwise
         * clean, but that entry's own liveness is genuinely unproven --
         * "missing liveness is unavailable, not live": this reverse
         * reference must not be silently counted as a confirmed endpoint,
         * but neither may it be treated as a proven third-endpoint
         * contradiction. */
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(
            anomalyKeys.end(),
            reverseScan.livenessUnavailableReverseRoomKeys.begin(),
            reverseScan.livenessUnavailableReverseRoomKeys.end());
        findings_inout.push_back(makeFinding(
            AxiomCode::AX_PASS_02,
            AxiomResult::UNKNOWN,
            ReasonCode::
                PASSAGE_CARDINALITY_REVERSE_ENDPOINT_LIVENESS_UNAVAILABLE,
            anomalyKeys));
        return;
    }

    const bool knownSideReal   = isRealPassageEndpoint(knownSide);
    const bool prospectiveReal = isRealPassageEndpoint(prospective);

    /* Union of every real (forward or reverse) endpoint identity, computed
     * before applying the maximum-two-endpoint rule: a reverse-only room
     * that merely fills an otherwise-empty forward slot must not be
     * misreported as a third endpoint when the true union still contains
     * only two distinct rooms. */
    std::vector<EntityKey> unionKeys;
    if (knownSideReal)
    {
        unionKeys.push_back(*knownSide.key);
    }
    if (prospectiveReal)
    {
        unionKeys.push_back(*prospective.key);
    }
    for (const EntityKey &reverseKey : reverseScan.confirmedReverseRoomKeys)
    {
        if (std::find(unionKeys.begin(), unionKeys.end(), reverseKey) ==
            unionKeys.end())
        {
            unionKeys.push_back(reverseKey);
        }
    }

    if (unionKeys.size() > 2U)
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           unionKeys.begin(),
                           unionKeys.end());
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_THIRD_ENDPOINT,
                        anomalyKeys));
        return;
    }

    if (unionKeys.empty())
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_NO_CONFIRMED_ENDPOINT,
                        involvedKeys));
        return;
    }

    if (knownSideReal &&
        !roomListsPassageBack(snapshot_in, *knownSide.key, passage_in.key))
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_NON_RECIPROCAL,
                        involvedKeys));
        return;
    }
    if (prospectiveReal &&
        !roomListsPassageBack(snapshot_in, *prospective.key, passage_in.key))
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_NON_RECIPROCAL,
                        involvedKeys));
        return;
    }

    /* At least one real, reciprocal, non-contradictory endpoint and no
     * third/bad/cross-map/unresolvable/duplicate reverse reference --
     * genuinely the best this schema can show. Still UNKNOWN, never PASS:
     * see this file's own Doxygen. */
    findings_inout.push_back(
        makeFinding(AxiomCode::AX_PASS_02,
                    AxiomResult::UNKNOWN,
                    ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED,
                    involvedKeys));
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
