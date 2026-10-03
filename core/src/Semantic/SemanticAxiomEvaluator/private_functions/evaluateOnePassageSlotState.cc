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
 * @file            evaluateOnePassageSlotState.cc
 *
 * @brief           Implements evaluateOnePassageSlotState(), declared
 *                  in private_functions.h.
 *
 *                  The current Passage model has exactly one
 *                  prospective-room field
 *                  (PassageRecord::prospectiveRoomRef), so "the
 *                  opposite slot contains at most one stable
 *                  prospective handle" is structurally guaranteed
 *                  rather than checked; the substantive, checkable
 *                  clause left is that a "known"/near side, when it
 *                  resolves at all, must actually be a confirmed
 *                  (ROOM-variant) room rather than an unpromoted
 *                  prospective handle -- a fact read directly from
 *                  RoomRecord::variant, independent of the
 *                  authoritative-endpoint-slot gap
 *                  computeAxiomCapabilityTable() records for this code
 *                  (downgraded to PARTIAL alongside AX-PASS-02/04, since the
 *                  "which slot is authoritative" clause is unprovable, even
 *                  though this specific variant-confirmation clause is not).
 *                  Cardinality/duplicate/reciprocity issues are
 *                  AX-PASS-02's concern, not re-checked here.
 *
 *                     The terminal success path also appends a typed
 *                     PASSAGE_SLOT_ENDPOINT_PROOF_UNVERIFIED UNKNOWN alongside
 *                     the clause-level PASSAGE_SLOT_STATE_VALID PASS, so the
 *                     AX-PASS-03 aggregate can never become PASS while
 *                     PassageRecord::endpointSlotReason remains
 *                     NOT_TRACKED_BY_CURRENT_SCHEMA (FAIL > UNKNOWN > PASS
 *                     still lets an independently observed contradiction above
 *                     dominate, since this addition only runs after every FAIL
 *                     return).
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateOnePassageSlotState(const PassageRecord         &passage_in,
                                const SemanticGraphSnapshot &snapshot_in,
                                std::vector<Finding>        &findings_inout)
{
    const long unsigned int expectedMapId =
        passage_in.declaredMapId.value_or(passage_in.key.mapId);
    ResolvedRoomEndpoint knownSide{};
    if (resolveRoomEndpoint(passage_in.knownSideRoomRef,
                            expectedMapId,
                            snapshot_in,
                            knownSide) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: resolveRoomEndpoint returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    ResolvedRoomEndpoint prospective{};
    if (resolveRoomEndpoint(passage_in.prospectiveRoomRef,
                            expectedMapId,
                            snapshot_in,
                            prospective) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: resolveRoomEndpoint returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<EntityKey> involvedKeys{passage_in.key};
    if (knownSide.key.has_value())
    {
        involvedKeys.push_back(*knownSide.key);
    }
    if (prospective.key.has_value())
    {
        involvedKeys.push_back(*prospective.key);
    }

    bool isKnownInvalidPassageEndpointReference2{};
    if (isKnownInvalidPassageEndpointReference(
            knownSide,
            isKnownInvalidPassageEndpointReference2) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // isKnownInvalidPassageEndpointReference cannot fail; continue as
        // before.
    }
    bool isKnownInvalidPassageEndpointReference3{};
    if (!(isKnownInvalidPassageEndpointReference2) &&
        isKnownInvalidPassageEndpointReference(
            prospective,
            isKnownInvalidPassageEndpointReference3) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // isKnownInvalidPassageEndpointReference cannot fail; continue as
        // before.
    }
    if (isKnownInvalidPassageEndpointReference2 ||
        isKnownInvalidPassageEndpointReference3)
    {
        /* A provably invalid
         * forward reference must dominate this clause too, not only
         * AX-PASS-02's own cardinality check. */
        Finding finding{};
        if (makeFinding(AxiomCode::AX_PASS_03,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_SLOT_FORWARD_ENDPOINT_INVALID,
                        involvedKeys,
                        finding) != SemanticAxiomEvaluatorStatus::
                                        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if ((knownSide.isReferencePresent && knownSide.isCrossMap) ||
        (prospective.isReferencePresent && prospective.isCrossMap))
    {
        /* Mirrors AX-PASS-02's own
         * unconditional-on-isReferencePresent cross-map check (not gated on
         * isRealPassageEndpoint, since an unenumerated or unconfirmed
         * cross-map reference is just as much a known contradiction as a
         * confirmed one) -- isKnownInvalidPassageEndpointReference()
         * deliberately excludes cross-map so each axiom can use its own
         * dedicated reason code. */
        Finding finding2{};
        if (makeFinding(AxiomCode::AX_PASS_03,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_SLOT_ENDPOINT_CROSS_MAP,
                        involvedKeys,
                        finding2) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding2);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (knownSide.isReferencePresent && knownSide.isFoundInSnapshot &&
        knownSide.isLive && !knownSide.isConfirmedRoomVariant)
    {
        Finding finding3{};
        if (makeFinding(AxiomCode::AX_PASS_03,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_SLOT_KNOWN_SIDE_NOT_CONFIRMED,
                        involvedKeys,
                        finding3) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding3);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    Finding finding4{};
    if (makeFinding(AxiomCode::AX_PASS_03,
                    AxiomResult::PASS,
                    ReasonCode::PASSAGE_SLOT_STATE_VALID,
                    involvedKeys,
                    finding4) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeFinding returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    findings_inout.push_back(finding4);
    Finding finding5{};
    if (makeFinding(AxiomCode::AX_PASS_03,
                    AxiomResult::UNKNOWN,
                    ReasonCode::PASSAGE_SLOT_ENDPOINT_PROOF_UNVERIFIED,
                    involvedKeys,
                    finding5) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeFinding returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    findings_inout.push_back(finding5);

    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
