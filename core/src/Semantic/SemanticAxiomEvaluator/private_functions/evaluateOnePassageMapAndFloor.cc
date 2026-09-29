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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateOnePassageMapAndFloor(const PassageRecord         &passage_in,
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
        /* A provably invalid forward reference (wrong kind, unresolvable,
         * duplicate identity, cross-map, declared-map mismatch, or
         * known-bad liveness) must dominate this clause too, not only
         * AX-PASS-02's own cardinality check. */
        Finding finding{};
        if (makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_FLOOR_FORWARD_ENDPOINT_INVALID,
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
        /* No longer gated on
         * isRealPassageEndpoint -- mirrors AX-PASS-02's own
         * unconditional-on-isReferencePresent cross-map check, since an
         * unenumerated or unconfirmed cross-map reference is just as much a
         * known contradiction as a confirmed one. */
        Finding finding2{};
        if (makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_FLOOR_ENDPOINT_CROSS_MAP,
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

    PassageFloorAgreement agreement{};
    if (evaluatePassageFloorAgreement(knownSide,
                                      prospective,
                                      snapshot_in,
                                      agreement) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluatePassageFloorAgreement returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    switch (agreement)
    {
    case PassageFloorAgreement::DISAGREE:
    {
        Finding finding3{};
        if (makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_FLOOR_DISAGREEMENT,
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
    }
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    case PassageFloorAgreement::AMBIGUOUS:
    {
        Finding finding4{};
        if (makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_FLOOR_IDENTITY_AMBIGUOUS,
                        involvedKeys,
                        finding4) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding4);
    }
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    case PassageFloorAgreement::ENDPOINT_ROOM_FLOOR_INVALID:
    {
        Finding finding5{};
        if (makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_INVALID,
                        involvedKeys,
                        finding5) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding5);
    }
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    case PassageFloorAgreement::EVIDENCE_UNAVAILABLE:
    {
        Finding finding6{};
        if (makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::UNKNOWN,
                        ReasonCode::PASSAGE_FLOOR_EVIDENCE_UNAVAILABLE,
                        involvedKeys,
                        finding6) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding6);
    }
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    case PassageFloorAgreement::ENDPOINT_ROOM_FLOOR_UNVERIFIED:
    {
        Finding finding7{};
        if (makeFinding(
                AxiomCode::AX_PASS_04,
                AxiomResult::UNKNOWN,
                ReasonCode::PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_UNVERIFIED,
                involvedKeys,
                finding7) != SemanticAxiomEvaluatorStatus::
                                 SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding7);
    }
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    case PassageFloorAgreement::NOT_APPLICABLE:
    case PassageFloorAgreement::AGREE:
    {
        Finding finding8{};
        if (makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::PASS,
                        ReasonCode::PASSAGE_FLOOR_AGREEMENT_VALID,
                        involvedKeys,
                        finding8) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding8);
    }
        Finding finding9{};
        if (makeFinding(AxiomCode::AX_PASS_04,
                        AxiomResult::UNKNOWN,
                        ReasonCode::PASSAGE_FLOOR_ENDPOINT_PROOF_UNVERIFIED,
                        involvedKeys,
                        finding9) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding9);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
