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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateOnePassageCardinality(const PassageRecord         &passage_in,
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

    if (knownSide.isReferenceUnresolvable ||
        prospective.isReferenceUnresolvable)
    {
        Finding finding{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_UNRESOLVABLE,
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

    if (knownSide.isWrongKind || prospective.isWrongKind)
    {
        Finding finding2{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_WRONG_KIND,
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

    if (knownSide.isReasonInconsistent || prospective.isReasonInconsistent)
    {
        /* A keyed forward reference whose own
         * EntityRef::reason is not NONE violates EntityRef's documented
         * invariant and must never flow through as an ordinary valid
         * reference. */
        Finding finding3{};
        if (makeFinding(
                AxiomCode::AX_PASS_02,
                AxiomResult::FAIL,
                ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_REASON_INCONSISTENT,
                involvedKeys,
                finding3) != SemanticAxiomEvaluatorStatus::
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

    if (knownSide.isContainingMapAmbiguous ||
        prospective.isContainingMapAmbiguous)
    {
        /* A duplicate MapSnapshot::mapId for
         * the referenced map means no first-match lookup can supply
         * positive proof for this endpoint. */
        Finding finding4{};
        if (makeFinding(
                AxiomCode::AX_PASS_02,
                AxiomResult::FAIL,
                ReasonCode::
                    PASSAGE_CARDINALITY_ENDPOINT_CONTAINING_MAP_AMBIGUOUS,
                involvedKeys,
                finding4) != SemanticAxiomEvaluatorStatus::
                                 SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding4);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    /* isLive is trustworthy only when isLiveAvailable -- a keyed endpoint
     * with genuinely unproven liveness is not a bad endpoint merely because
     * ResolvedRoomEndpoint::isLive defaults to false; see
     * resolveRoomEndpoint.cc's Doxygen and the dedicated
     * liveness-unavailable check below. */
    const bool knownSideBad = knownSide.isReferencePresent &&
                              knownSide.isLiveAvailable && !knownSide.isLive;
    const bool prospectiveBad = prospective.isReferencePresent &&
                                prospective.isLiveAvailable &&
                                !prospective.isLive;
    if (knownSideBad || prospectiveBad)
    {
        Finding finding5{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_BAD,
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
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if ((knownSide.isFoundInSnapshot &&
         knownSide.isTargetDeclaredMapMismatch) ||
        (prospective.isFoundInSnapshot &&
         prospective.isTargetDeclaredMapMismatch))
    {
        Finding finding6{};
        if (makeFinding(
                AxiomCode::AX_PASS_02,
                AxiomResult::FAIL,
                ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_DECLARED_MAP_MISMATCH,
                involvedKeys,
                finding6) != SemanticAxiomEvaluatorStatus::
                                 SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding6);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if ((knownSide.isReferencePresent && knownSide.isCrossMap) ||
        (prospective.isReferencePresent && prospective.isCrossMap))
    {
        Finding finding7{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_CROSS_MAP,
                        involvedKeys,
                        finding7) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding7);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (knownSide.isReferencePresent && prospective.isReferencePresent &&
        knownSide.key.has_value() && prospective.key.has_value() &&
        (*knownSide.key == *prospective.key))
    {
        Finding finding8{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_DUPLICATE_ENDPOINT,
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
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    ReversePassageEndpointScan reverseScan{};
    if (scanReversePassageEndpoints(passage_in,
                                    expectedMapId,
                                    snapshot_in,
                                    reverseScan) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: scanReversePassageEndpoints returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    if (!reverseScan.badReverseRoomKeys.empty())
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           reverseScan.badReverseRoomKeys.begin(),
                           reverseScan.badReverseRoomKeys.end());
        Finding finding9{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_BAD,
                        anomalyKeys,
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
    if (!reverseScan.crossMapReverseRoomKeys.empty())
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           reverseScan.crossMapReverseRoomKeys.begin(),
                           reverseScan.crossMapReverseRoomKeys.end());
        Finding finding10{};
        if (makeFinding(
                AxiomCode::AX_PASS_02,
                AxiomResult::FAIL,
                ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_CROSS_MAP,
                anomalyKeys,
                finding10) != SemanticAxiomEvaluatorStatus::
                                  SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding10);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (!reverseScan.duplicateIdentityRoomKeys.empty())
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           reverseScan.duplicateIdentityRoomKeys.begin(),
                           reverseScan.duplicateIdentityRoomKeys.end());
        Finding finding11{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_DUPLICATE_ROOM_IDENTITY,
                        anomalyKeys,
                        finding11) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding11);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (!reverseScan.wrongKindReverseRoomKeys.empty())
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           reverseScan.wrongKindReverseRoomKeys.begin(),
                           reverseScan.wrongKindReverseRoomKeys.end());
        Finding finding12{};
        if (makeFinding(
                AxiomCode::AX_PASS_02,
                AxiomResult::FAIL,
                ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_WRONG_KIND,
                anomalyKeys,
                finding12) != SemanticAxiomEvaluatorStatus::
                                  SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding12);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (!reverseScan.duplicateReferenceRoomKeys.empty())
    {
        std::vector<EntityKey> anomalyKeys = involvedKeys;
        anomalyKeys.insert(anomalyKeys.end(),
                           reverseScan.duplicateReferenceRoomKeys.begin(),
                           reverseScan.duplicateReferenceRoomKeys.end());
        Finding finding13{};
        if (makeFinding(
                AxiomCode::AX_PASS_02,
                AxiomResult::FAIL,
                ReasonCode::PASSAGE_CARDINALITY_REVERSE_REFERENCE_DUPLICATED,
                anomalyKeys,
                finding13) != SemanticAxiomEvaluatorStatus::
                                  SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding13);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (knownSide.isDuplicateIdentity || prospective.isDuplicateIdentity)
    {
        Finding finding14{};
        if (makeFinding(
                AxiomCode::AX_PASS_02,
                AxiomResult::FAIL,
                ReasonCode::
                    PASSAGE_CARDINALITY_FORWARD_ENDPOINT_DUPLICATE_IDENTITY,
                involvedKeys,
                finding14) != SemanticAxiomEvaluatorStatus::
                                  SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding14);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if ((knownSide.isReferencePresent && !knownSide.isLiveAvailable) ||
        (prospective.isReferencePresent && !prospective.isLiveAvailable))
    {
        /* A forward endpoint reference is present with no other known
         * contradiction, but its own liveness is genuinely unproven: this
         * passage's cardinality cannot be certified either way. Checked
         * after every reverse-scan FAIL above so an independently known
         * contradiction still dominates (FAIL > UNKNOWN). */
        Finding finding15{};
        if (makeFinding(
                AxiomCode::AX_PASS_02,
                AxiomResult::UNKNOWN,
                ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_LIVENESS_UNAVAILABLE,
                involvedKeys,
                finding15) != SemanticAxiomEvaluatorStatus::
                                  SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding15);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
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
        Finding finding16{};
        if (makeFinding(
                AxiomCode::AX_PASS_02,
                AxiomResult::UNKNOWN,
                ReasonCode::
                    PASSAGE_CARDINALITY_REVERSE_ENDPOINT_LIVENESS_UNAVAILABLE,
                anomalyKeys,
                finding16) != SemanticAxiomEvaluatorStatus::
                                  SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding16);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    bool knownSideReal{};
    if (isRealPassageEndpoint(knownSide, knownSideReal) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isRealPassageEndpoint returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool prospectiveReal{};
    if (isRealPassageEndpoint(prospective, prospectiveReal) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isRealPassageEndpoint returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

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
        Finding finding17{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_THIRD_ENDPOINT,
                        anomalyKeys,
                        finding17) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding17);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (unionKeys.empty())
    {
        Finding finding18{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_NO_CONFIRMED_ENDPOINT,
                        involvedKeys,
                        finding18) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding18);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    bool listsPassageBack{};
    if ((knownSideReal) && roomListsPassageBack(snapshot_in,
                                                *knownSide.key,
                                                passage_in.key,
                                                listsPassageBack) !=
                               SemanticAxiomEvaluatorStatus::
                                   SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: roomListsPassageBack returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (knownSideReal && !listsPassageBack)
    {
        Finding finding19{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_NON_RECIPROCAL,
                        involvedKeys,
                        finding19) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding19);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    bool listsPassageBack2{};
    if ((prospectiveReal) && roomListsPassageBack(snapshot_in,
                                                  *prospective.key,
                                                  passage_in.key,
                                                  listsPassageBack2) !=
                                 SemanticAxiomEvaluatorStatus::
                                     SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: roomListsPassageBack returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (prospectiveReal && !listsPassageBack2)
    {
        Finding finding20{};
        if (makeFinding(AxiomCode::AX_PASS_02,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_CARDINALITY_NON_RECIPROCAL,
                        involvedKeys,
                        finding20) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding20);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    /* At least one real, reciprocal, non-contradictory endpoint and no
     * third/bad/cross-map/unresolvable/duplicate reverse reference --
     * genuinely the best this schema can show. Still UNKNOWN, never PASS:
     * see this file's own Doxygen. */
    Finding finding21{};
    if (makeFinding(AxiomCode::AX_PASS_02,
                    AxiomResult::UNKNOWN,
                    ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED,
                    involvedKeys,
                    finding21) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeFinding returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    findings_inout.push_back(finding21);

    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
