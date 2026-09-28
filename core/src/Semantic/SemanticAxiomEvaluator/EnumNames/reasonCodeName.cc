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
 * @file            reasonCodeName.cc
 *
 * @brief           Implements reasonCodeName(), declared in EnumNames.h.
 */

#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus reasonCodeName(ReasonCode   reason_in,
                                            std::string &reasonCodeName_out)
{
    switch (reason_in)
    {
    case ReasonCode::FRAME_TRANSITION_EVALUATION_REQUIRED:
    {
        reasonCodeName_out = "FRAME_TRANSITION_EVALUATION_REQUIRED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::FRAME_EQUIVARIANCE_NOT_YET_IMPLEMENTED:
    {
        reasonCodeName_out = "FRAME_EQUIVARIANCE_NOT_YET_IMPLEMENTED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE:
    {
        reasonCodeName_out =
            "WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_SINGLE_VALID_OWNER";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_MULTIPLE_OWNERS:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_MULTIPLE_OWNERS";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_BAD:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_BAD";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_UNRESOLVABLE:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_UNRESOLVABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_CROSS_MAP:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_CROSS_MAP";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OBSERVATION_RAY_EVIDENCE_UNAVAILABLE:
    {
        reasonCodeName_out = "WALL_OBSERVATION_RAY_EVIDENCE_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_ABSENT:
    {
        reasonCodeName_out = "WALL_TWIN_ABSENT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_SELF:
    {
        reasonCodeName_out = "WALL_TWIN_SELF";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_ASYMMETRIC:
    {
        reasonCodeName_out = "WALL_TWIN_ASYMMETRIC";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_BAD:
    {
        reasonCodeName_out = "WALL_TWIN_BAD";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_CROSS_MAP:
    {
        reasonCodeName_out = "WALL_TWIN_CROSS_MAP";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_WRONG_TYPE:
    {
        reasonCodeName_out = "WALL_TWIN_WRONG_TYPE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_MAP_UNAVAILABLE:
    {
        reasonCodeName_out = "WALL_TWIN_MAP_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_SHARED_OWNER_FORBIDDEN:
    {
        reasonCodeName_out = "WALL_TWIN_SHARED_OWNER_FORBIDDEN";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_STRUCTURALLY_VALID_GEOMETRY_UNVERIFIED:
    {
        reasonCodeName_out = "WALL_TWIN_STRUCTURALLY_VALID_GEOMETRY_UNVERIFIED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_PROVENANCE_NOT_PASSABLE:
    {
        reasonCodeName_out = "PASSAGE_PROVENANCE_NOT_PASSABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_PROVENANCE_FULL_CHAIN_UNVERIFIABLE:
    {
        reasonCodeName_out = "PASSAGE_PROVENANCE_FULL_CHAIN_UNVERIFIABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_NO_CONFIRMED_ENDPOINT:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_NO_CONFIRMED_ENDPOINT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_DUPLICATE_ENDPOINT:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_DUPLICATE_ENDPOINT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_BAD:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_ENDPOINT_BAD";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_UNRESOLVABLE:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_ENDPOINT_UNRESOLVABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_CROSS_MAP:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_ENDPOINT_CROSS_MAP";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_NON_RECIPROCAL:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_NON_RECIPROCAL";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_VALID:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_VALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_SLOT_KNOWN_SIDE_NOT_CONFIRMED:
    {
        reasonCodeName_out = "PASSAGE_SLOT_KNOWN_SIDE_NOT_CONFIRMED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_SLOT_STATE_VALID:
    {
        reasonCodeName_out = "PASSAGE_SLOT_STATE_VALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_FLOOR_ENDPOINT_CROSS_MAP:
    {
        reasonCodeName_out = "PASSAGE_FLOOR_ENDPOINT_CROSS_MAP";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_FLOOR_DISAGREEMENT:
    {
        reasonCodeName_out = "PASSAGE_FLOOR_DISAGREEMENT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_FLOOR_EVIDENCE_UNAVAILABLE:
    {
        reasonCodeName_out = "PASSAGE_FLOOR_EVIDENCE_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_FLOOR_AGREEMENT_VALID:
    {
        reasonCodeName_out = "PASSAGE_FLOOR_AGREEMENT_VALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_CREATION_PROVENANCE_UNAVAILABLE:
    {
        reasonCodeName_out = "ROOM_CREATION_PROVENANCE_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FAR_SIDE_EVIDENCE_UNAVAILABLE:
    {
        reasonCodeName_out = "ROOM_FAR_SIDE_EVIDENCE_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_TOO_FEW_CORNERS:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_TOO_FEW_CORNERS";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_DEGENERATE_EDGE:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_DEGENERATE_EDGE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_SELF_INTERSECTING:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_SELF_INTERSECTING";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_NO_WALL_EVIDENCE:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_NO_WALL_EVIDENCE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_CONFLICTING_STATE:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_CONFLICTING_STATE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_GAP_CORRESPONDENCE_UNVERIFIED:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_GAP_CORRESPONDENCE_UNVERIFIED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_NOT_YET_COMPLETE:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_NOT_YET_COMPLETE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_STRUCTURALLY_VALID:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_STRUCTURALLY_VALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_UNLINKED:
    {
        reasonCodeName_out = "ROOM_FLOOR_UNLINKED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_NON_RECIPROCAL:
    {
        reasonCodeName_out = "ROOM_FLOOR_NON_RECIPROCAL";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_CROSS_MAP:
    {
        reasonCodeName_out = "ROOM_FLOOR_CROSS_MAP";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_RECIPROCAL_VALID:
    {
        reasonCodeName_out = "ROOM_FLOOR_RECIPROCAL_VALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::FLOOR_PASSAGE_CROSS_FLOOR:
    {
        reasonCodeName_out = "FLOOR_PASSAGE_CROSS_FLOOR";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::FLOOR_PASSAGE_EVIDENCE_UNAVAILABLE:
    {
        reasonCodeName_out = "FLOOR_PASSAGE_EVIDENCE_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::FLOOR_PASSAGE_AGREEMENT_VALID:
    {
        reasonCodeName_out = "FLOOR_PASSAGE_AGREEMENT_VALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::LIFECYCLE_QUARANTINE_PROVENANCE_UNAVAILABLE:
    {
        reasonCodeName_out = "LIFECYCLE_QUARANTINE_PROVENANCE_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::TRANSACTION_EVALUATION_REQUIRES_TRANSITION:
    {
        reasonCodeName_out = "TRANSACTION_EVALUATION_REQUIRES_TRANSITION";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::TRANSACTION_POSTCONDITION_NOT_YET_IMPLEMENTED:
    {
        reasonCodeName_out = "TRANSACTION_POSTCONDITION_NOT_YET_IMPLEMENTED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_ZERO_CONFIRMED_ROOMS:
    {
        reasonCodeName_out = "COMPLETENESS_ZERO_CONFIRMED_ROOMS";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_LIVE_PROSPECTIVE_ROOM_PRESENT:
    {
        reasonCodeName_out = "COMPLETENESS_LIVE_PROSPECTIVE_ROOM_PRESENT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_ROOM_BOUNDARY_NOT_COMPLETE:
    {
        reasonCodeName_out = "COMPLETENESS_ROOM_BOUNDARY_NOT_COMPLETE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_PASSAGE_ENDPOINTS_INVALID:
    {
        reasonCodeName_out = "COMPLETENESS_PASSAGE_ENDPOINTS_INVALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_HARD_CONTRADICTION:
    {
        reasonCodeName_out = "COMPLETENESS_HARD_CONTRADICTION";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_EVIDENCE_UNAVAILABLE:
    {
        reasonCodeName_out = "COMPLETENESS_EVIDENCE_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_NO_MAP_PRESENT:
    {
        reasonCodeName_out = "COMPLETENESS_NO_MAP_PRESENT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_ALL_CLEAR:
    {
        reasonCodeName_out = "COMPLETENESS_ALL_CLEAR";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::MERGE_PRESERVATION_NOT_YET_IMPLEMENTED:
    {
        reasonCodeName_out = "MERGE_PRESERVATION_NOT_YET_IMPLEMENTED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_THIRD_ENDPOINT:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_THIRD_ENDPOINT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_BAD:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_REVERSE_ENDPOINT_BAD";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_CROSS_MAP:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_REVERSE_ENDPOINT_CROSS_MAP";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_ROOM_HAS_MALFORMED_REFERENCE:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_ROOM_HAS_MALFORMED_REFERENCE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_DUPLICATE_ROOM_IDENTITY:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_DUPLICATE_ROOM_IDENTITY";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_LIVENESS_UNAVAILABLE:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_LIVENESS_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_WRONG_VARIANT:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_WRONG_VARIANT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_MISMATCH:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_DECLARED_MAP_MISMATCH";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_NOT_RECIPROCAL:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_NOT_RECIPROCAL";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_DUPLICATE_IDENTITY:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_DUPLICATE_IDENTITY";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_UNAVAILABLE:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_RECORD_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_NON_FINITE_CORNER:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_NON_FINITE_CORNER";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_EDGE_SUPPORT_UNVERIFIED:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_EDGE_SUPPORT_UNVERIFIED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_DUPLICATE_REVERSE_MEMBERSHIP:
    {
        reasonCodeName_out = "ROOM_FLOOR_DUPLICATE_REVERSE_MEMBERSHIP";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_CLAIMED_BY_MULTIPLE_FLOORS:
    {
        reasonCodeName_out = "ROOM_FLOOR_CLAIMED_BY_MULTIPLE_FLOORS";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_DUPLICATE_IDENTITY:
    {
        reasonCodeName_out = "ROOM_FLOOR_DUPLICATE_IDENTITY";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_PASSAGE_SLOT_PROOF_UNAVAILABLE:
    {
        reasonCodeName_out = "COMPLETENESS_PASSAGE_SLOT_PROOF_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_SLOT_ENDPOINT_PROOF_UNVERIFIED:
    {
        reasonCodeName_out = "PASSAGE_SLOT_ENDPOINT_PROOF_UNVERIFIED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_FLOOR_ENDPOINT_PROOF_UNVERIFIED:
    {
        reasonCodeName_out = "PASSAGE_FLOOR_ENDPOINT_PROOF_UNVERIFIED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::FLOOR_PASSAGE_ENDPOINT_PROOF_UNVERIFIED:
    {
        reasonCodeName_out = "FLOOR_PASSAGE_ENDPOINT_PROOF_UNVERIFIED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_WRONG_KIND:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_REVERSE_ENDPOINT_WRONG_KIND";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_REVERSE_REFERENCE_DUPLICATED:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_REVERSE_REFERENCE_DUPLICATED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_FORWARD_ENDPOINT_DUPLICATE_IDENTITY:
    {
        reasonCodeName_out =
            "PASSAGE_CARDINALITY_FORWARD_ENDPOINT_DUPLICATE_IDENTITY";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_WALL_DUPLICATE_IDENTITY:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_WALL_DUPLICATE_IDENTITY";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_WALL_DECLARED_MAP_MISMATCH:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_WALL_DECLARED_MAP_MISMATCH";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_NOT_LIVE:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_RECORD_NOT_LIVE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_INVALID_WALL_EVIDENCE:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_INVALID_WALL_EVIDENCE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_REVERSE_CLAIM_WITHOUT_FORWARD_LINK:
    {
        reasonCodeName_out = "ROOM_FLOOR_REVERSE_CLAIM_WITHOUT_FORWARD_LINK";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_WRONG_KIND:
    {
        reasonCodeName_out = "ROOM_FLOOR_WRONG_KIND";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_DECLARED_MAP_MISMATCH:
    {
        reasonCodeName_out = "ROOM_FLOOR_DECLARED_MAP_MISMATCH";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_FLOOR_IDENTITY_AMBIGUOUS:
    {
        reasonCodeName_out = "PASSAGE_FLOOR_IDENTITY_AMBIGUOUS";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::FLOOR_PASSAGE_IDENTITY_AMBIGUOUS:
    {
        reasonCodeName_out = "FLOOR_PASSAGE_IDENTITY_AMBIGUOUS";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_ROOM_CREATION_PROVENANCE_UNAVAILABLE:
    {
        reasonCodeName_out =
            "COMPLETENESS_ROOM_CREATION_PROVENANCE_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_DUPLICATE_IDENTITY:
    {
        reasonCodeName_out = "COMPLETENESS_DUPLICATE_IDENTITY";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_WRONG_KIND:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_ENDPOINT_WRONG_KIND";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_DECLARED_MAP_MISMATCH:
    {
        reasonCodeName_out =
            "PASSAGE_CARDINALITY_ENDPOINT_DECLARED_MAP_MISMATCH";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_LIVENESS_UNAVAILABLE:
    {
        reasonCodeName_out =
            "PASSAGE_CARDINALITY_ENDPOINT_LIVENESS_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_LIVENESS_UNAVAILABLE:
    {
        reasonCodeName_out =
            "PASSAGE_CARDINALITY_REVERSE_ENDPOINT_LIVENESS_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_SLOT_FORWARD_ENDPOINT_INVALID:
    {
        reasonCodeName_out = "PASSAGE_SLOT_FORWARD_ENDPOINT_INVALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_FLOOR_FORWARD_ENDPOINT_INVALID:
    {
        reasonCodeName_out = "PASSAGE_FLOOR_FORWARD_ENDPOINT_INVALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_INVALID:
    {
        reasonCodeName_out = "PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_INVALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::FLOOR_PASSAGE_FORWARD_ENDPOINT_INVALID:
    {
        reasonCodeName_out = "FLOOR_PASSAGE_FORWARD_ENDPOINT_INVALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::FLOOR_PASSAGE_ENDPOINT_ROOM_FLOOR_INVALID:
    {
        reasonCodeName_out = "FLOOR_PASSAGE_ENDPOINT_ROOM_FLOOR_INVALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_WALL_WRONG_KEY_KIND:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_WALL_WRONG_KEY_KIND";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_WALL_WRONG_PLANE_TYPE:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_WALL_WRONG_PLANE_TYPE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_WRONG_KEY_KIND:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_WRONG_KEY_KIND";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_WALL_DECLARED_MAP_UNAVAILABLE:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_WALL_DECLARED_MAP_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_RECIPROCAL_DUPLICATE:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_RECIPROCAL_DUPLICATE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_BOUNDARY_WALL_EVIDENCE_UNAVAILABLE:
    {
        reasonCodeName_out = "ROOM_BOUNDARY_WALL_EVIDENCE_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_DUPLICATE_MAP_IDENTITY:
    {
        reasonCodeName_out = "COMPLETENESS_DUPLICATE_MAP_IDENTITY";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_ROOM_WRONG_KIND:
    {
        reasonCodeName_out = "ROOM_FLOOR_ROOM_WRONG_KIND";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_ROOM_DECLARED_MAP_MISMATCH:
    {
        reasonCodeName_out = "ROOM_FLOOR_ROOM_DECLARED_MAP_MISMATCH";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_ROOM_DECLARED_MAP_UNAVAILABLE:
    {
        reasonCodeName_out = "ROOM_FLOOR_ROOM_DECLARED_MAP_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::COMPLETENESS_ROOM_HAS_MALFORMED_PASSAGE_REFERENCE:
    {
        reasonCodeName_out =
            "COMPLETENESS_ROOM_HAS_MALFORMED_PASSAGE_REFERENCE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_REASON_INCONSISTENT:
    {
        reasonCodeName_out = "PASSAGE_CARDINALITY_ENDPOINT_REASON_INCONSISTENT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_SLOT_ENDPOINT_CROSS_MAP:
    {
        reasonCodeName_out = "PASSAGE_SLOT_ENDPOINT_CROSS_MAP";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::FLOOR_PASSAGE_ENDPOINT_CROSS_MAP:
    {
        reasonCodeName_out = "FLOOR_PASSAGE_ENDPOINT_CROSS_MAP";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_FLOOR_DECLARED_MAP_UNAVAILABLE:
    {
        reasonCodeName_out = "ROOM_FLOOR_FLOOR_DECLARED_MAP_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_CONTAINING_MAP_AMBIGUOUS:
    {
        reasonCodeName_out = "ROOM_FLOOR_CONTAINING_MAP_AMBIGUOUS";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_ROOM_REASON_INCONSISTENT:
    {
        reasonCodeName_out = "ROOM_FLOOR_ROOM_REASON_INCONSISTENT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_REVERSE_MEMBER_INVALID:
    {
        reasonCodeName_out = "ROOM_FLOOR_REVERSE_MEMBER_INVALID";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::ROOM_FLOOR_REVERSE_MEMBER_LIVENESS_UNAVAILABLE:
    {
        reasonCodeName_out = "ROOM_FLOOR_REVERSE_MEMBER_LIVENESS_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_CONTAINING_MAP_AMBIGUOUS:
    {
        reasonCodeName_out =
            "PASSAGE_CARDINALITY_ENDPOINT_CONTAINING_MAP_AMBIGUOUS";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_REASON_INCONSISTENT:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_REASON_INCONSISTENT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_CONTAINING_MAP_AMBIGUOUS:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_CONTAINING_MAP_AMBIGUOUS";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_UNAVAILABLE:
    {
        reasonCodeName_out = "WALL_OWNERSHIP_OWNER_DECLARED_MAP_UNAVAILABLE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_REASON_INCONSISTENT:
    {
        reasonCodeName_out = "WALL_TWIN_REASON_INCONSISTENT";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_DUPLICATE_IDENTITY:
    {
        reasonCodeName_out = "WALL_TWIN_DUPLICATE_IDENTITY";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::WALL_TWIN_CONTAINING_MAP_AMBIGUOUS:
    {
        reasonCodeName_out = "WALL_TWIN_CONTAINING_MAP_AMBIGUOUS";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_UNVERIFIED:
    {
        reasonCodeName_out = "PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_UNVERIFIED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case ReasonCode::FLOOR_PASSAGE_ENDPOINT_ROOM_FLOOR_UNVERIFIED:
    {
        reasonCodeName_out = "FLOOR_PASSAGE_ENDPOINT_ROOM_FLOOR_UNVERIFIED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    }
    reasonCodeName_out = "UNKNOWN_REASON_CODE";
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
