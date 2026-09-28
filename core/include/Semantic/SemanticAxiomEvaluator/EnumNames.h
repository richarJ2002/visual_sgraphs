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
 * @file            EnumNames.h
 *
 * @brief           Declares the public stable human-readable name functions
 *                  for every enum used in semantic evaluation, findings, and
 *                  reporting. A public-functions module in its own right
 *                  (CPP_CODING_STANDARD.md Section 5.4) -- previously
 *                  misplaced under objects/, even though this file declares
 *                  no type of its own, only a group of free functions.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_ENUM_NAMES_H
#define SEMANTIC_AXIOM_EVALUATOR_ENUM_NAMES_H

#include <string>

#include "Semantic/SemanticAxiomEvaluator/SemanticAxiomEvaluatorStatus.h"
#include "Semantic/SemanticAxiomEvaluator/objects/AxiomClass.h"
#include "Semantic/SemanticAxiomEvaluator/objects/AxiomCode.h"
#include "Semantic/SemanticAxiomEvaluator/objects/AxiomResult.h"
#include "Semantic/SemanticAxiomEvaluator/objects/CapabilityLevel.h"
#include "Semantic/SemanticAxiomEvaluator/objects/MissingProofOwner.h"
#include "Semantic/SemanticAxiomEvaluator/objects/ReasonCode.h"
#include "Semantic/SemanticGraphSnapshot/objects/EntityKind.h"
#include "Semantic/SemanticGraphSnapshot/objects/UnavailableReason.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief Return stable human-readable name for an axiom code.
 * @param code_in The axiom code enum value.
 * @param[out] axiomCodeName_out Stable name string (e.g. "AX_FRAME_01",
 * "AX_WALL_01"), or "UNKNOWN_AXIOM_CODE" for a value outside the declared enum.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    axiomCodeName(AxiomCode code_in, std::string &axiomCodeName_out);

/*!
 * @brief Return stable human-readable name for an axiom result.
 * @param result_in The result enum value.
 * @param[out] axiomResultName_out Stable name string ("PASS", "FAIL", or
 * "UNKNOWN"), or "UNKNOWN_AXIOM_RESULT" for a value outside the declared enum.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    axiomResultName(AxiomResult result_in, std::string &axiomResultName_out);

/*!
 * @brief Return stable human-readable name for an axiom class.
 * @param class_in The class enum value.
 * @param[out] axiomClassName_out Stable name string ("HARD" or "DERIVED"), or
 * "UNKNOWN_AXIOM_CLASS" for a value outside the declared enum.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    axiomClassName(AxiomClass class_in, std::string &axiomClassName_out);

/*!
 * @brief Return stable human-readable name for a reason code.
 * @param reason_in The reason code enum value.
 * @param[out] reasonCodeName_out Stable name string (e.g.
 * "FRAME_TRANSITION_EVALUATION_REQUIRED"), or "UNKNOWN_REASON_CODE" for a value
 * outside the declared enum.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    reasonCodeName(ReasonCode reason_in, std::string &reasonCodeName_out);

/*!
 * @brief Return stable human-readable name for an entity kind.
 * @param kind_in The entity kind enum value.
 * @param[out] entityKindName_out Stable name string ("ROOM", "WALL", "PASSAGE",
 * or "FLOOR"), or "UNKNOWN_ENTITY_KIND" for a value outside the declared enum.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    entityKindName(EntityKind kind_in, std::string &entityKindName_out);

/*!
 * @brief Return stable human-readable name for an unavailable reason.
 * @param reason_in The unavailable reason enum value.
 * @param[out] unavailableReasonName_out Stable name string (e.g. "NONE",
 * "NOT_TRACKED_BY_CURRENT_SCHEMA"), or "UNKNOWN_UNAVAILABLE_REASON" for a value
 * outside the declared enum.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    unavailableReasonName(UnavailableReason reason_in,
                          std::string      &unavailableReasonName_out);

/*!
 * @brief Return stable human-readable name for a capability level.
 * @param level_in The capability level enum value.
 * @param[out] capabilityLevelName_out Stable name string ("FULL", "PARTIAL", or
 * "DEFERRED"), or "UNKNOWN_CAPABILITY_LEVEL" for a value outside the declared
 * enum.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    capabilityLevelName(CapabilityLevel level_in,
                        std::string    &capabilityLevelName_out);

/*!
 * @brief Return stable human-readable name for a missing-proof owner.
 * @param owner_in The missing-proof owner enum value.
 * @param[out] missingProofOwnerName_out Stable name string (e.g. "NONE",
 * "PHASE_2", "SCOPE_DECISION_REQUIRED"), or "UNKNOWN_MISSING_PROOF_OWNER" for a
 * value outside the declared enum.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    missingProofOwnerName(MissingProofOwner owner_in,
                          std::string      &missingProofOwnerName_out);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_ENUM_NAMES_H
