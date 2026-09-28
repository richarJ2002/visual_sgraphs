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
 * @file            missingProofOwnerName.cc
 *
 * @brief           Implements missingProofOwnerName(), declared in
 *                  EnumNames.h.
 */

#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    missingProofOwnerName(MissingProofOwner owner_in,
                          std::string      &missingProofOwnerName_out)
{
    switch (owner_in)
    {
    case MissingProofOwner::NONE:
    {
        missingProofOwnerName_out = "NONE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case MissingProofOwner::PHASE_2:
    {
        missingProofOwnerName_out = "PHASE_2";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case MissingProofOwner::PHASE_3:
    {
        missingProofOwnerName_out = "PHASE_3";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case MissingProofOwner::PHASE_4:
    {
        missingProofOwnerName_out = "PHASE_4";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case MissingProofOwner::PHASE_4_OR_5:
    {
        missingProofOwnerName_out = "PHASE_4_OR_5";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case MissingProofOwner::PHASE_5:
    {
        missingProofOwnerName_out = "PHASE_5";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case MissingProofOwner::PHASE_6:
    {
        missingProofOwnerName_out = "PHASE_6";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case MissingProofOwner::PHASE_7:
    {
        missingProofOwnerName_out = "PHASE_7";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case MissingProofOwner::PHASE_8:
    {
        missingProofOwnerName_out = "PHASE_8";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case MissingProofOwner::SCOPE_DECISION_REQUIRED:
    {
        missingProofOwnerName_out = "SCOPE_DECISION_REQUIRED";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    }
    missingProofOwnerName_out = "UNKNOWN_MISSING_PROOF_OWNER";
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
