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
 * @file            unavailableReasonName.cc
 *
 * @brief           Implements unavailableReasonName(), declared in
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
    unavailableReasonName(UnavailableReason reason_in,
                          std::string      &unavailableReasonName_out)
{
    switch (reason_in)
    {
    case UnavailableReason::NONE:
    {
        unavailableReasonName_out = "NONE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case UnavailableReason::NULL_REFERENCE:
    {
        unavailableReasonName_out = "NULL_REFERENCE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case UnavailableReason::ENTITY_HAS_NO_MAP:
    {
        unavailableReasonName_out = "ENTITY_HAS_NO_MAP";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA:
    {
        unavailableReasonName_out = "NOT_TRACKED_BY_CURRENT_SCHEMA";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case UnavailableReason::NOT_EXPOSED_BY_CURRENT_API:
    {
        unavailableReasonName_out = "NOT_EXPOSED_BY_CURRENT_API";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case UnavailableReason::NOT_CAPTURED_IN_FOUNDATION_SLICE:
    {
        unavailableReasonName_out = "NOT_CAPTURED_IN_FOUNDATION_SLICE";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    }
    unavailableReasonName_out = "UNKNOWN_UNAVAILABLE_REASON";
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
