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
 * @file            evaluateAxComp01.cc
 *
 * @brief           Implements evaluateAxComp01(), declared in
 *                  private_functions.h.
 *
 *                  Purely derived from already-computed MapCompletenessResult
 *                  entries (computeConservativeMapCompleteness() guarantees
 *                  MapCompletenessResult::reasons is homogeneous with
 *                  MapCompletenessResult::conservativeResult's own severity,
 *                  so every Finding built here reuses that same result
 *                  value directly, one per listed reason).
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateAxComp01(const std::vector<MapCompletenessResult> &completeness_in,
                     std::vector<Finding>                     &findings_inout)
{
    if (completeness_in.empty())
    {
        Finding finding{};
        if (makeFinding(AxiomCode::AX_COMP_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::COMPLETENESS_NO_MAP_PRESENT,
                        {},
                        finding) != SemanticAxiomEvaluatorStatus::
                                        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            // makeFinding cannot fail; continue as before.
        }
        findings_inout.push_back(finding);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    for (const MapCompletenessResult &completeness : completeness_in)
    {
        if (completeness.reasons.empty())
        {
            Finding finding2{};
            if (makeFinding(AxiomCode::AX_COMP_01,
                            completeness.conservativeResult,
                            ReasonCode::COMPLETENESS_ALL_CLEAR,
                            completeness.relevantEntityKeys,
                            finding2) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                // makeFinding cannot fail; continue as before.
            }
            findings_inout.push_back(finding2);
            continue;
        }
        for (const ReasonCode reason : completeness.reasons)
        {
            Finding finding3{};
            if (makeFinding(AxiomCode::AX_COMP_01,
                            completeness.conservativeResult,
                            reason,
                            completeness.relevantEntityKeys,
                            finding3) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                // makeFinding cannot fail; continue as before.
            }
            findings_inout.push_back(finding3);
        }
    }

    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
