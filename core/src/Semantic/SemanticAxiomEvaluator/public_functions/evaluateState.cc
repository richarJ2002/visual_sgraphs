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
 * @file            evaluateState.cc
 *
 * @brief           Implements evaluateState(), declared in
 *                  public_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/public_functions.h"

#include <utility>

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateState(const SemanticGraphSnapshot &snapshot_in,
                  AxiomEvaluationReport       &report_out)
{
    std::vector<Finding> findings;

    if (evaluateAxFrame01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxFrame01 cannot fail; continue as before.
    }
    if (evaluateAxWall01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxWall01 cannot fail; continue as before.
    }
    if (evaluateAxWall02(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxWall02 cannot fail; continue as before.
    }
    if (evaluateAxWall03(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxWall03 cannot fail; continue as before.
    }
    if (evaluateAxPass01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxPass01 cannot fail; continue as before.
    }
    if (evaluateAxPass02(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxPass02 cannot fail; continue as before.
    }
    if (evaluateAxPass03(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxPass03 cannot fail; continue as before.
    }
    if (evaluateAxPass04(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxPass04 cannot fail; continue as before.
    }
    if (evaluateAxRoom01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxRoom01 cannot fail; continue as before.
    }
    if (evaluateAxRoom02(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxRoom02 cannot fail; continue as before.
    }
    if (evaluateAxBound01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxBound01 cannot fail; continue as before.
    }
    if (evaluateAxFloor01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxFloor01 cannot fail; continue as before.
    }
    if (evaluateAxLife01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxLife01 cannot fail; continue as before.
    }
    if (evaluateAxTxn01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxTxn01 cannot fail; continue as before.
    }

    /* AX-COMP-01 is derived from the same completeness calculation
     * evaluateMapCompleteness() exposes on its own; computed here, not
     * inside computeConservativeMapCompleteness(), to keep that function
     * free of any dependency back on this one (see its own Doxygen). */
    std::vector<MapCompletenessResult> completeness{};
    if (evaluateMapCompleteness(snapshot_in, completeness) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateMapCompleteness cannot fail; continue as before.
    }
    if (evaluateAxComp01(completeness, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxComp01 cannot fail; continue as before.
    }

    if (evaluateAxMerge01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // evaluateAxMerge01 cannot fail; continue as before.
    }

    if (sortFindings(findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // sortFindings cannot fail; continue as before.
    }

    AxiomEvaluationReport             report;
    std::vector<AggregateAxiomResult> aggregateResults{};
    if (aggregateFindings(findings, aggregateResults) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // aggregateFindings cannot fail; continue as before.
    }
    report.aggregates = aggregateResults;
    report.findings   = std::move(findings);
    report_out        = report;
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
