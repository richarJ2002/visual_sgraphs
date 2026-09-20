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
 * @file            evaluateTransition.cc
 *
 * @brief           Implements evaluateTransition(), declared in
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

AxiomEvaluationReport
    evaluateTransition(const SemanticGraphSnapshot       &before_in,
                       const SemanticGraphSnapshot       &after_in,
                       const TransitionEvaluationContext &context_in)
{
    (void)before_in;
    (void)context_in;

    /* Every static axiom's meaning is unchanged by a transition, so reuse
     * evaluateState(after_in)'s own findings for everything except
     * AX-FRAME-01 and AX-TXN-01, whose evaluateState() placeholder reasons
     * specifically say "a static snapshot cannot prove this" -- rebuilt
     * below with the transition-specific "not yet implemented" reason
     * instead, since this slice does not yet implement real detection for
     * either. AX-MERGE-01 keeps its single reason either way: it is always
     * MERGE_PRESERVATION_NOT_YET_IMPLEMENTED regardless of evaluation
     * mode. */
    AxiomEvaluationReport afterStateReport = evaluateState(after_in);

    std::vector<Finding> findings;
    findings.reserve(afterStateReport.findings.size());
    for (Finding &finding : afterStateReport.findings)
    {
        if (finding.axiomCode == AxiomCode::AX_FRAME_01 ||
            finding.axiomCode == AxiomCode::AX_TXN_01)
        {
            continue;
        }
        findings.push_back(std::move(finding));
    }

    findings.push_back(
        makeFinding(AxiomCode::AX_FRAME_01,
                    AxiomResult::UNKNOWN,
                    ReasonCode::FRAME_EQUIVARIANCE_NOT_YET_IMPLEMENTED,
                    {}));
    findings.push_back(
        makeFinding(AxiomCode::AX_TXN_01,
                    AxiomResult::UNKNOWN,
                    ReasonCode::TRANSACTION_POSTCONDITION_NOT_YET_IMPLEMENTED,
                    {}));

    sortFindings(findings);

    AxiomEvaluationReport report;
    report.aggregates = aggregateFindings(findings);
    report.findings   = std::move(findings);
    return report;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
