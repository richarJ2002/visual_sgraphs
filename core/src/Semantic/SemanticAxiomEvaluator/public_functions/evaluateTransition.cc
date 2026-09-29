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

#include <rclcpp/logging.hpp>
#include <utility>

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateTransition(const SemanticGraphSnapshot       &before_in,
                       const SemanticGraphSnapshot       &after_in,
                       const TransitionEvaluationContext &context_in,
                       AxiomEvaluationReport             &report_out)
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
    AxiomEvaluationReport afterStateReport{};
    if (evaluateState(after_in, afterStateReport) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateState returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

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

    Finding finding2{};
    if (makeFinding(AxiomCode::AX_FRAME_01,
                    AxiomResult::UNKNOWN,
                    ReasonCode::FRAME_EQUIVARIANCE_NOT_YET_IMPLEMENTED,
                    {},
                    finding2) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeFinding returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    findings.push_back(finding2);
    Finding finding3{};
    if (makeFinding(AxiomCode::AX_TXN_01,
                    AxiomResult::UNKNOWN,
                    ReasonCode::TRANSACTION_POSTCONDITION_NOT_YET_IMPLEMENTED,
                    {},
                    finding3) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeFinding returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    findings.push_back(finding3);

    if (sortFindings(findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: sortFindings returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    AxiomEvaluationReport             report;
    std::vector<AggregateAxiomResult> aggregateResults{};
    if (aggregateFindings(findings, aggregateResults) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: aggregateFindings returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
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
