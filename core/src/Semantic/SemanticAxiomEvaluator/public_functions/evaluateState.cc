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
    evaluateState(const SemanticGraphSnapshot &snapshot_in,
                  AxiomEvaluationReport       &report_out)
{
    std::vector<Finding> findings;

    if (evaluateAxFrame01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxFrame01 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxWall01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxWall01 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxWall02(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxWall02 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxWall03(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxWall03 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxPass01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxPass01 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxPass02(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxPass02 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxPass03(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxPass03 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxPass04(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxPass04 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxRoom01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxRoom01 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxRoom02(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxRoom02 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxBound01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxBound01 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxFloor01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxFloor01 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxLife01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxLife01 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxTxn01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxTxn01 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* AX-COMP-01 is derived from the same completeness calculation
     * evaluateMapCompleteness() exposes on its own; computed here, not
     * inside computeConservativeMapCompleteness(), to keep that function
     * free of any dependency back on this one (see its own Doxygen). */
    std::vector<MapCompletenessResult> completeness{};
    if (evaluateMapCompleteness(snapshot_in, completeness) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateMapCompleteness returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (evaluateAxComp01(completeness, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxComp01 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    if (evaluateAxMerge01(snapshot_in, findings) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateAxMerge01 returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

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
