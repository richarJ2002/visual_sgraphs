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
 * @file            evaluateOnePassageProvenance.cc
 *
 * @brief           Implements evaluateOnePassageProvenance(), declared in
 *                  private_functions.h.
 *
 *                  Shared AX-PASS-01 leaf used by evaluateAxPass01() and
 *                  computeConservativeMapCompleteness() so both consume
 *                  the identical leaf rather than the completeness path
 *                  re-deriving its own hand-written passable() check.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateOnePassageProvenance(const PassageRecord  &passage_in,
                                 std::vector<Finding> &findings_inout)
{
    if (!passage_in.isPassable)
    {
        Finding finding{};
        if (makeFinding(AxiomCode::AX_PASS_01,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_PROVENANCE_NOT_PASSABLE,
                        {passage_in.key},
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
    Finding finding2{};
    if (makeFinding(AxiomCode::AX_PASS_01,
                    AxiomResult::UNKNOWN,
                    ReasonCode::PASSAGE_PROVENANCE_FULL_CHAIN_UNVERIFIABLE,
                    {passage_in.key},
                    finding2) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeFinding returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    findings_inout.push_back(finding2);

    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
