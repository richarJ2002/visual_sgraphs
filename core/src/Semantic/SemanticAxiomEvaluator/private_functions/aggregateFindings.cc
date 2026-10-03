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
 * @file            aggregateFindings.cc
 *
 * @brief           Implements aggregateFindings(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

namespace
{
/*!
 * @brief           Every axiom code, in the fixed presentation order the
 *                  aggregate report and capability table both use.
 */
constexpr AxiomCode ALL_AXIOM_CODES[16] = {AxiomCode::AX_FRAME_01,
                                           AxiomCode::AX_WALL_01,
                                           AxiomCode::AX_WALL_02,
                                           AxiomCode::AX_WALL_03,
                                           AxiomCode::AX_PASS_01,
                                           AxiomCode::AX_PASS_02,
                                           AxiomCode::AX_PASS_03,
                                           AxiomCode::AX_PASS_04,
                                           AxiomCode::AX_ROOM_01,
                                           AxiomCode::AX_ROOM_02,
                                           AxiomCode::AX_BOUND_01,
                                           AxiomCode::AX_FLOOR_01,
                                           AxiomCode::AX_LIFE_01,
                                           AxiomCode::AX_TXN_01,
                                           AxiomCode::AX_COMP_01,
                                           AxiomCode::AX_MERGE_01};
} // namespace

SemanticAxiomEvaluatorStatus
    aggregateFindings(const std::vector<Finding>        &findings_in,
                      std::vector<AggregateAxiomResult> &aggregateResults_out)
{
    std::vector<AggregateAxiomResult> aggregates;
    aggregates.reserve(16U);

    for (const AxiomCode axiomCode : ALL_AXIOM_CODES)
    {
        AggregateAxiomResult aggregate;
        aggregate.axiomCode = axiomCode;
        AxiomClass axiomClass{};
        if (axiomClassFor(axiomCode, axiomClass) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomClassFor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        aggregate.classification           = axiomClass;
        aggregate.result                   = AxiomResult::PASS;
        aggregate.contributingFindingCount = 0U;

        bool sawFail    = false;
        bool sawUnknown = false;
        for (const Finding &finding : findings_in)
        {
            if (finding.axiomCode != axiomCode)
            {
                continue;
            }
            aggregate.contributingFindingCount++;
            if (finding.result == AxiomResult::FAIL)
            {
                sawFail = true;
            }
            else if (finding.result == AxiomResult::UNKNOWN)
            {
                sawUnknown = true;
            }
        }

        /* FAIL > UNKNOWN > PASS; a vacuously empty finding set for this
         * code (contributingFindingCount == 0) keeps the PASS default set
         * above, which is correct only for an axiom whose contract permits
         * a vacuous witness -- every per-axiom evaluator in this module is
         * documented to always emit at least one Finding, so this case is
         * not expected to occur in practice, but is handled safely either
         * way rather than by an unchecked assumption. */
        if (sawFail)
        {
            aggregate.result = AxiomResult::FAIL;
        }
        else if (sawUnknown)
        {
            aggregate.result = AxiomResult::UNKNOWN;
        }

        aggregates.push_back(aggregate);
    }

    aggregateResults_out = aggregates;
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
