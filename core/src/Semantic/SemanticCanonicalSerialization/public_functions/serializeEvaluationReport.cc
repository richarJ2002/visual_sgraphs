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
 * @file            serializeEvaluationReport.cc
 *
 * @brief           Implements serializeEvaluationReport(), declared in
 *                  public_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/public_functions.h"

#include <algorithm>
#include <utility>

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json serializeEvaluationReport(const AxiomEvaluationReport &report_in)
{
    nlohmann::json json;
    json["schema"] = SEMANTIC_EVALUATION_REPORT_SCHEMA_VERSION;

    /* isFindingLessTotalOrder() compares every field this module emits, not
     * only id: two findings can share a deterministic id (a genuine
     * axiomCode/reasonCode/involvedKeys collision) while still differing in
     * evidence, so an id-only sort left their relative order dependent on
     * input permutation. */
    std::vector<Finding> findings = report_in.findings;
    std::sort(findings.begin(), findings.end(), &isFindingLessTotalOrder);
    nlohmann::json findingsJson = nlohmann::json::array();
    for (const Finding &finding : findings)
    {
        findingsJson.push_back(serializeFinding(finding));
    }
    json["findings"] = std::move(findingsJson);

    /* isAggregateAxiomResultLessTotalOrder() compares every field this
     * module emits, not only axiomCode, so two aggregates sharing axiomCode
     * (a genuine collision) still serialize in one fixed order. */
    std::vector<AggregateAxiomResult> aggregates = report_in.aggregates;
    std::sort(aggregates.begin(),
              aggregates.end(),
              &isAggregateAxiomResultLessTotalOrder);
    nlohmann::json aggregatesJson = nlohmann::json::array();
    for (const AggregateAxiomResult &aggregate : aggregates)
    {
        aggregatesJson.push_back(serializeAggregateAxiomResult(aggregate));
    }
    json["aggregates"] = std::move(aggregatesJson);

    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
