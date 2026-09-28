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
 * @file            serializeAggregateAxiomResult.cc
 *
 * @brief           Implements serializeAggregateAxiomResult(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json
    serializeAggregateAxiomResult(const AggregateAxiomResult &value_in)
{
    nlohmann::json json;
    json["axiomCode"] = static_cast<unsigned int>(value_in.axiomCode);
    std::string axiomCodeName2{};
    if (axiomCodeName(value_in.axiomCode, axiomCodeName2) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // axiomCodeName cannot fail; continue as before.
    }
    json["axiomCodeName"] = axiomCodeName2;
    json["result"]        = static_cast<unsigned int>(value_in.result);
    std::string axiomResultName2{};
    if (axiomResultName(value_in.result, axiomResultName2) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // axiomResultName cannot fail; continue as before.
    }
    json["resultName"]     = axiomResultName2;
    json["classification"] = static_cast<unsigned int>(value_in.classification);
    std::string axiomClassName2{};
    if (axiomClassName(value_in.classification, axiomClassName2) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // axiomClassName cannot fail; continue as before.
    }
    json["classificationName"]       = axiomClassName2;
    json["contributingFindingCount"] = value_in.contributingFindingCount;
    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
