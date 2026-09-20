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
    json["axiomCode"]      = static_cast<unsigned int>(value_in.axiomCode);
    json["axiomCodeName"]  = axiomCodeName(value_in.axiomCode);
    json["result"]         = static_cast<unsigned int>(value_in.result);
    json["resultName"]     = axiomResultName(value_in.result);
    json["classification"] = static_cast<unsigned int>(value_in.classification);
    json["classificationName"]       = axiomClassName(value_in.classification);
    json["contributingFindingCount"] = value_in.contributingFindingCount;
    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
