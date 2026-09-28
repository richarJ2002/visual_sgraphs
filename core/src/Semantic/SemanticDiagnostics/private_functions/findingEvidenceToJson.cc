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
 * @file            findingEvidenceToJson.cc
 *
 * @brief           Implements findingEvidenceToJson(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticDiagnostics/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticDiagnosticsStatus
    findingEvidenceToJson(const FindingEvidence &evidence_in,
                          nlohmann::json        &json_out)
{
    nlohmann::json json;
    if (evidence_in.observedCount.has_value())
    {
        json["observedCount"] = *evidence_in.observedCount;
    }
    if (evidence_in.expectedCount.has_value())
    {
        json["expectedCount"] = *evidence_in.expectedCount;
    }
    if (evidence_in.numericValue.has_value())
    {
        json["numericValue"] =
            serializeFiniteAwareDouble(*evidence_in.numericValue);
    }
    json_out = json;
    return SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
