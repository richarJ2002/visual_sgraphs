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
 * @file            violationDetailToJson.cc
 *
 * @brief           Implements violationDetailToJson(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticDiagnostics/private_functions.h"

#include <string>

#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json violationDetailToJson(const Finding &finding_in,
                                     const char    *p_transition_in)
{
    /* WARN for a newly appeared hard failure (the operator-relevant case);
     * INFO for a resolution or any other transition. This never emits DEBUG
     * (no opt-in proposal trace exists yet) or SG_DECISION (no real opt-in
     * decision source exists yet). */
    const bool isNewlyAppearedHardFailure =
        std::string(p_transition_in) == "appeared" &&
        finding_in.classification == AxiomClass::HARD;

    nlohmann::json json;
    json["level"]        = isNewlyAppearedHardFailure ? "WARN" : "INFO";
    json["findingId"]    = finding_in.id;
    json["transition"]   = p_transition_in;
    json["axiomCode"]    = axiomCodeName(finding_in.axiomCode);
    json["result"]       = axiomResultName(finding_in.result);
    json["severity"]     = axiomClassName(finding_in.classification);
    json["reasonCode"]   = reasonCodeName(finding_in.reasonCode);
    json["involvedKeys"] = entityKeysToJson(finding_in.involvedKeys);
    json["evidence"]     = findingEvidenceToJson(finding_in.evidence);
    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
