/**
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
 * @file            private_functions.h
 *
 * @brief           Declares the internal helpers used by
 *                  buildSemanticDiagnosticUpdate() (CPP_CODING_STANDARD.md
 *                  Section 5.4). Not exported or included by a public
 *                  header. Deliberately does not include
 *                  SemanticCanonicalSerialization's private_functions.h
 *                  (serializeDouble() is not part of that module's public
 *                  boundary) -- serializeFiniteAwareDouble() below is this
 *                  module's own small, self-contained equivalent, so
 *                  neither module depends on the other's private header.
 */

#ifndef SEMANTIC_DIAGNOSTICS_PRIVATE_FUNCTIONS_H
#define SEMANTIC_DIAGNOSTICS_PRIVATE_FUNCTIONS_H

#include <vector>

#include "Thirdparty/nlohmann/json.hpp"

#include "Semantic/SemanticAxiomEvaluator/objects.h"
#include "Semantic/SemanticGraphSnapshot/objects/EntityKey.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*! @brief Serializes \p value_in as a JSON number for every finite value, or
 *  as one of the explicit string sentinels "NaN"/"Infinity"/"-Infinity" for
 *  a non-finite value, so a canonical diagnostic record can never contain
 *  the JSON null that nlohmann::json's default numeric formatting would
 *  otherwise silently substitute for a non-finite double. */
nlohmann::json serializeFiniteAwareDouble(double value_in);

/*! @brief Serializes one Finding's evidence: observedCount/expectedCount
 *  present only when set; numericValue present only when set, via
 *  serializeFiniteAwareDouble(). */
nlohmann::json findingEvidenceToJson(const FindingEvidence &evidence_in);

/*! @brief Serializes \p keys_in as a sorted (ascending, from a private
 *  copy) JSON array of {kind, mapId, entityId} objects. Never mutates \p
 *  keys_in. */
nlohmann::json entityKeysToJson(const std::vector<EntityKey> &keys_in);

/*! @brief Serializes one FAIL Finding as a bounded SG_VIOLATION detail
 *  object for the given transition ("appeared", "changed", or "resolved"):
 *  level (WARN for a newly appeared HARD FAIL, INFO otherwise), findingId,
 *  transition, readable axiomCode/result/severity/reasonCode, sorted
 *  involvedKeys, and evidence. \p finding_in must have
 *  \c result == AxiomResult::FAIL -- see buildSemanticDiagnosticUpdate.cc,
 *  the only caller. */
nlohmann::json violationDetailToJson(const Finding &finding_in,
                                     const char    *transition_in);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_DIAGNOSTICS_PRIVATE_FUNCTIONS_H
