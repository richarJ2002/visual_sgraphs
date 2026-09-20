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
 * @file            MissionHealthTopologyJson.cc
 *
 * @brief           Implements augmentMissionHealthTopologyJsonWithSemantics(),
 *                   declared in MissionHealthTopologyJson.h.
 */

#include "MissionHealthTopologyJson.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include "Semantic/SemanticAxiomEvaluator.h"
#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

namespace vs_graphs
{
namespace core
{

namespace
{
nlohmann::json entityKeysToJson(std::vector<semantic::EntityKey> keys_in)
{
    std::sort(keys_in.begin(), keys_in.end());
    nlohmann::json json = nlohmann::json::array();
    for (const semantic::EntityKey &key : keys_in)
    {
        json.push_back({{"kind", semantic::entityKindName(key.kind)},
                        {"mapId", key.mapId},
                        {"entityId", key.entityId}});
    }
    return json;
}

/* Guards against nlohmann::json's default numeric formatting silently
 * collapsing a non-finite double to JSON null, matching
 * SemanticDiagnostics/private_functions.h's serializeFiniteAwareDouble(). */
nlohmann::json finiteAwareDoubleToJson(double value_in)
{
    if (std::isnan(value_in))
    {
        return "NaN";
    }
    if (std::isinf(value_in))
    {
        return value_in > 0.0 ? "Infinity" : "-Infinity";
    }
    return value_in;
}

/* Sorted, readable-name projection of computeAxiomCapabilityTable(): fixed
 * and snapshot-independent, so it never needs entry_in. */
nlohmann::json axiomCapabilitiesToJson()
{
    std::vector<semantic::AxiomCapabilityEntry> table =
        semantic::computeAxiomCapabilityTable();
    std::sort(table.begin(),
              table.end(),
              [](const semantic::AxiomCapabilityEntry &lhs_in,
                 const semantic::AxiomCapabilityEntry &rhs_in)
              { return lhs_in.axiomCode < rhs_in.axiomCode; });

    nlohmann::json json = nlohmann::json::array();
    for (const semantic::AxiomCapabilityEntry &row : table)
    {
        json.push_back(
            {{"axiomCode", semantic::axiomCodeName(row.axiomCode)},
             {"classification", semantic::axiomClassName(row.classification)},
             {"capability", semantic::capabilityLevelName(row.capability)},
             {"missingProofOwner",
              semantic::missingProofOwnerName(row.owner)}});
    }
    return json;
}
} // namespace

nlohmann::json augmentMissionHealthTopologyJsonWithSemantics(
    nlohmann::json                            topologyJson_in,
    const semantic::SemanticReportCacheEntry &entry_in,
    bool                                      cacheAvailable_in)
{
    topologyJson_in["schema"]                 = 2;
    topologyJson_in["semanticCacheAvailable"] = cacheAvailable_in;
    if (!cacheAvailable_in)
    {
        return topologyJson_in;
    }

    const std::int64_t ageMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - entry_in.updateInstant)
            .count();
    topologyJson_in["semanticCacheAgeMs"]     = ageMs;
    topologyJson_in["semanticCycle"]          = entry_in.semanticCycle;
    topologyJson_in["semanticUpdateSequence"] = entry_in.updateSequence;
    if (entry_in.currentMapId.has_value())
    {
        topologyJson_in["semanticCurrentMapId"] = *entry_in.currentMapId;
    }
    if (entry_in.mapRevision.has_value())
    {
        topologyJson_in["semanticMapRevision"] = *entry_in.mapRevision;
    }
    topologyJson_in["semanticGeometryRevision"] = entry_in.geometryRevision;
    topologyJson_in["semanticTopologyDigest"] =
        entry_in.canonicalTopologyDigest;
    topologyJson_in["semanticFullGeometryDigest"] =
        entry_in.canonicalFullGeometryDigest;

    /* AxiomEvaluationReport::aggregates is already sorted deterministically
     * by evaluateState()/evaluateTransition(); no re-sort needed here. */
    nlohmann::json aggregatesJson = nlohmann::json::array();
    for (const semantic::AggregateAxiomResult &aggregate :
         entry_in.evaluationReport.aggregates)
    {
        aggregatesJson.push_back(
            {{"axiomCode", semantic::axiomCodeName(aggregate.axiomCode)},
             {"result", semantic::axiomResultName(aggregate.result)},
             {"classification",
              semantic::axiomClassName(aggregate.classification)},
             {"contributingFindingCount", aggregate.contributingFindingCount}});
    }
    topologyJson_in["semanticAggregates"] = std::move(aggregatesJson);

    /* AxiomEvaluationReport::findings is already sorted deterministically;
     * only the FAIL subset is a "current violation". */
    nlohmann::json violationsJson = nlohmann::json::array();
    for (const semantic::Finding &finding : entry_in.evaluationReport.findings)
    {
        if (finding.result != semantic::AxiomResult::FAIL)
        {
            continue;
        }
        nlohmann::json evidenceJson;
        if (finding.evidence.observedCount.has_value())
        {
            evidenceJson["observedCount"] = *finding.evidence.observedCount;
        }
        if (finding.evidence.expectedCount.has_value())
        {
            evidenceJson["expectedCount"] = *finding.evidence.expectedCount;
        }
        if (finding.evidence.numericValue.has_value())
        {
            evidenceJson["numericValue"] =
                finiteAwareDoubleToJson(*finding.evidence.numericValue);
        }
        violationsJson.push_back(
            {{"findingId", finding.id},
             {"axiomCode", semantic::axiomCodeName(finding.axiomCode)},
             {"reasonCode", semantic::reasonCodeName(finding.reasonCode)},
             {"severity", semantic::axiomClassName(finding.classification)},
             {"involvedKeys", entityKeysToJson(finding.involvedKeys)},
             {"evidence", evidenceJson}});
    }
    topologyJson_in["semanticViolations"] = std::move(violationsJson);

    nlohmann::json completenessJson = nlohmann::json::array();
    for (const semantic::MapCompletenessResult &completeness :
         entry_in.completenessResults)
    {
        std::vector<semantic::ReasonCode> sortedReasons = completeness.reasons;
        std::sort(sortedReasons.begin(), sortedReasons.end());
        nlohmann::json reasonsJson = nlohmann::json::array();
        for (const semantic::ReasonCode reason : sortedReasons)
        {
            reasonsJson.push_back(semantic::reasonCodeName(reason));
        }

        completenessJson.push_back(
            {{"mapId", completeness.mapId},
             {"isComplete", completeness.isComplete},
             {"conservativeResult",
              semantic::axiomResultName(completeness.conservativeResult)},
             {"reasons", std::move(reasonsJson)},
             {"relevantEntityKeys",
              entityKeysToJson(completeness.relevantEntityKeys)}});
    }
    topologyJson_in["semanticMapCompleteness"]   = std::move(completenessJson);
    topologyJson_in["semanticAxiomCapabilities"] = axiomCapabilitiesToJson();

    return topologyJson_in;
}

} // namespace core
} // namespace vs_graphs
