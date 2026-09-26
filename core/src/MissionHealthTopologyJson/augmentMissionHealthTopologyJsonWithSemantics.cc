/*!
 * @file         augmentMissionHealthTopologyJsonWithSemantics.cc
 *
 * @brief        Implements augmentMissionHealthTopologyJsonWithSemantics
 *               declared in MissionHealthTopologyJson.h.
 */

#include "MissionHealthTopologyJson.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "Semantic/SemanticAxiomEvaluator.h"
#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

#include "private_functions.h"

namespace vs_graphs
{
namespace core
{

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
