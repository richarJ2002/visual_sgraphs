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
#include <rclcpp/logging.hpp>
#include <utility>
#include <vector>

#include "Semantic/SemanticAxiomEvaluator.h"
#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

#include "private_functions.h"

namespace vs_graphs
{
namespace core
{

MissionHealthTopologyJsonStatus augmentMissionHealthTopologyJsonWithSemantics(
    nlohmann::json                            topologyJson_in,
    const semantic::SemanticReportCacheEntry &entry_in,
    bool                                      cacheAvailable_in,
    nlohmann::json                           &augmentedJson_out)
{
    topologyJson_in["schema"]                 = 2;
    topologyJson_in["semanticCacheAvailable"] = cacheAvailable_in;
    if (!cacheAvailable_in)
    {
        augmentedJson_out = topologyJson_in;
        return MissionHealthTopologyJsonStatus::
            MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS;
    }

    const std::int64_t ageMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - entry_in.updateInstant)
            .count();
    topologyJson_in["semanticCacheAgeMs"]     = ageMilliseconds;
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
        std::string axiomCodeName2{};
        if (semantic::axiomCodeName(aggregate.axiomCode, axiomCodeName2) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomCodeName returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::string axiomResultName2{};
        if (semantic::axiomResultName(aggregate.result, axiomResultName2) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomResultName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::string axiomClassName2{};
        if (semantic::axiomClassName(aggregate.classification,
                                     axiomClassName2) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomClassName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        aggregatesJson.push_back(
            {{"axiomCode", axiomCodeName2},
             {"result", axiomResultName2},
             {"classification", axiomClassName2},
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
            nlohmann::json json2{};
            if (finiteAwareDoubleToJson(*finding.evidence.numericValue,
                                        json2) !=
                MissionHealthTopologyJsonStatus::
                    MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: finiteAwareDoubleToJson returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            evidenceJson["numericValue"] = json2;
        }
        nlohmann::json json3{};
        if (entityKeysToJson(finding.involvedKeys, json3) !=
            MissionHealthTopologyJsonStatus::
                MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: entityKeysToJson returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::string axiomCodeName3{};
        if (semantic::axiomCodeName(finding.axiomCode, axiomCodeName3) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomCodeName returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::string reasonCodeName2{};
        if (semantic::reasonCodeName(finding.reasonCode, reasonCodeName2) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reasonCodeName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::string axiomClassName3{};
        if (semantic::axiomClassName(finding.classification, axiomClassName3) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomClassName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        violationsJson.push_back({{"findingId", finding.id},
                                  {"axiomCode", axiomCodeName3},
                                  {"reasonCode", reasonCodeName2},
                                  {"severity", axiomClassName3},
                                  {"involvedKeys", json3},
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
            std::string reasonCodeName3{};
            if (semantic::reasonCodeName(reason, reasonCodeName3) !=
                semantic::SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: reasonCodeName returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            reasonsJson.push_back(reasonCodeName3);
        }

        nlohmann::json json4{};
        if (entityKeysToJson(completeness.relevantEntityKeys, json4) !=
            MissionHealthTopologyJsonStatus::
                MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: entityKeysToJson returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::string axiomResultName3{};
        if (semantic::axiomResultName(completeness.conservativeResult,
                                      axiomResultName3) !=
            semantic::SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomResultName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        completenessJson.push_back({{"mapId", completeness.mapId},
                                    {"isComplete", completeness.isComplete},
                                    {"conservativeResult", axiomResultName3},
                                    {"reasons", std::move(reasonsJson)},
                                    {"relevantEntityKeys", json4}});
    }
    topologyJson_in["semanticMapCompleteness"] = std::move(completenessJson);
    nlohmann::json json5{};
    if (axiomCapabilitiesToJson(json5) !=
        MissionHealthTopologyJsonStatus::
            MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: axiomCapabilitiesToJson returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    topologyJson_in["semanticAxiomCapabilities"] = json5;

    augmentedJson_out = topologyJson_in;
    return MissionHealthTopologyJsonStatus::
        MISSION_HEALTH_TOPOLOGY_JSON_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
