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
 * @file            buildSemanticDiagnosticUpdate.cc
 *
 * @brief           Implements buildSemanticDiagnosticUpdate(), declared in
 *                  public_functions.h.
 */

#include "Semantic/SemanticDiagnostics/public_functions.h"

#include <map>
#include <rclcpp/logging.hpp>
#include <string>

#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"
#include "Semantic/SemanticDiagnostics/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticDiagnosticsStatus buildSemanticDiagnosticUpdate(
    const SemanticReportCacheEntry &entry_in,
    SemanticDiagnosticState        &state_in_out,
    SemanticDiagnosticUpdate       &semanticDiagnosticUpdate_out)
{
    /* Only FAIL findings are violations. Filtering both maps to FAIL before
     * computing appeared/changed/resolved means: (1) the very first call,
     * with no prior state, never reports a PASS/UNKNOWN finding as
     * "appeared"; (2) a finding that flips FAIL->PASS (a different
     * deterministic id, since id excludes result but includes reasonCode)
     * correctly reports as "resolved", never re-"appeared" as its new
     * PASS/UNKNOWN identity; and (3) UNKNOWN findings never produce a
     * violation detail, remaining visible only via the summary's
     * unknownCount/completeness fields. */
    std::map<std::string, const Finding *> currentFailuresById;
    for (const Finding &finding : entry_in.evaluationReport.findings)
    {
        if (finding.result == AxiomResult::FAIL)
        {
            currentFailuresById[finding.id] = &finding;
        }
    }
    std::map<std::string, const Finding *> previousFailuresById;
    if (state_in_out.lastLoggedEvaluationReport.has_value())
    {
        for (const Finding &finding :
             state_in_out.lastLoggedEvaluationReport->findings)
        {
            if (finding.result == AxiomResult::FAIL)
            {
                previousFailuresById[finding.id] = &finding;
            }
        }
    }

    std::vector<nlohmann::json> violationDetails;
    std::size_t                 totalTransitions = 0U;
    for (const auto &[id, p_currentFinding] : currentFailuresById)
    {
        const auto previousIterator = previousFailuresById.find(id);
        if (previousIterator == previousFailuresById.end())
        {
            ++totalTransitions;
            if (violationDetails.size() < kMaxViolationDetailsPerCycle)
            {
                nlohmann::json json2{};
                if (violationDetailToJson(*p_currentFinding,
                                          "appeared",
                                          json2) !=
                    SemanticDiagnosticsStatus::
                        SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: violationDetailToJson returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                violationDetails.push_back(json2);
            }
            continue;
        }
        const Finding &previousFinding = *previousIterator->second;
        if (previousFinding.reasonCode != p_currentFinding->reasonCode)
        {
            ++totalTransitions;
            if (violationDetails.size() < kMaxViolationDetailsPerCycle)
            {
                nlohmann::json json3{};
                if (violationDetailToJson(*p_currentFinding,
                                          "changed",
                                          json3) !=
                    SemanticDiagnosticsStatus::
                        SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: violationDetailToJson returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                violationDetails.push_back(json3);
            }
        }
    }
    for (const auto &[id, p_previousFinding] : previousFailuresById)
    {
        if (currentFailuresById.count(id) == 0U)
        {
            ++totalTransitions;
            if (violationDetails.size() < kMaxViolationDetailsPerCycle)
            {
                nlohmann::json json4{};
                if (violationDetailToJson(*p_previousFinding,
                                          "resolved",
                                          json4) !=
                    SemanticDiagnosticsStatus::
                        SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: violationDetailToJson returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                violationDetails.push_back(json4);
            }
        }
    }

    const bool discreteStateChanged =
        !state_in_out.lastLoggedTopologyDigest.has_value() ||
        *state_in_out.lastLoggedTopologyDigest !=
            entry_in.canonicalTopologyDigest ||
        totalTransitions > 0U;

    /* cyclesSinceLastDiagnosticSummary counts this call: it becomes 1 on
     * the very first call and on the call immediately after any emission,
     * and increments by one on every subsequent non-emitting call. A
     * heartbeat is due once it reaches kDiagnosticHeartbeatCycles, so with
     * a call on every semantic cycle and no discrete changes: cycle 1
     * emits the initial summary (nothing to compare against yet); cycles 2
     * through kDiagnosticHeartbeatCycles-1 emit nothing; cycle
     * kDiagnosticHeartbeatCycles emits the heartbeat. */
    ++state_in_out.cyclesSinceLastDiagnosticSummary;
    const bool emitHeartbeat = !discreteStateChanged &&
                               state_in_out.cyclesSinceLastDiagnosticSummary >=
                                   kDiagnosticHeartbeatCycles;

    SemanticDiagnosticUpdate update;
    update.shouldEmit = discreteStateChanged || emitHeartbeat;
    if (!update.shouldEmit)
    {
        semanticDiagnosticUpdate_out = update;
        return SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS;
    }
    update.violationDetails = std::move(violationDetails);

    std::size_t passCount = 0U, failCount = 0U, unknownCount = 0U,
                hardCount = 0U, unresolvedCount = 0U;
    std::map<std::string, unsigned int> perCodeCounts;
    for (const AggregateAxiomResult &aggregate :
         entry_in.evaluationReport.aggregates)
    {
        switch (aggregate.result)
        {
        case AxiomResult::PASS:
            ++passCount;
            break;
        case AxiomResult::FAIL:
            ++failCount;
            break;
        case AxiomResult::UNKNOWN:
            ++unknownCount;
            break;
        }
        if (aggregate.classification == AxiomClass::HARD)
        {
            ++hardCount;
            if (aggregate.result != AxiomResult::PASS)
            {
                ++unresolvedCount;
            }
        }
        std::string axiomCodeName2{};
        if (axiomCodeName(aggregate.axiomCode, axiomCodeName2) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomCodeName returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        perCodeCounts[axiomCodeName2] =
            static_cast<unsigned int>(aggregate.contributingFindingCount);
    }

    nlohmann::json completenessJson = nlohmann::json::array();
    for (const MapCompletenessResult &completeness :
         entry_in.completenessResults)
    {
        std::string axiomResultName2{};
        if (axiomResultName(completeness.conservativeResult,
                            axiomResultName2) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: axiomResultName returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        completenessJson.push_back({{"mapId", completeness.mapId},
                                    {"isComplete", completeness.isComplete},
                                    {"conservativeResult", axiomResultName2}});
    }

    nlohmann::json summary;
    summary["schema"]         = kSemanticDiagnosticSchemaVersion;
    summary["level"]          = "INFO";
    summary["eventType"]      = discreteStateChanged ? "summary" : "heartbeat";
    summary["semanticCycle"]  = entry_in.semanticCycle;
    summary["updateSequence"] = entry_in.updateSequence;
    if (entry_in.currentMapId.has_value())
    {
        summary["currentMapId"] = *entry_in.currentMapId;
    }
    if (entry_in.mapRevision.has_value())
    {
        summary["mapRevision"] = *entry_in.mapRevision;
    }
    summary["geometryRevision"]        = entry_in.geometryRevision;
    summary["canonicalTopologyDigest"] = entry_in.canonicalTopologyDigest;
    summary["canonicalFullGeometryDigest"] =
        entry_in.canonicalFullGeometryDigest;
    summary["passCount"]                        = passCount;
    summary["failCount"]                        = failCount;
    summary["unknownCount"]                     = unknownCount;
    summary["hardCount"]                        = hardCount;
    summary["unresolvedCount"]                  = unresolvedCount;
    summary["perCodeContributingFindingCounts"] = perCodeCounts;
    summary["completeness"]                     = std::move(completenessJson);
    summary["evaluationDurationMs"]  = entry_in.evaluationDuration.count();
    summary["cacheAvailable"]        = true;
    summary["emittedViolationCount"] = update.violationDetails.size();
    summary["omittedViolationCount"] =
        totalTransitions - update.violationDetails.size();
    update.summary = std::move(summary);

    state_in_out.cyclesSinceLastDiagnosticSummary = 1U;
    state_in_out.lastLoggedEvaluationReport       = entry_in.evaluationReport;
    state_in_out.lastLoggedTopologyDigest = entry_in.canonicalTopologyDigest;
    state_in_out.lastLoggedFullGeometryDigest =
        entry_in.canonicalFullGeometryDigest;

    semanticDiagnosticUpdate_out = update;
    return SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
