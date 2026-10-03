/*!
 * @file            test_SemanticDiagnostics.cpp
 *
 * @brief           Unit tests for semantic diagnostics (SemanticDiagnostics).
 */

/*
 * Focused, ROS/Gazebo-free tests for the pure
 * buildSemanticDiagnosticUpdate() builder extracted from
 * SemanticsManager::logSemanticDiagnostics().
 */

#include "Semantic/SemanticDiagnostics.h"

#include <algorithm>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
namespace
{

Finding makeFailFinding(const std::string &id_in, ReasonCode reasonCode_in)
{
    Finding finding;
    finding.id             = id_in;
    finding.axiomCode      = AxiomCode::AX_WALL_01;
    finding.result         = AxiomResult::FAIL;
    finding.classification = AxiomClass::HARD;
    finding.reasonCode     = reasonCode_in;
    return finding;
}

Finding makePassFinding(const std::string &id_in)
{
    Finding finding;
    finding.id             = id_in;
    finding.axiomCode      = AxiomCode::AX_WALL_01;
    finding.result         = AxiomResult::PASS;
    finding.classification = AxiomClass::HARD;
    finding.reasonCode     = ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER;
    return finding;
}

Finding makeUnknownFinding(const std::string &id_in)
{
    Finding finding;
    finding.id             = id_in;
    finding.axiomCode      = AxiomCode::AX_WALL_01;
    finding.result         = AxiomResult::UNKNOWN;
    finding.classification = AxiomClass::HARD;
    finding.reasonCode =
        ReasonCode::WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE;
    return finding;
}

SemanticReportCacheEntry makeEntry(std::uint64_t               cycle_in,
                                   const std::vector<Finding> &findings_in,
                                   const std::string &topologyDigest_in)
{
    SemanticReportCacheEntry entry;
    entry.semanticCycle               = cycle_in;
    entry.updateSequence              = cycle_in;
    entry.evaluationReport.findings   = findings_in;
    entry.canonicalTopologyDigest     = topologyDigest_in;
    entry.canonicalFullGeometryDigest = topologyDigest_in;
    return entry;
}

} // namespace

/*!
 * @brief        Checks that the first cycle emits a summary whose only
 *               violation detail is the failing finding, with pass and unknown
 *               findings left out.
 */
TEST(SemanticDiagnostics, FirstCycleEmitsSummaryAndOnlyFailAppeared)
{
    SemanticDiagnosticState        state;
    const SemanticReportCacheEntry entry =
        makeEntry(1U,
                  {makeFailFinding("f1", ReasonCode::WALL_OWNERSHIP_OWNER_BAD),
                   makePassFinding("p1"),
                   makeUnknownFinding("u1")},
                  "digest-1");

    SemanticDiagnosticUpdate update{};
    ASSERT_EQ((buildSemanticDiagnosticUpdate(entry, state, update)),
              SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);

    ASSERT_TRUE(update.shouldEmit);
    EXPECT_EQ(update.summary["eventType"], "summary");
    ASSERT_EQ(update.violationDetails.size(), 1U);
    EXPECT_EQ(update.violationDetails[0]["findingId"], "f1");
    EXPECT_EQ(update.violationDetails[0]["transition"], "appeared");
    for (const nlohmann::json &detail : update.violationDetails)
    {
        EXPECT_NE(detail["findingId"], "p1");
        EXPECT_NE(detail["findingId"], "u1");
    }
    EXPECT_EQ(update.summary["emittedViolationCount"], 1U);
    EXPECT_EQ(update.summary["omittedViolationCount"], 0U);
}

/*!
 * @brief        Checks that cycles 2 to 9 emit nothing when the findings and
 *               digest are unchanged since cycle 1.
 */
TEST(SemanticDiagnostics, UnchangedCycles2Through9EmitNothing)
{
    SemanticDiagnosticState    state;
    const std::vector<Finding> findings = {
        makeFailFinding("f1", ReasonCode::WALL_OWNERSHIP_OWNER_BAD)};

    SemanticDiagnosticUpdate semanticDiagnosticUpdate{};
    ASSERT_EQ((buildSemanticDiagnosticUpdate(makeEntry(1U, findings, "d"),
                                             state,
                                             semanticDiagnosticUpdate)),
              SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);
    ASSERT_TRUE(semanticDiagnosticUpdate.shouldEmit);

    for (std::uint64_t cycle = 2U; cycle <= 9U; ++cycle)
    {
        SemanticDiagnosticUpdate update{};
        ASSERT_EQ(
            (buildSemanticDiagnosticUpdate(makeEntry(cycle, findings, "d"),
                                           state,
                                           update)),
            SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);
        EXPECT_FALSE(update.shouldEmit) << "cycle " << cycle;
    }
}

/*!
 * @brief        Checks that an unchanged state emits a detail-free heartbeat on
 *               cycle 10 and again on cycle 19, and nothing in between.
 */
TEST(SemanticDiagnostics, HeartbeatEmittedExactlyOnCycle10)
{
    SemanticDiagnosticState    state;
    const std::vector<Finding> findings = {
        makeFailFinding("f1", ReasonCode::WALL_OWNERSHIP_OWNER_BAD)};

    SemanticDiagnosticUpdate semanticDiagnosticUpdate{};
    ASSERT_EQ((buildSemanticDiagnosticUpdate(makeEntry(1U, findings, "d"),
                                             state,
                                             semanticDiagnosticUpdate)),
              SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);
    ASSERT_TRUE(semanticDiagnosticUpdate.shouldEmit);
    for (std::uint64_t cycle = 2U; cycle <= 9U; ++cycle)
    {
        SemanticDiagnosticUpdate semanticDiagnosticUpdate2{};
        ASSERT_EQ(
            (buildSemanticDiagnosticUpdate(makeEntry(cycle, findings, "d"),
                                           state,
                                           semanticDiagnosticUpdate2)),
            SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);
        ASSERT_FALSE(semanticDiagnosticUpdate2.shouldEmit);
    }

    SemanticDiagnosticUpdate heartbeat{};
    ASSERT_EQ((buildSemanticDiagnosticUpdate(makeEntry(10U, findings, "d"),
                                             state,
                                             heartbeat)),
              SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);
    ASSERT_TRUE(heartbeat.shouldEmit);
    EXPECT_EQ(heartbeat.summary["eventType"], "heartbeat");
    EXPECT_TRUE(heartbeat.violationDetails.empty());

    /* And the cadence repeats: cycles 11-18 unchanged, cycle 19 heartbeats
     * again (9 cycles after cycle 10, matching the first cadence from cycle
     * 1). */
    for (std::uint64_t cycle = 11U; cycle <= 18U; ++cycle)
    {
        SemanticDiagnosticUpdate semanticDiagnosticUpdate3{};
        ASSERT_EQ(
            (buildSemanticDiagnosticUpdate(makeEntry(cycle, findings, "d"),
                                           state,
                                           semanticDiagnosticUpdate3)),
            SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);
        ASSERT_FALSE(semanticDiagnosticUpdate3.shouldEmit);
    }
    SemanticDiagnosticUpdate semanticDiagnosticUpdate4{};
    ASSERT_EQ((buildSemanticDiagnosticUpdate(makeEntry(19U, findings, "d"),
                                             state,
                                             semanticDiagnosticUpdate4)),
              SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);
    EXPECT_TRUE(semanticDiagnosticUpdate4.shouldEmit);
}

/*!
 * @brief        Checks that a failing finding reported for the first time is
 *               marked appeared, one that disappears is marked resolved, and
 *               one that persists is not reported.
 */
TEST(SemanticDiagnostics, AppearedChangedAndResolvedFailTransitions)
{
    SemanticDiagnosticState  state;
    SemanticDiagnosticUpdate semanticDiagnosticUpdate{};
    ASSERT_EQ(
        (buildSemanticDiagnosticUpdate(
            makeEntry(1U,
                      {makeFailFinding("f-stable",
                                       ReasonCode::WALL_OWNERSHIP_OWNER_BAD),
                       makeFailFinding(
                           "f-resolved",
                           ReasonCode::WALL_OWNERSHIP_OWNER_UNRESOLVABLE)},
                      "d1"),
            state,
            semanticDiagnosticUpdate)),
        SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);

    /* f-stable persists unchanged; f-resolved disappears (resolved);
     * f-new appears. Digest changes too, so this is a "summary" event. */
    SemanticDiagnosticUpdate update{};
    ASSERT_EQ(
        (buildSemanticDiagnosticUpdate(
            makeEntry(2U,
                      {makeFailFinding("f-stable",
                                       ReasonCode::WALL_OWNERSHIP_OWNER_BAD),
                       makeFailFinding("f-new",
                                       ReasonCode::WALL_OWNERSHIP_OWNER_BAD)},
                      "d2"),
            state,
            update)),
        SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);

    ASSERT_TRUE(update.shouldEmit);
    std::map<std::string, std::string> transitionById;
    for (const nlohmann::json &detail : update.violationDetails)
    {
        transitionById[detail["findingId"]] = detail["transition"];
    }
    ASSERT_EQ(transitionById.size(), 2U);
    EXPECT_EQ(transitionById.at("f-new"), "appeared");
    EXPECT_EQ(transitionById.at("f-resolved"), "resolved");
    EXPECT_EQ(transitionById.count("f-stable"), 0U);
}

/*!
 * @brief        Checks that a change in the full-geometry digest alone, with
 *               the same topology digest and findings, emits nothing.
 */
TEST(SemanticDiagnostics, GeometryOnlyDriftNeverEmitsOrRepeatsDetails)
{
    SemanticDiagnosticState    state;
    const std::vector<Finding> findings = {
        makeFailFinding("f1", ReasonCode::WALL_OWNERSHIP_OWNER_BAD)};

    SemanticDiagnosticUpdate semanticDiagnosticUpdate{};
    ASSERT_EQ((buildSemanticDiagnosticUpdate(makeEntry(1U, findings, "topo-a"),
                                             state,
                                             semanticDiagnosticUpdate)),
              SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);
    ASSERT_TRUE(semanticDiagnosticUpdate.shouldEmit);

    SemanticReportCacheEntry geometryDriftEntry =
        makeEntry(2U, findings, "topo-a");
    geometryDriftEntry.canonicalFullGeometryDigest = "geom-changed";
    geometryDriftEntry.geometryRevision            = 7U;

    SemanticDiagnosticUpdate update{};
    ASSERT_EQ(
        (buildSemanticDiagnosticUpdate(geometryDriftEntry, state, update)),
        SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);
    EXPECT_FALSE(update.shouldEmit);
}

/*!
 * @brief        Checks that 60 new failures emit only the capped number of
 *               details and that the summary counts both the emitted and the
 *               omitted ones.
 */
TEST(SemanticDiagnostics, DetailCapBoundsOutputAndCountsEverything)
{
    SemanticDiagnosticState  state;
    SemanticDiagnosticUpdate semanticDiagnosticUpdate{};
    ASSERT_EQ((buildSemanticDiagnosticUpdate(makeEntry(1U, {}, "d1"),
                                             state,
                                             semanticDiagnosticUpdate)),
              SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);

    std::vector<Finding> manyFailures;
    for (int i = 0; i < 60; ++i)
    {
        manyFailures.push_back(
            makeFailFinding("f" + std::to_string(i),
                            ReasonCode::WALL_OWNERSHIP_OWNER_BAD));
    }
    SemanticDiagnosticUpdate update{};
    ASSERT_EQ((buildSemanticDiagnosticUpdate(makeEntry(2U, manyFailures, "d2"),
                                             state,
                                             update)),
              SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);

    ASSERT_TRUE(update.shouldEmit);
    EXPECT_EQ(update.violationDetails.size(), kMaxViolationDetailsPerCycle);
    EXPECT_EQ(update.summary["emittedViolationCount"],
              kMaxViolationDetailsPerCycle);
    EXPECT_EQ(update.summary["omittedViolationCount"],
              manyFailures.size() - kMaxViolationDetailsPerCycle);
}

/*!
 * @brief        Checks that the same findings in opposite input order give
 *               identical summary and detail text, and that all of it parses as
 *               JSON.
 */
TEST(SemanticDiagnostics, OutputIsDeterministicAndJsonParseable)
{
    SemanticDiagnosticState    stateA;
    SemanticDiagnosticState    stateB;
    const std::vector<Finding> findings = {
        makeFailFinding("f2", ReasonCode::WALL_OWNERSHIP_OWNER_UNRESOLVABLE),
        makeFailFinding("f1", ReasonCode::WALL_OWNERSHIP_OWNER_BAD)};

    SemanticDiagnosticUpdate updateA{};
    ASSERT_EQ((buildSemanticDiagnosticUpdate(makeEntry(1U, findings, "d"),
                                             stateA,
                                             updateA)),
              SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);
    std::vector<Finding> reversedFindings = findings;
    std::reverse(reversedFindings.begin(), reversedFindings.end());
    SemanticDiagnosticUpdate updateB{};
    ASSERT_EQ(
        (buildSemanticDiagnosticUpdate(makeEntry(1U, reversedFindings, "d"),
                                       stateB,
                                       updateB)),
        SemanticDiagnosticsStatus::SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS);

    EXPECT_EQ(updateA.summary.dump(), updateB.summary.dump());
    ASSERT_EQ(updateA.violationDetails.size(), updateB.violationDetails.size());
    for (std::size_t i = 0U; i < updateA.violationDetails.size(); ++i)
    {
        EXPECT_EQ(updateA.violationDetails[i].dump(),
                  updateB.violationDetails[i].dump());
    }

    EXPECT_NO_THROW(nlohmann::json::parse(updateA.summary.dump()));
    for (const nlohmann::json &detail : updateA.violationDetails)
    {
        EXPECT_NO_THROW(nlohmann::json::parse(detail.dump()));
    }
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
