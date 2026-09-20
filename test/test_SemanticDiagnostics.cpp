/*!
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

TEST(SemanticDiagnostics, FirstCycleEmitsSummaryAndOnlyFailAppeared)
{
    SemanticDiagnosticState        state;
    const SemanticReportCacheEntry entry =
        makeEntry(1U,
                  {makeFailFinding("f1", ReasonCode::WALL_OWNERSHIP_OWNER_BAD),
                   makePassFinding("p1"),
                   makeUnknownFinding("u1")},
                  "digest-1");

    const SemanticDiagnosticUpdate update =
        buildSemanticDiagnosticUpdate(entry, state);

    ASSERT_TRUE(update.emit);
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

TEST(SemanticDiagnostics, UnchangedCycles2Through9EmitNothing)
{
    SemanticDiagnosticState    state;
    const std::vector<Finding> findings = {
        makeFailFinding("f1", ReasonCode::WALL_OWNERSHIP_OWNER_BAD)};

    ASSERT_TRUE(
        buildSemanticDiagnosticUpdate(makeEntry(1U, findings, "d"), state)
            .emit);

    for (std::uint64_t cycle = 2U; cycle <= 9U; ++cycle)
    {
        const SemanticDiagnosticUpdate update =
            buildSemanticDiagnosticUpdate(makeEntry(cycle, findings, "d"),
                                          state);
        EXPECT_FALSE(update.emit) << "cycle " << cycle;
    }
}

TEST(SemanticDiagnostics, HeartbeatEmittedExactlyOnCycle10)
{
    SemanticDiagnosticState    state;
    const std::vector<Finding> findings = {
        makeFailFinding("f1", ReasonCode::WALL_OWNERSHIP_OWNER_BAD)};

    ASSERT_TRUE(
        buildSemanticDiagnosticUpdate(makeEntry(1U, findings, "d"), state)
            .emit);
    for (std::uint64_t cycle = 2U; cycle <= 9U; ++cycle)
    {
        ASSERT_FALSE(
            buildSemanticDiagnosticUpdate(makeEntry(cycle, findings, "d"),
                                          state)
                .emit);
    }

    const SemanticDiagnosticUpdate heartbeat =
        buildSemanticDiagnosticUpdate(makeEntry(10U, findings, "d"), state);
    ASSERT_TRUE(heartbeat.emit);
    EXPECT_EQ(heartbeat.summary["eventType"], "heartbeat");
    EXPECT_TRUE(heartbeat.violationDetails.empty());

    /* And the cadence repeats: cycles 11-18 unchanged, cycle 19 heartbeats
     * again (9 cycles after cycle 10, matching the first cadence from cycle
     * 1). */
    for (std::uint64_t cycle = 11U; cycle <= 18U; ++cycle)
    {
        ASSERT_FALSE(
            buildSemanticDiagnosticUpdate(makeEntry(cycle, findings, "d"),
                                          state)
                .emit);
    }
    EXPECT_TRUE(
        buildSemanticDiagnosticUpdate(makeEntry(19U, findings, "d"), state)
            .emit);
}

TEST(SemanticDiagnostics, AppearedChangedAndResolvedFailTransitions)
{
    SemanticDiagnosticState state;
    buildSemanticDiagnosticUpdate(
        makeEntry(
            1U,
            {makeFailFinding("f-stable", ReasonCode::WALL_OWNERSHIP_OWNER_BAD),
             makeFailFinding("f-resolved",
                             ReasonCode::WALL_OWNERSHIP_OWNER_UNRESOLVABLE)},
            "d1"),
        state);

    /* f-stable persists unchanged; f-resolved disappears (resolved);
     * f-new appears. Digest changes too, so this is a "summary" event. */
    const SemanticDiagnosticUpdate update = buildSemanticDiagnosticUpdate(
        makeEntry(
            2U,
            {makeFailFinding("f-stable", ReasonCode::WALL_OWNERSHIP_OWNER_BAD),
             makeFailFinding("f-new", ReasonCode::WALL_OWNERSHIP_OWNER_BAD)},
            "d2"),
        state);

    ASSERT_TRUE(update.emit);
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

TEST(SemanticDiagnostics, GeometryOnlyDriftNeverEmitsOrRepeatsDetails)
{
    SemanticDiagnosticState    state;
    const std::vector<Finding> findings = {
        makeFailFinding("f1", ReasonCode::WALL_OWNERSHIP_OWNER_BAD)};

    ASSERT_TRUE(
        buildSemanticDiagnosticUpdate(makeEntry(1U, findings, "topo-a"), state)
            .emit);

    SemanticReportCacheEntry geometryDriftEntry =
        makeEntry(2U, findings, "topo-a");
    geometryDriftEntry.canonicalFullGeometryDigest = "geom-changed";
    geometryDriftEntry.geometryRevision            = 7U;

    const SemanticDiagnosticUpdate update =
        buildSemanticDiagnosticUpdate(geometryDriftEntry, state);
    EXPECT_FALSE(update.emit);
}

TEST(SemanticDiagnostics, DetailCapBoundsOutputAndCountsEverything)
{
    SemanticDiagnosticState state;
    buildSemanticDiagnosticUpdate(makeEntry(1U, {}, "d1"), state);

    std::vector<Finding> manyFailures;
    for (int i = 0; i < 60; ++i)
    {
        manyFailures.push_back(
            makeFailFinding("f" + std::to_string(i),
                            ReasonCode::WALL_OWNERSHIP_OWNER_BAD));
    }
    const SemanticDiagnosticUpdate update =
        buildSemanticDiagnosticUpdate(makeEntry(2U, manyFailures, "d2"), state);

    ASSERT_TRUE(update.emit);
    EXPECT_EQ(update.violationDetails.size(), kMaxViolationDetailsPerCycle);
    EXPECT_EQ(update.summary["emittedViolationCount"],
              kMaxViolationDetailsPerCycle);
    EXPECT_EQ(update.summary["omittedViolationCount"],
              manyFailures.size() - kMaxViolationDetailsPerCycle);
}

TEST(SemanticDiagnostics, OutputIsDeterministicAndJsonParseable)
{
    SemanticDiagnosticState    stateA;
    SemanticDiagnosticState    stateB;
    const std::vector<Finding> findings = {
        makeFailFinding("f2", ReasonCode::WALL_OWNERSHIP_OWNER_UNRESOLVABLE),
        makeFailFinding("f1", ReasonCode::WALL_OWNERSHIP_OWNER_BAD)};

    const SemanticDiagnosticUpdate updateA =
        buildSemanticDiagnosticUpdate(makeEntry(1U, findings, "d"), stateA);
    std::vector<Finding> reversedFindings = findings;
    std::reverse(reversedFindings.begin(), reversedFindings.end());
    const SemanticDiagnosticUpdate updateB =
        buildSemanticDiagnosticUpdate(makeEntry(1U, reversedFindings, "d"),
                                      stateB);

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
