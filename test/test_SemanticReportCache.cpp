/**
 * Semantic-axiom-reliability-plan.md Phase 1 (P1.4): focused, ROS/Gazebo-
 * free tests for the copied-value SemanticReportCache contract.
 */

#include "Semantic/SemanticReportCache.h"

#include <chrono>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace ORB_SLAM3
{
namespace semantic
{

TEST(SemanticReportCache, UnavailableBeforeFirstUpdate)
{
    SemanticReportCache cache;
    EXPECT_FALSE(cache.isAvailable());

    /* getLatest() before any update() still returns a valid, default
     * value -- never null, never a dangling reference -- but callers must
     * consult isAvailable() to know it is not yet meaningful. */
    const SemanticReportCacheEntry entry = cache.getLatest();
    EXPECT_EQ(entry.semanticCycle, 0U);
    EXPECT_EQ(entry.updateSequence, 0U);
    EXPECT_FALSE(entry.currentMapId.has_value());
    EXPECT_TRUE(entry.evaluationReport.findings.empty());
}

TEST(SemanticReportCache, AvailableAfterOneUpdate)
{
    SemanticReportCache   cache;
    SemanticGraphSnapshot snapshot;
    AxiomEvaluationReport report;

    cache.update(snapshot,
                 report,
                 {},
                 /*semanticCycle_in=*/1U,
                 /*currentMapId_in=*/std::nullopt,
                 /*mapRevision_in=*/std::nullopt,
                 "topology-digest-1",
                 "geometry-digest-1",
                 std::chrono::milliseconds(5));

    EXPECT_TRUE(cache.isAvailable());
    const SemanticReportCacheEntry entry = cache.getLatest();
    EXPECT_EQ(entry.semanticCycle, 1U);
    EXPECT_EQ(entry.updateSequence, 1U);
}

/* Map id 0 is a real, valid Atlas map id (the first map created), distinct
 * from "no current map" (absent optional). The cache must preserve that
 * distinction rather than treating 0 as a sentinel for "none". */
TEST(SemanticReportCache, PreservesMapIdZeroDistinctFromNoMap)
{
    SemanticReportCache   cache;
    SemanticGraphSnapshot snapshot;
    AxiomEvaluationReport report;

    cache.update(snapshot,
                 report,
                 {},
                 1U,
                 /*currentMapId_in=*/0UL,
                 /*mapRevision_in=*/0,
                 "t",
                 "g",
                 std::chrono::milliseconds(0));
    SemanticReportCacheEntry withMapZero = cache.getLatest();
    ASSERT_TRUE(withMapZero.currentMapId.has_value());
    EXPECT_EQ(*withMapZero.currentMapId, 0UL);
    ASSERT_TRUE(withMapZero.mapRevision.has_value());
    EXPECT_EQ(*withMapZero.mapRevision, 0);

    cache.update(snapshot,
                 report,
                 {},
                 2U,
                 /*currentMapId_in=*/std::nullopt,
                 /*mapRevision_in=*/std::nullopt,
                 "t",
                 "g",
                 std::chrono::milliseconds(0));
    SemanticReportCacheEntry withNoMap = cache.getLatest();
    EXPECT_FALSE(withNoMap.currentMapId.has_value());
    EXPECT_FALSE(withNoMap.mapRevision.has_value());
}

/* The returned entry is a value copy: mutating it, or mutating the input
 * arguments after the call returns, must never change what a later
 * getLatest() call returns. */
TEST(SemanticReportCache, GetLatestReturnsNonAliasingCopy)
{
    SemanticReportCache   cache;
    SemanticGraphSnapshot snapshot;
    AxiomEvaluationReport report;
    Finding               finding;
    finding.id = "original";
    report.findings.push_back(finding);

    cache.update(snapshot,
                 report,
                 {},
                 1U,
                 std::nullopt,
                 std::nullopt,
                 "t",
                 "g",
                 std::chrono::milliseconds(0));

    SemanticReportCacheEntry firstCopy = cache.getLatest();
    ASSERT_EQ(firstCopy.evaluationReport.findings.size(), 1U);
    firstCopy.evaluationReport.findings[0].id = "mutated-by-caller";

    /* Mutating the input report after update() returned must also not
     * retroactively change the cached value. */
    report.findings[0].id = "mutated-input-after-update";

    const SemanticReportCacheEntry secondCopy = cache.getLatest();
    ASSERT_EQ(secondCopy.evaluationReport.findings.size(), 1U);
    EXPECT_EQ(secondCopy.evaluationReport.findings[0].id, "original");
}

TEST(SemanticReportCache,
     UpdateSequenceIsMonotonicAndIndependentOfSemanticCycle)
{
    SemanticReportCache   cache;
    SemanticGraphSnapshot snapshot;
    AxiomEvaluationReport report;

    /* semanticCycle can skip values (SemanticsManager owns numbering);
     * updateSequence must still increase by exactly one per update() call. */
    cache.update(snapshot,
                 report,
                 {},
                 5U,
                 std::nullopt,
                 std::nullopt,
                 "t",
                 "g",
                 {});
    EXPECT_EQ(cache.getLatest().updateSequence, 1U);
    cache.update(snapshot,
                 report,
                 {},
                 9U,
                 std::nullopt,
                 std::nullopt,
                 "t",
                 "g",
                 {});
    EXPECT_EQ(cache.getLatest().updateSequence, 2U);
    EXPECT_EQ(cache.getLatest().semanticCycle, 9U);
}

TEST(SemanticReportCache,
     GeometryRevisionIncrementsOnlyWhenFullGeometryDigestChanges)
{
    SemanticReportCache   cache;
    SemanticGraphSnapshot snapshot;
    AxiomEvaluationReport report;

    cache.update(snapshot,
                 report,
                 {},
                 1U,
                 std::nullopt,
                 std::nullopt,
                 "topology-a",
                 "geometry-a",
                 {});
    EXPECT_EQ(cache.getLatest().geometryRevision, 0U);

    /* Unchanged full-geometry digest (even with a changed topology digest,
     * representing e.g. a genuinely unrelated cycle) must not bump the
     * counter. */
    cache.update(snapshot,
                 report,
                 {},
                 2U,
                 std::nullopt,
                 std::nullopt,
                 "topology-b",
                 "geometry-a",
                 {});
    EXPECT_EQ(cache.getLatest().geometryRevision, 0U);

    cache.update(snapshot,
                 report,
                 {},
                 3U,
                 std::nullopt,
                 std::nullopt,
                 "topology-b",
                 "geometry-b",
                 {});
    EXPECT_EQ(cache.getLatest().geometryRevision, 1U);

    cache.update(snapshot,
                 report,
                 {},
                 4U,
                 std::nullopt,
                 std::nullopt,
                 "topology-b",
                 "geometry-c",
                 {});
    EXPECT_EQ(cache.getLatest().geometryRevision, 2U);
}

/* No ROS fixture required: this only proves the mutex actually serializes
 * concurrent writer/reader access without a crash, a torn read, or a
 * deadlock -- run under an ordinary std::thread. */
TEST(SemanticReportCache, ConcurrentWriterAndReaderStayConsistent)
{
    SemanticReportCache   cache;
    SemanticGraphSnapshot snapshot;

    constexpr int kIterations = 500;
    std::thread   writer(
        [&]()
        {
            for (int i = 0; i < kIterations; ++i)
            {
                AxiomEvaluationReport report;
                Finding               finding;
                finding.id = std::to_string(i);
                report.findings.push_back(finding);
                cache.update(snapshot,
                             report,
                               {},
                             static_cast<std::uint64_t>(i),
                             std::nullopt,
                             std::nullopt,
                             "t",
                             "g",
                               {});
            }
        });

    bool sawAnyUpdate = false;
    for (int i = 0; i < kIterations; ++i)
    {
        const SemanticReportCacheEntry entry = cache.getLatest();
        /* A torn read would show a findings vector whose single element's
         * id does not match a value the writer ever wrote atomically; the
         * mutex must prevent that regardless of scheduling. */
        if (!entry.evaluationReport.findings.empty())
        {
            sawAnyUpdate = true;
            EXPECT_EQ(entry.evaluationReport.findings.size(), 1U);
        }
    }
    writer.join();
    EXPECT_TRUE(cache.isAvailable());
    /* sawAnyUpdate is not asserted true: on an unlucky schedule the reader
     * loop could finish before the writer thread starts. The absence of a
     * crash/deadlock/inconsistent read across kIterations iterations of
     * true concurrent access is this test's actual contract. */
    (void)sawAnyUpdate;
}

} // namespace semantic
} // namespace ORB_SLAM3
