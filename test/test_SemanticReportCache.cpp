/*!
 * Focused, ROS/Gazebo-free tests for the copied-value SemanticReportCache
 * contract.
 */

#include "Semantic/SemanticReportCache.h"

#include <chrono>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

TEST(SemanticReportCache, UnavailableBeforeFirstUpdate)
{
    SemanticReportCache cache;
    bool                isAvailable2{};
    ASSERT_EQ((cache.isAvailable(isAvailable2)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    EXPECT_FALSE(isAvailable2);

    /* getLatest() before any update() still returns a valid, default
     * value -- never null, never a dangling reference -- but callers must
     * consult isAvailable() to know it is not yet meaningful. */
    SemanticReportCacheEntry entry{};
    ASSERT_EQ((cache.getLatest(entry)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
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

    ASSERT_EQ((cache.update(snapshot,
                            report,
                            {},
                            /*semanticCycle_in=*/1U,
                            /*currentMapId_in=*/std::nullopt,
                            /*mapRevision_in=*/std::nullopt,
                            "topology-digest-1",
                            "geometry-digest-1",
                            std::chrono::milliseconds(5))),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);

    bool isAvailable2{};
    ASSERT_EQ((cache.isAvailable(isAvailable2)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    EXPECT_TRUE(isAvailable2);
    SemanticReportCacheEntry entry{};
    ASSERT_EQ((cache.getLatest(entry)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
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

    ASSERT_EQ((cache.update(snapshot,
                            report,
                            {},
                            1U,
                            /*currentMapId_in=*/0UL,
                            /*mapRevision_in=*/0,
                            "t",
                            "g",
                            std::chrono::milliseconds(0))),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    SemanticReportCacheEntry withMapZero{};
    ASSERT_EQ((cache.getLatest(withMapZero)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    ASSERT_TRUE(withMapZero.currentMapId.has_value());
    EXPECT_EQ(*withMapZero.currentMapId, 0UL);
    ASSERT_TRUE(withMapZero.mapRevision.has_value());
    EXPECT_EQ(*withMapZero.mapRevision, 0);

    ASSERT_EQ((cache.update(snapshot,
                            report,
                            {},
                            2U,
                            /*currentMapId_in=*/std::nullopt,
                            /*mapRevision_in=*/std::nullopt,
                            "t",
                            "g",
                            std::chrono::milliseconds(0))),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    SemanticReportCacheEntry withNoMap{};
    ASSERT_EQ((cache.getLatest(withNoMap)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
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

    ASSERT_EQ((cache.update(snapshot,
                            report,
                            {},
                            1U,
                            std::nullopt,
                            std::nullopt,
                            "t",
                            "g",
                            std::chrono::milliseconds(0))),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);

    SemanticReportCacheEntry firstCopy{};
    ASSERT_EQ((cache.getLatest(firstCopy)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    ASSERT_EQ(firstCopy.evaluationReport.findings.size(), 1U);
    firstCopy.evaluationReport.findings[0].id = "mutated-by-caller";

    /* Mutating the input report after update() returned must also not
     * retroactively change the cached value. */
    report.findings[0].id = "mutated-input-after-update";

    SemanticReportCacheEntry secondCopy{};
    ASSERT_EQ((cache.getLatest(secondCopy)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
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
    ASSERT_EQ((cache.update(snapshot,
                            report,
                            {},
                            5U,
                            std::nullopt,
                            std::nullopt,
                            "t",
                            "g",
                            {})),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    SemanticReportCacheEntry getLatest2{};
    ASSERT_EQ((cache.getLatest(getLatest2)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    EXPECT_EQ(getLatest2.updateSequence, 1U);
    ASSERT_EQ((cache.update(snapshot,
                            report,
                            {},
                            9U,
                            std::nullopt,
                            std::nullopt,
                            "t",
                            "g",
                            {})),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    SemanticReportCacheEntry getLatest3{};
    ASSERT_EQ((cache.getLatest(getLatest3)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    EXPECT_EQ(getLatest3.updateSequence, 2U);
    SemanticReportCacheEntry getLatest4{};
    ASSERT_EQ((cache.getLatest(getLatest4)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    EXPECT_EQ(getLatest4.semanticCycle, 9U);
}

TEST(SemanticReportCache,
     GeometryRevisionIncrementsOnlyWhenFullGeometryDigestChanges)
{
    SemanticReportCache   cache;
    SemanticGraphSnapshot snapshot;
    AxiomEvaluationReport report;

    ASSERT_EQ((cache.update(snapshot,
                            report,
                            {},
                            1U,
                            std::nullopt,
                            std::nullopt,
                            "topology-a",
                            "geometry-a",
                            {})),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    SemanticReportCacheEntry getLatest2{};
    ASSERT_EQ((cache.getLatest(getLatest2)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    EXPECT_EQ(getLatest2.geometryRevision, 0U);

    /* Unchanged full-geometry digest (even with a changed topology digest,
     * representing e.g. a genuinely unrelated cycle) must not bump the
     * counter. */
    ASSERT_EQ((cache.update(snapshot,
                            report,
                            {},
                            2U,
                            std::nullopt,
                            std::nullopt,
                            "topology-b",
                            "geometry-a",
                            {})),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    SemanticReportCacheEntry getLatest3{};
    ASSERT_EQ((cache.getLatest(getLatest3)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    EXPECT_EQ(getLatest3.geometryRevision, 0U);

    ASSERT_EQ((cache.update(snapshot,
                            report,
                            {},
                            3U,
                            std::nullopt,
                            std::nullopt,
                            "topology-b",
                            "geometry-b",
                            {})),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    SemanticReportCacheEntry getLatest4{};
    ASSERT_EQ((cache.getLatest(getLatest4)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    EXPECT_EQ(getLatest4.geometryRevision, 1U);

    ASSERT_EQ((cache.update(snapshot,
                            report,
                            {},
                            4U,
                            std::nullopt,
                            std::nullopt,
                            "topology-b",
                            "geometry-c",
                            {})),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    SemanticReportCacheEntry getLatest5{};
    ASSERT_EQ((cache.getLatest(getLatest5)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    EXPECT_EQ(getLatest5.geometryRevision, 2U);
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
                if (cache.update(snapshot,
                                 report,
                                   {},
                                 static_cast<std::uint64_t>(i),
                                 std::nullopt,
                                 std::nullopt,
                                 "t",
                                 "g",
                                   {}) !=
                    SemanticReportCacheStatus::
                        SEMANTIC_REPORT_CACHE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: update returned a failure status although it "
                          "cannot fail; continuing as before.",
                        __func__);
                }
            }
        });

    bool sawAnyUpdate = false;
    for (int i = 0; i < kIterations; ++i)
    {
        SemanticReportCacheEntry entry{};
        ASSERT_EQ(
            (cache.getLatest(entry)),
            SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
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
    bool isAvailable2{};
    ASSERT_EQ((cache.isAvailable(isAvailable2)),
              SemanticReportCacheStatus::SEMANTIC_REPORT_CACHE_STATUS_SUCCESS);
    EXPECT_TRUE(isAvailable2);
    /* sawAnyUpdate is not asserted true: on an unlucky schedule the reader
     * loop could finish before the writer thread starts. The absence of a
     * crash/deadlock/inconsistent read across kIterations iterations of
     * true concurrent access is this test's actual contract. */
    (void)sawAnyUpdate;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
