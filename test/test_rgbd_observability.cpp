/**
 * @file test_rgbd_observability.cpp
 * @brief Tests deterministic RGB-D accounting and reset attribution strings.
 */

#include "../src/RgbdObservability.h"
#include "ResetCause.h"
#include "RgbdAllPointsCadence.h"

#include <chrono>
#include <future>
#include <limits>
#include <string>
#include <thread>

#include <gtest/gtest.h>

namespace
{
using vs_graphs::observability::PublishTopic;
using vs_graphs::observability::PublishTopicsTimingSink;
using vs_graphs::observability::RgbdObservability;
using vs_graphs::observability::RgbdObservabilitySnapshot;

using vs_graphs::rgbd::AllPointsCadence;

TEST(AllPointsCadenceTest, HandlesSimulatedTimeBoundariesAndRevision)
{
    AllPointsCadence cadence(1000000000);
    EXPECT_TRUE(cadence.shouldPublish(10, 1U));
    cadence.commitPublication();
    EXPECT_FALSE(cadence.shouldPublish(1000000009, 1U));
    EXPECT_TRUE(cadence.shouldPublish(1010000000, 1U));
    cadence.commitPublication();
    EXPECT_TRUE(cadence.shouldPublish(5, 1U));
    cadence.commitPublication();
    EXPECT_FALSE(cadence.shouldPublish(6, 1U));
    EXPECT_TRUE(cadence.shouldPublish(6, 2U));
    cadence.commitPublication();
}

TEST(AllPointsCadenceTest, ZeroPeriodPublishesEveryPacket)
{
    AllPointsCadence cadence(0);
    EXPECT_TRUE(cadence.shouldPublish(10, 1U));
    cadence.commitPublication();
    EXPECT_TRUE(cadence.shouldPublish(10, 1U));
}

TEST(AllPointsCadenceTest, FailedPublicationDoesNotConsumeCadence)
{
    AllPointsCadence cadence(1000000000);
    EXPECT_TRUE(cadence.shouldPublish(10, 1U));
    cadence.commitPublication();

    EXPECT_TRUE(cadence.shouldPublish(1000000010, 1U));
    cadence.rollbackPublication();

    EXPECT_TRUE(cadence.shouldPublish(1000000010, 1U));
    cadence.commitPublication();
    EXPECT_FALSE(cadence.shouldPublish(1000000011, 1U));
}

RgbdObservability::SteadyTime steadyNanoseconds(const std::int64_t value_in)
{
    return RgbdObservability::SteadyTime(std::chrono::nanoseconds(value_in));
}

TEST(RgbdObservabilityTest, AccountsEveryBoundaryOutcome)
{
    RgbdObservability accounting;

    for (std::uint64_t admissionIndex = 0U; admissionIndex < 6U;
         ++admissionIndex)
    {
        accounting.recordCallbackAdmission();
    }
    accounting.recordRgbDepthSkewReject();
    accounting.recordCloudImageSkewReject();
    accounting.recordNonMonotonicReject();
    accounting.recordPendingStore(1U, false);
    accounting.recordPendingStore(4U, true);
    accounting.recordPendingOverflow();
    accounting.recordWorkerStart(steadyNanoseconds(1), steadyNanoseconds(2));
    accounting.recordImageConversionReject();
    accounting.recordWorkerStart(steadyNanoseconds(3), steadyNanoseconds(4));
    accounting.recordCloudConversionReject();
    accounting.recordTrackCall();
    accounting.recordTrackCall();
    accounting.recordTrackCompletion();
    accounting.recordTrackFailure();
    accounting.recordShutdownPendingDrop();

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.callbackAdmissions, 6U);
    EXPECT_EQ(snapshot.callbacksInFlight, 0U);
    EXPECT_EQ(snapshot.workersInFlight, 0U);
    EXPECT_EQ(snapshot.rgbDepthSkewRejects, 1U);
    EXPECT_EQ(snapshot.cloudImageSkewRejects, 1U);
    EXPECT_EQ(snapshot.nonMonotonicRejects, 1U);
    EXPECT_EQ(snapshot.pendingStores, 2U);
    EXPECT_EQ(snapshot.pendingOverwrites, 1U);
    EXPECT_EQ(snapshot.pendingOverflows, 1U);
    EXPECT_EQ(snapshot.imageConversionRejects, 1U);
    EXPECT_EQ(snapshot.cloudConversionRejects, 1U);
    EXPECT_EQ(snapshot.trackCalls, 2U);
    EXPECT_EQ(snapshot.trackCompletions, 1U);
    EXPECT_EQ(snapshot.trackFailures, 1U);
    EXPECT_EQ(snapshot.shutdownPendingDrops, 1U);
    EXPECT_EQ(snapshot.pendingDepthHighWater, 4U);
    EXPECT_EQ(snapshot.callbackAdmissions,
              snapshot.rgbDepthSkewRejects + snapshot.cloudImageSkewRejects +
                  snapshot.nonMonotonicRejects + snapshot.pendingStores +
                  snapshot.pendingOverflows +
                  snapshot.shutdownAdmissionRejects +
                  snapshot.callbacksInFlight);
    EXPECT_EQ(snapshot.trackCalls,
              snapshot.trackCompletions + snapshot.trackFailures);
}

TEST(RgbdObservabilityTest, ImageConversionRejectCompletesWorker)
{
    RgbdObservability accounting;
    accounting.recordCallbackAdmission();
    accounting.recordPendingStore(1U);
    accounting.recordWorkerStart(steadyNanoseconds(1), steadyNanoseconds(2));

    EXPECT_EQ(accounting.snapshot().callbacksInFlight, 0U);
    EXPECT_EQ(accounting.snapshot().workersInFlight, 1U);
    accounting.recordImageConversionReject();

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.callbackAdmissions, 1U);
    EXPECT_EQ(snapshot.callbacksInFlight, 0U);
    EXPECT_EQ(snapshot.workersInFlight, 0U);
    EXPECT_EQ(snapshot.imageConversionRejects, 1U);
    EXPECT_EQ(snapshot.cloudConversionRejects, 0U);
}

TEST(RgbdObservabilityTest, CloudConversionRejectCompletesWorker)
{
    RgbdObservability accounting;
    accounting.recordCallbackAdmission();
    accounting.recordPendingStore(1U);
    accounting.recordWorkerStart(steadyNanoseconds(1), steadyNanoseconds(2));

    EXPECT_EQ(accounting.snapshot().callbacksInFlight, 0U);
    EXPECT_EQ(accounting.snapshot().workersInFlight, 1U);
    accounting.recordCloudConversionReject();

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.callbackAdmissions, 1U);
    EXPECT_EQ(snapshot.callbacksInFlight, 0U);
    EXPECT_EQ(snapshot.workersInFlight, 0U);
    EXPECT_EQ(snapshot.imageConversionRejects, 0U);
    EXPECT_EQ(snapshot.cloudConversionRejects, 1U);
}

TEST(RgbdObservabilityTest, KeepsSensorAndSteadyClockDomainsSeparate)
{
    RgbdObservability accounting;

    accounting.recordWorkerStart(steadyNanoseconds(10), steadyNanoseconds(15));
    accounting.recordWorkerStart(steadyNanoseconds(30), steadyNanoseconds(28));
    accounting.recordProcessed(9'000'000'000'000LL,
                               steadyNanoseconds(20),
                               steadyNanoseconds(27));
    accounting.recordProcessed(9'000'000'000'060LL,
                               steadyNanoseconds(40),
                               steadyNanoseconds(52));

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.queueWaitSamples, 2U);
    EXPECT_EQ(snapshot.queueWaitTotalNanoseconds, 5U);
    EXPECT_EQ(snapshot.queueWaitMaxNanoseconds, 5U);
    EXPECT_EQ(snapshot.processingDurationSamples, 2U);
    EXPECT_EQ(snapshot.processingDurationTotalNanoseconds, 19U);
    EXPECT_EQ(snapshot.processingDurationMaxNanoseconds, 12U);
    EXPECT_EQ(snapshot.processedPackets, 2U);
    EXPECT_EQ(snapshot.workersInFlight, 0U);
    EXPECT_EQ(snapshot.lastProcessedSensorTimestampNanoseconds,
              9'000'000'000'060LL);
    EXPECT_EQ(snapshot.interProcessedSensorGapSamples, 1U);
    EXPECT_EQ(snapshot.interProcessedSensorGapMaxNanoseconds, 60U);
}

TEST(RgbdObservabilityTest, AggregatesSuccessfulStageDurations)
{
    RgbdObservability accounting;
    accounting.recordImageConversion(steadyNanoseconds(10),
                                     steadyNanoseconds(15));
    accounting.recordCloudPreparation(steadyNanoseconds(20),
                                      steadyNanoseconds(28));
    accounting.recordMarkerAssociation(steadyNanoseconds(30),
                                       steadyNanoseconds(35));
    accounting.recordTrackDuration(steadyNanoseconds(40),
                                   steadyNanoseconds(52));
    accounting.recordPublishTopics(steadyNanoseconds(60),
                                   steadyNanoseconds(66));
    accounting.recordPublishTopic(PublishTopic::ALL_MAPPED_WALLS,
                                  true,
                                  steadyNanoseconds(70),
                                  steadyNanoseconds(75));
    accounting.recordPublishTopic(PublishTopic::PLANES,
                                  false,
                                  steadyNanoseconds(80),
                                  steadyNanoseconds(81));
    accounting.recordPublishTopic(PublishTopic::PLANES,
                                  true,
                                  steadyNanoseconds(90),
                                  steadyNanoseconds(102));

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.imageConversionSamples, 1U);
    EXPECT_EQ(snapshot.imageConversionTotalNanoseconds, 5U);
    EXPECT_EQ(snapshot.cloudPreparationTotalNanoseconds, 8U);
    EXPECT_EQ(snapshot.markerAssociationMaxNanoseconds, 5U);
    EXPECT_EQ(snapshot.trackDurationTotalNanoseconds, 12U);
    EXPECT_EQ(snapshot.publishTopicsSamples, 1U);
    EXPECT_EQ(snapshot.publishAllMappedWallsSamples, 1U);
    EXPECT_EQ(snapshot.publishAllMappedWallsTotalNanoseconds, 5U);
    EXPECT_EQ(snapshot.publishPlanesCalls, 1U);
    EXPECT_EQ(snapshot.publishPlanesSamples, 1U);
    EXPECT_EQ(snapshot.publishPlanesTotalNanoseconds, 12U);
}

TEST(RgbdObservabilityTest, RecordsCallbackGapsAndCompleteRateWindows)
{
    RgbdObservability accounting;
    accounting.recordCallbackAdmission(steadyNanoseconds(0));
    accounting.recordCallbackAdmission(steadyNanoseconds(200'000'000));
    accounting.recordCallbackAdmission(steadyNanoseconds(1'000'000'000));

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.callbackArrivalGapSamples, 2U);
    EXPECT_EQ(snapshot.callbackArrivalGapMaxNanoseconds, 800'000'000U);
    EXPECT_EQ(snapshot.callbackArrivalRateNumerator, 2U);
    EXPECT_EQ(snapshot.callbackArrivalRateDenominatorNanoseconds,
              1'000'000'000U);
    EXPECT_EQ(snapshot.processedCompletionRateNumerator, 0U);
    EXPECT_EQ(snapshot.processedCompletionRateDenominatorNanoseconds, 0U);
}

TEST(RgbdObservabilityTest, StoredPacketsHaveOneTerminalIdentity)
{
    RgbdObservability accounting;
    for (std::int64_t packetIndex = 0; packetIndex < 5; ++packetIndex)
    {
        accounting.recordCallbackAdmission(steadyNanoseconds(packetIndex));
        accounting.recordPendingStore(1U);
    }
    accounting.recordImageConversionReject();
    accounting.recordCloudConversionReject();
    accounting.recordTrackFailure();
    accounting.recordProcessed(1, steadyNanoseconds(1), steadyNanoseconds(2));
    accounting.recordShutdownPendingDrop();

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.pendingStores,
              snapshot.imageConversionRejects +
                  snapshot.cloudConversionRejects + snapshot.processedPackets +
                  snapshot.trackFailures + snapshot.shutdownPendingDrops);
}

TEST(RgbdObservabilityTest, PublicationFailureHasOneStoredPacketTerminal)
{
    RgbdObservability accounting;
    accounting.recordCallbackAdmission(steadyNanoseconds(10));
    accounting.recordPendingStore(1U);
    accounting.recordPublishTopics(steadyNanoseconds(20),
                                   steadyNanoseconds(27));
    accounting.recordPublishTopicsFailure();

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.publishTopicsSamples, 1U);
    EXPECT_EQ(snapshot.publishTopicsFailures, 1U);
    EXPECT_EQ(snapshot.pendingStores,
              snapshot.imageConversionRejects +
                  snapshot.cloudConversionRejects + snapshot.processedPackets +
                  snapshot.trackFailures + snapshot.publishTopicsFailures +
                  snapshot.shutdownPendingDrops);
}

TEST(RgbdObservabilityTest, AccountsLateShutdownAdmissionAsTerminal)
{
    RgbdObservability accounting;
    accounting.recordCallbackAdmission(steadyNanoseconds(10));
    accounting.recordShutdownAdmissionReject();

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.callbackAdmissions, 1U);
    EXPECT_EQ(snapshot.callbacksInFlight, 0U);
    EXPECT_EQ(snapshot.shutdownAdmissionRejects, 1U);
    EXPECT_EQ(snapshot.pendingStores, 0U);
    EXPECT_EQ(snapshot.shutdownPendingDrops, 0U);
}

TEST(RgbdObservabilityTest, SuppliesNonOwningTimingSink)
{
    RgbdObservability             accounting;
    const PublishTopicsTimingSink sink = accounting.publishTopicsTimingSink();

    ASSERT_NE(sink.callback, nullptr);
    sink.callback(sink.p_context,
                  PublishTopic::TRACKED_POINTS,
                  true,
                  steadyNanoseconds(10),
                  steadyNanoseconds(19));

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.publishTrackedPointsSamples, 1U);
    EXPECT_EQ(snapshot.publishTrackedPointsTotalNanoseconds, 9U);
    EXPECT_EQ(snapshot.publishTrackedPointsMaxNanoseconds, 9U);
}

TEST(RgbdObservabilityTest, ClampsBackwardStageDurationsAndSensorGaps)
{
    RgbdObservability accounting;
    accounting.recordImageConversion(steadyNanoseconds(20),
                                     steadyNanoseconds(10));
    accounting.recordTrackDuration(steadyNanoseconds(20),
                                   steadyNanoseconds(10));
    accounting.recordTrackCompletion();
    accounting.recordProcessed(100,
                               steadyNanoseconds(100),
                               steadyNanoseconds(100));
    accounting.recordProcessed(90,
                               steadyNanoseconds(90),
                               steadyNanoseconds(90));
    accounting.recordProcessed(130,
                               steadyNanoseconds(130),
                               steadyNanoseconds(130));

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.imageConversionTotalNanoseconds, 0U);
    EXPECT_EQ(snapshot.trackDurationMaxNanoseconds, 0U);
    EXPECT_EQ(snapshot.processedCompletionGapMaxNanoseconds, 30U);
    EXPECT_EQ(snapshot.processedTumblingWindowSamples, 0U);
}

TEST(RgbdObservabilityTest, ReportsCompletedPassingAndFailingWindows)
{
    RgbdObservability passing;
    passing.recordTrackCompletion();
    EXPECT_EQ(passing.snapshot().processedTumblingWindowSamples, 0U);
    passing.recordProcessed(0, steadyNanoseconds(0), steadyNanoseconds(0));
    passing.recordProcessed(1,
                            steadyNanoseconds(1),
                            steadyNanoseconds(100'000'000));
    passing.recordProcessed(2,
                            steadyNanoseconds(2),
                            steadyNanoseconds(200'000'000));
    passing.recordProcessed(3,
                            steadyNanoseconds(3),
                            steadyNanoseconds(1'100'000'000));

    RgbdObservabilitySnapshot passingSnapshot = passing.snapshot();
    EXPECT_EQ(passingSnapshot.processedTumblingWindowSamples, 1U);
    EXPECT_EQ(passingSnapshot.processedTumblingWindowMinimumCount, 3U);
    EXPECT_EQ(passingSnapshot.processedTumblingWindowMedianCount, 3U);
    EXPECT_EQ(passingSnapshot.processedCompletionRateNumerator, 3U);
    EXPECT_EQ(passingSnapshot.processedCompletionRateDenominatorNanoseconds,
              1'000'000'000U);
    EXPECT_EQ(passingSnapshot.processedCompletionGapMaxNanoseconds,
              900'000'000U);

    RgbdObservability failing;
    failing.recordTrackCompletion();
    failing.recordProcessed(0, steadyNanoseconds(0), steadyNanoseconds(0));
    failing.recordProcessed(1,
                            steadyNanoseconds(2'000'000'000),
                            steadyNanoseconds(2'000'000'000));

    const RgbdObservabilitySnapshot failingSnapshot = failing.snapshot();
    EXPECT_EQ(failingSnapshot.processedTumblingWindowSamples, 2U);
    EXPECT_EQ(failingSnapshot.processedTumblingWindowMinimumCount, 0U);
    EXPECT_EQ(failingSnapshot.processedTumblingWindowMedianCount, 0U);
}

TEST(RgbdObservabilityTest, SaturatesExtremeForwardSensorGap)
{
    RgbdObservability accounting;
    accounting.recordProcessed(std::numeric_limits<std::int64_t>::min(),
                               steadyNanoseconds(0),
                               steadyNanoseconds(0));
    accounting.recordProcessed(std::numeric_limits<std::int64_t>::max(),
                               steadyNanoseconds(1),
                               steadyNanoseconds(1));

    EXPECT_EQ(accounting.snapshot().interProcessedSensorGapMaxNanoseconds,
              std::numeric_limits<std::uint64_t>::max());

    RgbdObservability reversed;
    reversed.recordProcessed(std::numeric_limits<std::int64_t>::max(),
                             steadyNanoseconds(0),
                             steadyNanoseconds(0));
    reversed.recordProcessed(std::numeric_limits<std::int64_t>::min(),
                             steadyNanoseconds(1),
                             steadyNanoseconds(1));
    EXPECT_EQ(reversed.snapshot().interProcessedSensorGapMaxNanoseconds, 0U);
}

TEST(RgbdObservabilityTest, IgnoresOutOfOrderProcessedSteadyTimes)
{
    RgbdObservability accounting;
    accounting.recordProcessed(10,
                               steadyNanoseconds(0),
                               steadyNanoseconds(1'000'000'000));
    accounting.recordProcessed(11,
                               steadyNanoseconds(1),
                               steadyNanoseconds(500'000'000));
    accounting.recordProcessed(12,
                               steadyNanoseconds(2),
                               steadyNanoseconds(2'100'000'000));

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.processedCompletionOutOfOrderCount, 1U);
    EXPECT_EQ(snapshot.processedTumblingWindowSamples, 1U);
    EXPECT_EQ(snapshot.processedCompletionGapMaxNanoseconds, 1'100'000'000U);
}

TEST(RgbdObservabilityTest, ExposesAdmissionUntilItsOutcomeIsRecorded)
{
    RgbdObservability  accounting;
    std::promise<void> admissionRecorded;
    std::promise<void> allowOutcome;
    std::future<void>  admissionFuture = admissionRecorded.get_future();
    std::future<void>  outcomeFuture   = allowOutcome.get_future();

    std::thread callbackThread(
        [&accounting, &admissionRecorded, &outcomeFuture]()
        {
            accounting.recordCallbackAdmission();
            admissionRecorded.set_value();
            outcomeFuture.wait();
            accounting.recordPendingStore(1U, false);
        });

    admissionFuture.wait();
    RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.callbackAdmissions, 1U);
    EXPECT_EQ(snapshot.callbacksInFlight, 1U);
    EXPECT_EQ(snapshot.pendingStores, 0U);

    allowOutcome.set_value();
    callbackThread.join();
    snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.callbackAdmissions, 1U);
    EXPECT_EQ(snapshot.callbacksInFlight, 0U);
    EXPECT_EQ(snapshot.pendingStores, 1U);
}

TEST(RgbdObservabilityTest, SupportsConcurrentStageUpdatesAndSnapshots)
{
    RgbdObservability accounting;
    std::thread       updateThread(
        [&accounting]()
        {
            for (std::int64_t sampleIndex = 0; sampleIndex < 100; ++sampleIndex)
            {
                accounting.recordTrackDuration(
                    steadyNanoseconds(sampleIndex),
                    steadyNanoseconds(sampleIndex + 1));
                accounting.recordTrackCompletion();
                accounting.recordProcessed(
                    sampleIndex,
                    steadyNanoseconds(sampleIndex),
                    steadyNanoseconds(sampleIndex * 20'000'000));
            }
        });

    for (std::size_t snapshotIndex = 0U; snapshotIndex < 100U; ++snapshotIndex)
    {
        const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
        EXPECT_GE(snapshot.trackDurationSamples, 0U);
        EXPECT_LE(snapshot.trackDurationSamples, 100U);
        EXPECT_LE(snapshot.trackCompletions, 100U);
    }
    updateThread.join();

    const RgbdObservabilitySnapshot snapshot = accounting.snapshot();
    EXPECT_EQ(snapshot.trackDurationSamples, 100U);
    EXPECT_EQ(snapshot.trackCompletions, 100U);
}

TEST(RgbdObservabilityTest, FormatsStableCumulativeSchema)
{
    RgbdObservabilitySnapshot snapshot;
    snapshot.callbackAdmissions    = 2U;
    snapshot.pendingStores         = 1U;
    snapshot.pendingOverwrites     = 1U;
    snapshot.pendingDepthHighWater = 1U;

    const std::string expected =
        "VSG_RGBD_OBSERVABILITY event=periodic "
        "boundary=grab_rgbd_synchronized_triples pre_boundary_drops=unknown "
        "callback_admissions=2 callbacks_in_flight=0 workers_in_flight=0 "
        "rgb_depth_skew_rejects=0 "
        "cloud_image_skew_rejects=0 non_monotonic_rejects=0 "
        "pending_stores=1 pending_overwrites=1 pending_overflows=0 "
        "image_conversion_rejects=0 "
        "cloud_conversion_rejects=0 track_calls=0 track_completions=0 "
        "track_failures=0 processed_packets=0 publish_topics_failures=0 "
        "shutdown_admission_rejects=0 "
        "shutdown_pending_drops=0 "
        "queue_wait_samples=0 queue_wait_total_ns=0 "
        "queue_wait_max_ns=0 processing_duration_samples=0 "
        "processing_duration_total_ns=0 processing_duration_max_ns=0 "
        "inter_processed_sensor_gap_samples=0 "
        "inter_processed_sensor_gap_max_ns=0 pending_depth_high_water=1 "
        "image_conversion_samples=0 image_conversion_total_ns=0 "
        "image_conversion_max_ns=0 cloud_preparation_samples=0 "
        "cloud_preparation_total_ns=0 cloud_preparation_max_ns=0 "
        "marker_association_samples=0 marker_association_total_ns=0 "
        "marker_association_max_ns=0 track_duration_samples=0 "
        "track_duration_total_ns=0 track_duration_max_ns=0 "
        "publish_topics_samples=0 publish_topics_total_ns=0 "
        "publish_topics_max_ns=0 publish_all_mapped_walls_samples=0 "
        "publish_all_mapped_walls_total_ns=0 publish_all_mapped_walls_max_ns=0 "
        "publish_segmented_cloud_samples=0 publish_segmented_cloud_total_ns=0 "
        "publish_segmented_cloud_max_ns=0 publish_planes_calls=0 "
        "publish_planes_samples=0 publish_planes_total_ns=0 "
        "publish_planes_max_ns=0 publish_all_points_calls=0 "
        "publish_all_points_samples=0 "
        "publish_all_points_total_ns=0 publish_all_points_max_ns=0 "
        "publish_tracked_points_samples=0 publish_tracked_points_total_ns=0 "
        "publish_tracked_points_max_ns=0 publish_free_space_clusters_samples=0 "
        "publish_free_space_clusters_total_ns=0 "
        "publish_free_space_clusters_max_ns=0 "
        "processed_completion_gap_max_ns=0 "
        "processed_completion_out_of_order_count=0 "
        "processed_tumbling_window_samples=0 "
        "processed_tumbling_window_min_count=0 "
        "processed_tumbling_window_median_count=0 "
        "callback_arrival_gap_samples=0 callback_arrival_gap_max_ns=0 "
        "callback_arrival_rate_numerator=0 "
        "callback_arrival_rate_denominator_ns=0 "
        "processed_completion_rate_numerator=0 "
        "processed_completion_rate_denominator_ns=0 "
        "last_processed_sensor_timestamp_ns=0";

    EXPECT_EQ(
        vs_graphs::observability::formatRgbdObservabilitySummary(snapshot,
                                                                 "periodic"),
        expected);
}

TEST(ResetCauseTest, FormatsStableCauseAndAction)
{
    using vs_graphs::core::ResetAction;
    using vs_graphs::core::ResetCause;

    EXPECT_STREQ(vs_graphs::core::resetCauseToString(
                     ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP),
                 "visual_tracking_lost_small_map");
    EXPECT_STREQ(
        vs_graphs::core::resetActionToString(ResetAction::RESET_ACTIVE_MAP_REQUEST),
        "reset_active_map_request");
    EXPECT_EQ(vs_graphs::core::formatResetAttribution(
                  ResetCause::VISUAL_TRACKING_LOST_NEW_MAP,
                  ResetAction::CREATE_MAP_EXECUTION),
              "VSG_RESET_ATTRIBUTION cause=visual_tracking_lost_new_map "
              "action=create_map_execution");
}

TEST(ResetCauseTest, RetainsSingleCauseAndMarksUnlikeCoalescedRequests)
{
    using vs_graphs::core::ResetCause;
    using vs_graphs::core::ResetCauseRetention;

    ResetCauseRetention retention;
    EXPECT_FALSE(retention.hasRetainedCause());

    retention.retain(ResetCause::IMU_DELIVERY_GAP);
    retention.retain(ResetCause::IMU_DELIVERY_GAP);
    EXPECT_TRUE(retention.hasRetainedCause());
    EXPECT_EQ(retention.consume(), ResetCause::IMU_DELIVERY_GAP);
    EXPECT_FALSE(retention.hasRetainedCause());

    retention.retain(ResetCause::VIEWER_REQUEST);
    retention.retain(ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP);
    EXPECT_EQ(retention.consume(), ResetCause::MULTIPLE_COALESCED_REQUESTS);
    EXPECT_EQ(retention.consume(), ResetCause::UNATTRIBUTED_PUBLIC_REQUEST);
}

TEST(ResetCauseTest, RetainsCausesPerOwnerWithoutOwnerLayoutChanges)
{
    using vs_graphs::core::ResetCause;

    const int firstOwner  = 1;
    const int secondOwner = 2;
    vs_graphs::core::retainResetCause(&firstOwner, ResetCause::VIEWER_REQUEST);
    vs_graphs::core::retainResetCause(&secondOwner, ResetCause::IMU_DELIVERY_GAP);

    EXPECT_EQ(vs_graphs::core::consumeResetCause(&firstOwner),
              ResetCause::VIEWER_REQUEST);
    EXPECT_EQ(vs_graphs::core::consumeResetCause(&secondOwner),
              ResetCause::IMU_DELIVERY_GAP);
    EXPECT_EQ(vs_graphs::core::consumeResetCause(&firstOwner),
              ResetCause::UNATTRIBUTED_PUBLIC_REQUEST);

    vs_graphs::core::retainResetCause(&firstOwner, ResetCause::VIEWER_REQUEST);
    vs_graphs::core::clearResetCause(&firstOwner);
    EXPECT_EQ(vs_graphs::core::consumeResetCause(&firstOwner),
              ResetCause::UNATTRIBUTED_PUBLIC_REQUEST);
}
} /* namespace */
