/*!
 * @file RgbdObservability.cc
 * @brief Implements package-private RGB-D callback/worker accounting.
 */

#include "RgbdObservability.h"

#include <algorithm>
#include <sstream>

namespace vs_graphs::observability
{
namespace
{
std::uint64_t
    nonNegativeNanoseconds(const RgbdObservability::SteadyTime start_in,
                           const RgbdObservability::SteadyTime end_in)
{
    const std::chrono::nanoseconds duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end_in - start_in);
    return duration.count() > 0 ? static_cast<std::uint64_t>(duration.count())
                                : 0U;
}

void addStageSample(const std::uint64_t durationNanoseconds,
                    std::uint64_t      &samples_out,
                    std::uint64_t      &totalNanoseconds_out,
                    std::uint64_t      &maxNanoseconds_out)
{
    ++samples_out;
    totalNanoseconds_out += durationNanoseconds;
    maxNanoseconds_out = std::max(maxNanoseconds_out, durationNanoseconds);
}
} /* namespace */

void RgbdObservability::recordCallbackAdmission()
{
    recordCallbackAdmission(SteadyTime::clock::now());
}

void RgbdObservability::recordCallbackAdmission(const SteadyTime arrival_in)
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.callbackAdmissions;
    ++counters.callbacksInFlight;

    if (!hasCallbackArrivalWindow)
    {
        callbackArrivalWindowStart = arrival_in;
        hasCallbackArrivalWindow   = true;
    }
    else
    {
        ++counters.callbackArrivalGapSamples;
        counters.callbackArrivalGapMaxNanoseconds =
            std::max(counters.callbackArrivalGapMaxNanoseconds,
                     nonNegativeNanoseconds(lastCallbackArrival, arrival_in));
        while (arrival_in >=
               callbackArrivalWindowStart + std::chrono::seconds(1))
        {
            counters.callbackArrivalRateNumerator += callbackArrivalWindowCount;
            counters.callbackArrivalRateDenominatorNanoseconds +=
                1'000'000'000U;
            callbackArrivalWindowCount = 0U;
            callbackArrivalWindowStart += std::chrono::seconds(1);
        }
    }
    ++callbackArrivalWindowCount;
    lastCallbackArrival = arrival_in;
}

void RgbdObservability::recordRgbDepthSkewReject()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.rgbDepthSkewRejects;
    --counters.callbacksInFlight;
}

void RgbdObservability::recordCloudImageSkewReject()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.cloudImageSkewRejects;
    --counters.callbacksInFlight;
}

void RgbdObservability::recordNonMonotonicReject()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.nonMonotonicRejects;
    --counters.callbacksInFlight;
}

void RgbdObservability::recordPendingStore(const std::size_t pendingDepth_in,
                                           const bool didOverwritePending_in)
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.pendingStores;
    if (didOverwritePending_in)
    {
        ++counters.pendingOverwrites;
    }
    --counters.callbacksInFlight;
    counters.pendingDepthHighWater =
        std::max(counters.pendingDepthHighWater,
                 static_cast<std::uint64_t>(pendingDepth_in));
}

void RgbdObservability::recordPendingOverflow()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.pendingOverflows;
    --counters.callbacksInFlight;
}

void RgbdObservability::recordImageConversionReject()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.imageConversionRejects;
    if (counters.workersInFlight > 0U)
    {
        --counters.workersInFlight;
    }
}

void RgbdObservability::recordCloudConversionReject()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.cloudConversionRejects;
    if (counters.workersInFlight > 0U)
    {
        --counters.workersInFlight;
    }
}

void RgbdObservability::recordWorkerStart(const SteadyTime callbackArrival_in,
                                          const SteadyTime workerStart_in)
{
    const std::uint64_t queueWaitNanoseconds =
        nonNegativeNanoseconds(callbackArrival_in, workerStart_in);
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.queueWaitSamples;
    ++counters.workersInFlight;
    counters.queueWaitTotalNanoseconds += queueWaitNanoseconds;
    counters.queueWaitMaxNanoseconds =
        std::max(counters.queueWaitMaxNanoseconds, queueWaitNanoseconds);
}

void RgbdObservability::recordTrackCall()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.trackCalls;
}

void RgbdObservability::recordImageConversion(const SteadyTime start_in,
                                              const SteadyTime end_in)
{
    const std::lock_guard<std::mutex> lock(mutex);
    addStageSample(nonNegativeNanoseconds(start_in, end_in),
                   counters.imageConversionSamples,
                   counters.imageConversionTotalNanoseconds,
                   counters.imageConversionMaxNanoseconds);
}

void RgbdObservability::recordCloudPreparation(const SteadyTime start_in,
                                               const SteadyTime end_in)
{
    const std::lock_guard<std::mutex> lock(mutex);
    addStageSample(nonNegativeNanoseconds(start_in, end_in),
                   counters.cloudPreparationSamples,
                   counters.cloudPreparationTotalNanoseconds,
                   counters.cloudPreparationMaxNanoseconds);
}

void RgbdObservability::recordMarkerAssociation(const SteadyTime start_in,
                                                const SteadyTime end_in)
{
    const std::lock_guard<std::mutex> lock(mutex);
    addStageSample(nonNegativeNanoseconds(start_in, end_in),
                   counters.markerAssociationSamples,
                   counters.markerAssociationTotalNanoseconds,
                   counters.markerAssociationMaxNanoseconds);
}

void RgbdObservability::recordTrackDuration(const SteadyTime start_in,
                                            const SteadyTime end_in)
{
    const std::lock_guard<std::mutex> lock(mutex);
    addStageSample(nonNegativeNanoseconds(start_in, end_in),
                   counters.trackDurationSamples,
                   counters.trackDurationTotalNanoseconds,
                   counters.trackDurationMaxNanoseconds);
}

void RgbdObservability::recordPublishTopics(const SteadyTime start_in,
                                            const SteadyTime end_in)
{
    const std::lock_guard<std::mutex> lock(mutex);
    addStageSample(nonNegativeNanoseconds(start_in, end_in),
                   counters.publishTopicsSamples,
                   counters.publishTopicsTotalNanoseconds,
                   counters.publishTopicsMaxNanoseconds);
}

void RgbdObservability::recordPublishTopic(const PublishTopic topic_in,
                                           const bool isActualExecution_in,
                                           const SteadyTime start_in,
                                           const SteadyTime end_in)
{
    const std::uint64_t durationNanoseconds =
        nonNegativeNanoseconds(start_in, end_in);
    const std::lock_guard<std::mutex> lock(mutex);

    if (topic_in == PublishTopic::PLANES && !isActualExecution_in)
    {
        ++counters.publishPlanesCalls;
    }
    if (topic_in == PublishTopic::ALL_POINTS)
    {
        ++counters.publishAllPointsCalls;
    }

    std::uint64_t *p_samples          = nullptr;
    std::uint64_t *p_totalNanoseconds = nullptr;
    std::uint64_t *p_maxNanoseconds   = nullptr;
    switch (topic_in)
    {
    case PublishTopic::ALL_MAPPED_WALLS:
        p_samples          = &counters.publishAllMappedWallsSamples;
        p_totalNanoseconds = &counters.publishAllMappedWallsTotalNanoseconds;
        p_maxNanoseconds   = &counters.publishAllMappedWallsMaxNanoseconds;
        break;
    case PublishTopic::SEGMENTED_CLOUD:
        p_samples          = &counters.publishSegmentedCloudSamples;
        p_totalNanoseconds = &counters.publishSegmentedCloudTotalNanoseconds;
        p_maxNanoseconds   = &counters.publishSegmentedCloudMaxNanoseconds;
        break;
    case PublishTopic::PLANES:
        if (isActualExecution_in)
        {
            p_samples          = &counters.publishPlanesSamples;
            p_totalNanoseconds = &counters.publishPlanesTotalNanoseconds;
            p_maxNanoseconds   = &counters.publishPlanesMaxNanoseconds;
        }
        break;
    case PublishTopic::ALL_POINTS:
        p_samples          = &counters.publishAllPointsSamples;
        p_totalNanoseconds = &counters.publishAllPointsTotalNanoseconds;
        p_maxNanoseconds   = &counters.publishAllPointsMaxNanoseconds;
        break;
    case PublishTopic::TRACKED_POINTS:
        p_samples          = &counters.publishTrackedPointsSamples;
        p_totalNanoseconds = &counters.publishTrackedPointsTotalNanoseconds;
        p_maxNanoseconds   = &counters.publishTrackedPointsMaxNanoseconds;
        break;
    case PublishTopic::FREE_SPACE_CLUSTERS:
        p_samples          = &counters.publishFreeSpaceClustersSamples;
        p_totalNanoseconds = &counters.publishFreeSpaceClustersTotalNanoseconds;
        p_maxNanoseconds   = &counters.publishFreeSpaceClustersMaxNanoseconds;
        break;
    }
    if (p_samples != nullptr)
    {
        addStageSample(durationNanoseconds,
                       *p_samples,
                       *p_totalNanoseconds,
                       *p_maxNanoseconds);
    }
}

namespace
{
void recordPublishTopicCallback(
    void                               *p_context_in,
    const PublishTopic                  topic_in,
    const bool                          isActualExecution_in,
    const RgbdObservability::SteadyTime start_in,
    const RgbdObservability::SteadyTime end_in) noexcept
{
    try
    {
        static_cast<RgbdObservability *>(p_context_in)
            ->recordPublishTopic(topic_in,
                                 isActualExecution_in,
                                 start_in,
                                 end_in);
    }
    catch (...)
    {
        /* Timing is diagnostic only and must never affect the caller. */
    }
}
} /* namespace */

PublishTopicsTimingSink RgbdObservability::publishTopicsTimingSink() noexcept
{
    return {this, &recordPublishTopicCallback};
}

void RgbdObservability::recordTrackCompletion()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.trackCompletions;
}

void RgbdObservability::recordTrackFailure()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.trackFailures;
    if (counters.workersInFlight > 0U)
    {
        --counters.workersInFlight;
    }
}

void RgbdObservability::recordPublishTopicsFailure()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.publishTopicsFailures;
    if (counters.workersInFlight > 0U)
    {
        --counters.workersInFlight;
    }
}

void RgbdObservability::recordShutdownAdmissionReject()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.shutdownAdmissionRejects;
    --counters.callbacksInFlight;
}

void RgbdObservability::recordShutdownPendingDrop()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.shutdownPendingDrops;
}

void RgbdObservability::recordProcessed(
    const std::int64_t sensorTimestampNanoseconds_in,
    const SteadyTime   workerStart_in,
    const SteadyTime   workerEnd_in)
{
    const std::uint64_t processingNanoseconds =
        nonNegativeNanoseconds(workerStart_in, workerEnd_in);
    const std::lock_guard<std::mutex> lock(mutex);

    ++counters.processedPackets;
    if (counters.workersInFlight > 0U)
    {
        --counters.workersInFlight;
    }
    counters.lastProcessedSensorTimestampNanoseconds =
        sensorTimestampNanoseconds_in;
    ++counters.processingDurationSamples;
    counters.processingDurationTotalNanoseconds += processingNanoseconds;
    counters.processingDurationMaxNanoseconds =
        std::max(counters.processingDurationMaxNanoseconds,
                 processingNanoseconds);

    if (hasProcessedSensorTimestamp)
    {
        ++counters.interProcessedSensorGapSamples;
        if (sensorTimestampNanoseconds_in >
            lastProcessedSensorTimestampNanoseconds)
        {
            /* Unsigned subtraction is exact after the signed ordering check,
             * including INT64_MIN to INT64_MAX. */
            const std::uint64_t gapNanoseconds =
                static_cast<std::uint64_t>(sensorTimestampNanoseconds_in) -
                static_cast<std::uint64_t>(
                    lastProcessedSensorTimestampNanoseconds);
            counters.interProcessedSensorGapMaxNanoseconds =
                std::max(counters.interProcessedSensorGapMaxNanoseconds,
                         gapNanoseconds);
        }
    }
    hasProcessedSensorTimestamp             = true;
    lastProcessedSensorTimestampNanoseconds = sensorTimestampNanoseconds_in;

    if (hasProcessedWorkerEnd && workerEnd_in < lastProcessedWorkerEnd)
    {
        ++counters.processedCompletionOutOfOrderCount;
        return;
    }

    if (hasProcessedWorkerEnd)
    {
        const std::uint64_t completionGapNanoseconds =
            nonNegativeNanoseconds(lastProcessedWorkerEnd, workerEnd_in);
        counters.processedCompletionGapMaxNanoseconds =
            std::max(counters.processedCompletionGapMaxNanoseconds,
                     completionGapNanoseconds);
    }

    if (!hasCompletionWindow)
    {
        completionWindowStart = workerEnd_in;
        hasCompletionWindow   = true;
    }

    const std::chrono::seconds windowDuration{1};
    while (workerEnd_in >= completionWindowStart + windowDuration)
    {
        const std::size_t histogramIndex = static_cast<std::size_t>(
            std::min<std::uint64_t>(completionWindowCount,
                                    completionHistogramSize - 1U));
        ++completionWindowHistogram[histogramIndex];
        ++counters.processedTumblingWindowSamples;
        counters.processedCompletionRateNumerator += completionWindowCount;
        counters.processedCompletionRateDenominatorNanoseconds +=
            1'000'000'000U;
        if (counters.processedTumblingWindowSamples == 1U)
        {
            counters.processedTumblingWindowMinimumCount =
                completionWindowCount;
        }
        else
        {
            counters.processedTumblingWindowMinimumCount =
                std::min(counters.processedTumblingWindowMinimumCount,
                         completionWindowCount);
        }
        const std::uint64_t medianRank =
            (counters.processedTumblingWindowSamples + 1U) / 2U;
        std::uint64_t histogramCumulative = 0U;
        for (std::size_t index = 0U; index < completionHistogramSize; ++index)
        {
            histogramCumulative += completionWindowHistogram[index];
            if (histogramCumulative >= medianRank)
            {
                counters.processedTumblingWindowMedianCount = index;
                break;
            }
        }
        completionWindowCount = 0U;
        completionWindowStart += windowDuration;
    }
    ++completionWindowCount;
    hasProcessedWorkerEnd  = true;
    lastProcessedWorkerEnd = workerEnd_in;
}

RgbdObservabilitySnapshot RgbdObservability::snapshot() const
{
    const std::lock_guard<std::mutex> lock(mutex);
    return counters;
}

std::string
    formatRgbdObservabilitySummary(const RgbdObservabilitySnapshot &snapshot_in,
                                   const std::string               &event_in)
{
    std::ostringstream summary;
    summary
        << "VSG_RGBD_OBSERVABILITY" << " event=" << event_in
        << " boundary=grab_rgbd_synchronized_triples"
        << " pre_boundary_drops=unknown"
        << " callback_admissions=" << snapshot_in.callbackAdmissions
        << " callbacks_in_flight=" << snapshot_in.callbacksInFlight
        << " workers_in_flight=" << snapshot_in.workersInFlight
        << " rgb_depth_skew_rejects=" << snapshot_in.rgbDepthSkewRejects
        << " cloud_image_skew_rejects=" << snapshot_in.cloudImageSkewRejects
        << " non_monotonic_rejects=" << snapshot_in.nonMonotonicRejects
        << " pending_stores=" << snapshot_in.pendingStores
        << " pending_overwrites=" << snapshot_in.pendingOverwrites
        << " pending_overflows=" << snapshot_in.pendingOverflows
        << " image_conversion_rejects=" << snapshot_in.imageConversionRejects
        << " cloud_conversion_rejects=" << snapshot_in.cloudConversionRejects
        << " track_calls=" << snapshot_in.trackCalls
        << " track_completions=" << snapshot_in.trackCompletions
        << " track_failures=" << snapshot_in.trackFailures
        << " processed_packets=" << snapshot_in.processedPackets
        << " publish_topics_failures=" << snapshot_in.publishTopicsFailures
        << " shutdown_admission_rejects="
        << snapshot_in.shutdownAdmissionRejects
        << " shutdown_pending_drops=" << snapshot_in.shutdownPendingDrops
        << " queue_wait_samples=" << snapshot_in.queueWaitSamples
        << " queue_wait_total_ns=" << snapshot_in.queueWaitTotalNanoseconds
        << " queue_wait_max_ns=" << snapshot_in.queueWaitMaxNanoseconds
        << " processing_duration_samples="
        << snapshot_in.processingDurationSamples
        << " processing_duration_total_ns="
        << snapshot_in.processingDurationTotalNanoseconds
        << " processing_duration_max_ns="
        << snapshot_in.processingDurationMaxNanoseconds
        << " inter_processed_sensor_gap_samples="
        << snapshot_in.interProcessedSensorGapSamples
        << " inter_processed_sensor_gap_max_ns="
        << snapshot_in.interProcessedSensorGapMaxNanoseconds
        << " pending_depth_high_water=" << snapshot_in.pendingDepthHighWater
        << " image_conversion_samples=" << snapshot_in.imageConversionSamples
        << " image_conversion_total_ns="
        << snapshot_in.imageConversionTotalNanoseconds
        << " image_conversion_max_ns="
        << snapshot_in.imageConversionMaxNanoseconds
        << " cloud_preparation_samples=" << snapshot_in.cloudPreparationSamples
        << " cloud_preparation_total_ns="
        << snapshot_in.cloudPreparationTotalNanoseconds
        << " cloud_preparation_max_ns="
        << snapshot_in.cloudPreparationMaxNanoseconds
        << " marker_association_samples="
        << snapshot_in.markerAssociationSamples
        << " marker_association_total_ns="
        << snapshot_in.markerAssociationTotalNanoseconds
        << " marker_association_max_ns="
        << snapshot_in.markerAssociationMaxNanoseconds
        << " track_duration_samples=" << snapshot_in.trackDurationSamples
        << " track_duration_total_ns="
        << snapshot_in.trackDurationTotalNanoseconds
        << " track_duration_max_ns=" << snapshot_in.trackDurationMaxNanoseconds
        << " publish_topics_samples=" << snapshot_in.publishTopicsSamples
        << " publish_topics_total_ns="
        << snapshot_in.publishTopicsTotalNanoseconds
        << " publish_topics_max_ns=" << snapshot_in.publishTopicsMaxNanoseconds
        << " publish_all_mapped_walls_samples="
        << snapshot_in.publishAllMappedWallsSamples
        << " publish_all_mapped_walls_total_ns="
        << snapshot_in.publishAllMappedWallsTotalNanoseconds
        << " publish_all_mapped_walls_max_ns="
        << snapshot_in.publishAllMappedWallsMaxNanoseconds
        << " publish_segmented_cloud_samples="
        << snapshot_in.publishSegmentedCloudSamples
        << " publish_segmented_cloud_total_ns="
        << snapshot_in.publishSegmentedCloudTotalNanoseconds
        << " publish_segmented_cloud_max_ns="
        << snapshot_in.publishSegmentedCloudMaxNanoseconds
        << " publish_planes_calls=" << snapshot_in.publishPlanesCalls
        << " publish_planes_samples=" << snapshot_in.publishPlanesSamples
        << " publish_planes_total_ns="
        << snapshot_in.publishPlanesTotalNanoseconds
        << " publish_planes_max_ns=" << snapshot_in.publishPlanesMaxNanoseconds
        << " publish_all_points_calls=" << snapshot_in.publishAllPointsCalls
        << " publish_all_points_samples=" << snapshot_in.publishAllPointsSamples
        << " publish_all_points_total_ns="
        << snapshot_in.publishAllPointsTotalNanoseconds
        << " publish_all_points_max_ns="
        << snapshot_in.publishAllPointsMaxNanoseconds
        << " publish_tracked_points_samples="
        << snapshot_in.publishTrackedPointsSamples
        << " publish_tracked_points_total_ns="
        << snapshot_in.publishTrackedPointsTotalNanoseconds
        << " publish_tracked_points_max_ns="
        << snapshot_in.publishTrackedPointsMaxNanoseconds
        << " publish_free_space_clusters_samples="
        << snapshot_in.publishFreeSpaceClustersSamples
        << " publish_free_space_clusters_total_ns="
        << snapshot_in.publishFreeSpaceClustersTotalNanoseconds
        << " publish_free_space_clusters_max_ns="
        << snapshot_in.publishFreeSpaceClustersMaxNanoseconds
        << " processed_completion_gap_max_ns="
        << snapshot_in.processedCompletionGapMaxNanoseconds
        << " processed_completion_out_of_order_count="
        << snapshot_in.processedCompletionOutOfOrderCount
        << " processed_tumbling_window_samples="
        << snapshot_in.processedTumblingWindowSamples
        << " processed_tumbling_window_min_count="
        << snapshot_in.processedTumblingWindowMinimumCount
        << " processed_tumbling_window_median_count="
        << snapshot_in.processedTumblingWindowMedianCount
        << " callback_arrival_gap_samples="
        << snapshot_in.callbackArrivalGapSamples
        << " callback_arrival_gap_max_ns="
        << snapshot_in.callbackArrivalGapMaxNanoseconds
        << " callback_arrival_rate_numerator="
        << snapshot_in.callbackArrivalRateNumerator
        << " callback_arrival_rate_denominator_ns="
        << snapshot_in.callbackArrivalRateDenominatorNanoseconds
        << " processed_completion_rate_numerator="
        << snapshot_in.processedCompletionRateNumerator
        << " processed_completion_rate_denominator_ns="
        << snapshot_in.processedCompletionRateDenominatorNanoseconds
        << " last_processed_sensor_timestamp_ns="
        << snapshot_in.lastProcessedSensorTimestampNanoseconds;
    return summary.str();
}

} /* namespace vs_graphs::observability */
