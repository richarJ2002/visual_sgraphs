/**
 * @file RgbdObservability.h
 * @brief Declares package-private RGB-D callback/worker accounting.
 */

#ifndef VS_GRAPHS_RGBD_OBSERVABILITY_H
#define VS_GRAPHS_RGBD_OBSERVABILITY_H

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "../include/PublishTopicsTiming.h"

namespace vs_graphs::observability
{

/**
 * Cumulative observations at the GrabRGBD callback boundary. Processed-window
 * statistics use one-second, non-overlapping tumbling windows anchored at the
 * first fully processed packet's worker-end time. A window is counted only
 * after steady time reaches its end; therefore an incomplete trailing window
 * is excluded. Empty elapsed windows are counted as zero. The median is the
 * lower median, and the bounded histogram saturates counts above 1000 into its
 * final bin. These tumbling statistics are diagnostic evidence and do not
 * claim that every possible sliding one-second window passed.
 */
struct RgbdObservabilitySnapshot
{
    std::uint64_t callbackAdmissions{0U};
    std::uint64_t callbacksInFlight{0U};
    std::uint64_t workersInFlight{0U};
    std::uint64_t rgbDepthSkewRejects{0U};
    std::uint64_t cloudImageSkewRejects{0U};
    std::uint64_t nonMonotonicRejects{0U};
    std::uint64_t pendingStores{0U};
    std::uint64_t pendingOverwrites{0U};
    std::uint64_t pendingOverflows{0U};
    std::uint64_t imageConversionRejects{0U};
    std::uint64_t cloudConversionRejects{0U};
    std::uint64_t trackCalls{0U};
    std::uint64_t trackCompletions{0U};
    std::uint64_t trackFailures{0U};
    std::uint64_t processedPackets{0U};
    std::uint64_t publishTopicsFailures{0U};
    std::uint64_t shutdownAdmissionRejects{0U};
    std::uint64_t shutdownPendingDrops{0U};
    std::uint64_t queueWaitSamples{0U};
    std::uint64_t processingDurationSamples{0U};
    std::uint64_t interProcessedSensorGapSamples{0U};
    std::uint64_t queueWaitTotalNanoseconds{0U};
    std::uint64_t queueWaitMaxNanoseconds{0U};
    std::uint64_t processingDurationTotalNanoseconds{0U};
    std::uint64_t processingDurationMaxNanoseconds{0U};
    std::uint64_t interProcessedSensorGapMaxNanoseconds{0U};
    std::uint64_t pendingDepthHighWater{0U};
    std::uint64_t imageConversionSamples{0U};
    std::uint64_t imageConversionTotalNanoseconds{0U};
    std::uint64_t imageConversionMaxNanoseconds{0U};
    std::uint64_t cloudPreparationSamples{0U};
    std::uint64_t cloudPreparationTotalNanoseconds{0U};
    std::uint64_t cloudPreparationMaxNanoseconds{0U};
    std::uint64_t markerAssociationSamples{0U};
    std::uint64_t markerAssociationTotalNanoseconds{0U};
    std::uint64_t markerAssociationMaxNanoseconds{0U};
    std::uint64_t trackDurationSamples{0U};
    std::uint64_t trackDurationTotalNanoseconds{0U};
    std::uint64_t trackDurationMaxNanoseconds{0U};
    std::uint64_t publishTopicsSamples{0U};
    std::uint64_t publishTopicsTotalNanoseconds{0U};
    std::uint64_t publishTopicsMaxNanoseconds{0U};
    std::uint64_t publishAllMappedWallsSamples{0U};
    std::uint64_t publishAllMappedWallsTotalNanoseconds{0U};
    std::uint64_t publishAllMappedWallsMaxNanoseconds{0U};
    std::uint64_t publishSegmentedCloudSamples{0U};
    std::uint64_t publishSegmentedCloudTotalNanoseconds{0U};
    std::uint64_t publishSegmentedCloudMaxNanoseconds{0U};
    /* Calls count helper invocations; samples count work past helper gates. */
    std::uint64_t publishPlanesCalls{0U};
    std::uint64_t publishPlanesSamples{0U};
    std::uint64_t publishPlanesTotalNanoseconds{0U};
    std::uint64_t publishPlanesMaxNanoseconds{0U};
    std::uint64_t publishAllPointsSamples{0U};
    std::uint64_t publishAllPointsCalls{0U};
    std::uint64_t publishAllPointsTotalNanoseconds{0U};
    std::uint64_t publishAllPointsMaxNanoseconds{0U};
    std::uint64_t publishTrackedPointsSamples{0U};
    std::uint64_t publishTrackedPointsTotalNanoseconds{0U};
    std::uint64_t publishTrackedPointsMaxNanoseconds{0U};
    std::uint64_t publishFreeSpaceClustersSamples{0U};
    std::uint64_t publishFreeSpaceClustersTotalNanoseconds{0U};
    std::uint64_t publishFreeSpaceClustersMaxNanoseconds{0U};
    std::uint64_t processedCompletionGapMaxNanoseconds{0U};
    std::uint64_t processedCompletionOutOfOrderCount{0U};
    std::uint64_t processedTumblingWindowSamples{0U};
    std::uint64_t processedTumblingWindowMinimumCount{0U};
    std::uint64_t processedTumblingWindowMedianCount{0U};
    /* Rate numerators and denominators cover completed one-second steady-time
     * windows only; an incomplete trailing window is intentionally excluded. */
    std::uint64_t callbackArrivalGapSamples{0U};
    std::uint64_t callbackArrivalGapMaxNanoseconds{0U};
    std::uint64_t callbackArrivalRateNumerator{0U};
    std::uint64_t callbackArrivalRateDenominatorNanoseconds{0U};
    std::uint64_t processedCompletionRateNumerator{0U};
    std::uint64_t processedCompletionRateDenominatorNanoseconds{0U};
    std::int64_t  lastProcessedSensorTimestampNanoseconds{0};
};

/**
 * Thread-safe cumulative accounting for synchronized triples which have
 * entered ImageGrabber::GrabRGBD. DDS and message_filters losses before that
 * function are outside this boundary and remain unknown.
 */
class RgbdObservability
{
  public:
    using SteadyTime = std::chrono::steady_clock::time_point;

    RgbdObservability()                                     = default;
    ~RgbdObservability()                                    = default;
    RgbdObservability(const RgbdObservability &)            = delete;
    RgbdObservability &operator=(const RgbdObservability &) = delete;

    void recordCallbackAdmission();
    void recordCallbackAdmission(SteadyTime arrival_in);
    void recordRgbDepthSkewReject();
    void recordCloudImageSkewReject();
    void recordNonMonotonicReject();
    void recordPendingStore(std::size_t pendingDepth_in,
                            bool        didOverwritePending_in = false);
    void recordPendingOverflow();
    void recordImageConversionReject();
    void recordCloudConversionReject();
    void recordWorkerStart(SteadyTime callbackArrival_in,
                           SteadyTime workerStart_in);
    void recordImageConversion(SteadyTime start_in, SteadyTime end_in);
    void recordCloudPreparation(SteadyTime start_in, SteadyTime end_in);
    void recordMarkerAssociation(SteadyTime start_in, SteadyTime end_in);
    void recordTrackDuration(SteadyTime start_in, SteadyTime end_in);
    void recordPublishTopics(SteadyTime start_in, SteadyTime end_in);
    void recordPublishTopic(PublishTopic topic_in,
                            bool         isActualExecution_in,
                            SteadyTime   start_in,
                            SteadyTime   end_in);
    [[nodiscard]] PublishTopicsTimingSink publishTopicsTimingSink() noexcept;
    void                                  recordTrackCall();
    void                                  recordTrackCompletion();
    void                                  recordTrackFailure();
    void                                  recordPublishTopicsFailure();
    void                                  recordShutdownAdmissionReject();
    void                                  recordShutdownPendingDrop();
    void recordProcessed(std::int64_t sensorTimestampNanoseconds_in,
                         SteadyTime   workerStart_in,
                         SteadyTime   workerEnd_in);
    [[nodiscard]] RgbdObservabilitySnapshot snapshot() const;

  private:
    mutable std::mutex           mutex;
    RgbdObservabilitySnapshot    counters;
    bool                         hasProcessedSensorTimestamp{false};
    std::int64_t                 lastProcessedSensorTimestampNanoseconds{0};
    /* Production has one worker, so processed worker-end times are ordered. */
    bool                         hasProcessedWorkerEnd{false};
    SteadyTime                   lastProcessedWorkerEnd{};
    bool                         hasCompletionWindow{false};
    SteadyTime                   completionWindowStart{};
    std::uint64_t                completionWindowCount{0U};
    static constexpr std::size_t completionHistogramSize{1002U};
    std::array<std::uint64_t, completionHistogramSize>
                  completionWindowHistogram{};
    bool          hasCallbackArrivalWindow{false};
    SteadyTime    callbackArrivalWindowStart{};
    SteadyTime    lastCallbackArrival{};
    std::uint64_t callbackArrivalWindowCount{0U};
};

/** Formats one stable, single-line, key-value summary. */
[[nodiscard]] std::string
    formatRgbdObservabilitySummary(const RgbdObservabilitySnapshot &snapshot_in,
                                   const std::string               &event_in);

} /* namespace vs_graphs::observability */

#endif /* VS_GRAPHS_RGBD_OBSERVABILITY_H */
