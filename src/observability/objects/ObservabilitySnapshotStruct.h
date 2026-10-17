/*!
 * @file            ObservabilitySnapshotStruct.h
 *
 * @brief           Declares cumulative RGB-D callback and worker counters.
 *
 * @date            09/10/2026
 */

#ifndef HPP_OBSERVABILITYSNAPSHOT_HPP
#define HPP_OBSERVABILITYSNAPSHOT_HPP

/* Function Includes */
/* None */

/* Object Include */
/* None */

/* Data include */
#include <cstdint>

/* Generic Libraries */
/* None */

namespace vs_graphs::observability
{

/*!
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
    /*!
     * @brief           Number of synchronized triples that entered GrabRGBD,
     *                  counted before any check.
     */
    std::uint64_t callbackAdmissions{0U};

    /*!
     * @brief           Triples admitted by GrabRGBD whose fate (rejected or
     *                  stored as pending) is not recorded yet.
     */
    std::uint64_t callbacksInFlight{0U};

    /*!
     * @brief           Packets the worker thread has started but not yet
     *                  finished, rejected or failed.
     */
    std::uint64_t workersInFlight{0U};

    /*!
     * @brief           Triples dropped because RGB and depth stamps differed by
     *                  more than the allowed skew.
     */
    std::uint64_t rgbDepthSkewRejects{0U};

    /*!
     * @brief           Triples dropped because the point-cloud stamp was too
     *                  far from the image stamps.
     */
    std::uint64_t cloudImageSkewRejects{0U};

    /*!
     * @brief           Triples dropped because the RGB stamp was not newer than
     *                  the last admitted one.
     */
    std::uint64_t nonMonotonicRejects{0U};

    /*!
     * @brief           Number of triples stored as the pending packet for the
     *                  worker thread.
     */
    std::uint64_t pendingStores{0U};

    /*!
     * @brief           Stores that replaced a pending packet the worker had not
     *                  taken yet.
     */
    std::uint64_t pendingOverwrites{0U};

    /*!
     * @brief           Triples dropped because the pending queue was full.
     */
    std::uint64_t pendingOverflows{0U};

    /*!
     * @brief           Packets the worker dropped because the cv_bridge image
     *                  conversion failed.
     */
    std::uint64_t imageConversionRejects{0U};

    /*!
     * @brief           Packets the worker dropped because the point-cloud
     *                  preparation failed.
     */
    std::uint64_t cloudConversionRejects{0U};

    /*!
     * @brief           Number of times the worker called the SLAM tracker.
     */
    std::uint64_t trackCalls{0U};

    /*!
     * @brief           Number of tracker calls that returned normally.
     */
    std::uint64_t trackCompletions{0U};

    /*!
     * @brief           Number of tracker calls that ended with an exception.
     */
    std::uint64_t trackFailures{0U};

    /*!
     * @brief           Packets the worker finished completely, including
     *                  publication.
     */
    std::uint64_t processedPackets{0U};

    /*!
     * @brief           Packets whose topic publication ended with an exception.
     */
    std::uint64_t publishTopicsFailures{0U};

    /*!
     * @brief           Triples dropped by GrabRGBD because shutdown had been
     *                  requested.
     */
    std::uint64_t shutdownAdmissionRejects{0U};

    /*!
     * @brief           Pending packets discarded by the worker when shutdown
     *                  was requested.
     */
    std::uint64_t shutdownPendingDrops{0U};

    /*!
     * @brief           Number of packets for which the wait before the worker
     *                  started was recorded.
     */
    std::uint64_t queueWaitSamples{0U};

    /*!
     * @brief           Number of processed packets whose worker duration was
     *                  recorded.
     */
    std::uint64_t processingDurationSamples{0U};

    /*!
     * @brief           Number of consecutive processed-packet pairs compared by
     *                  sensor timestamp.
     */
    std::uint64_t interProcessedSensorGapSamples{0U};

    /*!
     * @brief           Sum of the times packets waited from GrabRGBD arrival to
     *                  worker start, nanoseconds.
     */
    std::uint64_t queueWaitTotalNanoseconds{0U};

    /*!
     * @brief           Longest wait from GrabRGBD arrival to worker start,
     *                  nanoseconds.
     */
    std::uint64_t queueWaitMaxNanoseconds{0U};

    /*!
     * @brief           Sum of worker durations (start to end) of processed
     *                  packets, nanoseconds.
     */
    std::uint64_t processingDurationTotalNanoseconds{0U};

    /*!
     * @brief           Longest worker duration of a processed packet,
     *                  nanoseconds.
     */
    std::uint64_t processingDurationMaxNanoseconds{0U};

    /*!
     * @brief           Largest increase in sensor timestamp between consecutive
     *                  processed packets, nanoseconds.
     */
    std::uint64_t interProcessedSensorGapMaxNanoseconds{0U};

    /*!
     * @brief           Largest pending depth reported when storing a packet.
     */
    std::uint64_t pendingDepthHighWater{0U};

    /*!
     * @brief           Number of cv_bridge image conversion timings recorded.
     */
    std::uint64_t imageConversionSamples{0U};

    /*!
     * @brief           Sum of all cv_bridge image conversion durations,
     *                  nanoseconds.
     */
    std::uint64_t imageConversionTotalNanoseconds{0U};

    /*!
     * @brief           Longest single cv_bridge image conversion duration,
     *                  nanoseconds.
     */
    std::uint64_t imageConversionMaxNanoseconds{0U};

    /*!
     * @brief           Number of point-cloud preparation timings recorded.
     */
    std::uint64_t cloudPreparationSamples{0U};

    /*!
     * @brief           Sum of all point-cloud preparation durations,
     *                  nanoseconds.
     */
    std::uint64_t cloudPreparationTotalNanoseconds{0U};

    /*!
     * @brief           Longest single point-cloud preparation duration,
     *                  nanoseconds.
     */
    std::uint64_t cloudPreparationMaxNanoseconds{0U};

    /*!
     * @brief           Number of marker association timings recorded.
     */
    std::uint64_t markerAssociationSamples{0U};

    /*!
     * @brief           Sum of all marker association durations, nanoseconds.
     */
    std::uint64_t markerAssociationTotalNanoseconds{0U};

    /*!
     * @brief           Longest single marker association duration, nanoseconds.
     */
    std::uint64_t markerAssociationMaxNanoseconds{0U};

    /*!
     * @brief           Number of tracker call timings recorded.
     */
    std::uint64_t trackDurationSamples{0U};

    /*!
     * @brief           Sum of all tracker call durations, nanoseconds.
     */
    std::uint64_t trackDurationTotalNanoseconds{0U};

    /*!
     * @brief           Longest single tracker call duration, nanoseconds.
     */
    std::uint64_t trackDurationMaxNanoseconds{0U};

    /*!
     * @brief           Number of publishTopics call timings recorded.
     */
    std::uint64_t publishTopicsSamples{0U};

    /*!
     * @brief           Sum of all publishTopics call durations, nanoseconds.
     */
    std::uint64_t publishTopicsTotalNanoseconds{0U};

    /*!
     * @brief           Longest single publishTopics call duration, nanoseconds.
     */
    std::uint64_t publishTopicsMaxNanoseconds{0U};

    /*!
     * @brief           Number of mapped-wall publication timings recorded.
     */
    std::uint64_t publishAllMappedWallsSamples{0U};

    /*!
     * @brief           Sum of all mapped-wall publication durations,
     *                  nanoseconds.
     */
    std::uint64_t publishAllMappedWallsTotalNanoseconds{0U};

    /*!
     * @brief           Longest single mapped-wall publication duration,
     *                  nanoseconds.
     */
    std::uint64_t publishAllMappedWallsMaxNanoseconds{0U};

    /*!
     * @brief           Number of segmented-cloud publication timings recorded.
     */
    std::uint64_t publishSegmentedCloudSamples{0U};

    /*!
     * @brief           Sum of all segmented-cloud publication durations,
     *                  nanoseconds.
     */
    std::uint64_t publishSegmentedCloudTotalNanoseconds{0U};

    /*!
     * @brief           Longest single segmented-cloud publication duration,
     *                  nanoseconds.
     */
    std::uint64_t publishSegmentedCloudMaxNanoseconds{0U};

    /*!
     * @brief           Times the plane publishing helper was invoked; counts
     *                  calls, not work done.
     */
    std::uint64_t publishPlanesCalls{0U};

    /*!
     * @brief           Plane publications that did work past the helper gates
     *                  and were timed.
     */
    std::uint64_t publishPlanesSamples{0U};

    /*!
     * @brief           Sum of all plane publication durations, nanoseconds.
     */
    std::uint64_t publishPlanesTotalNanoseconds{0U};

    /*!
     * @brief           Longest single plane publication duration, nanoseconds.
     */
    std::uint64_t publishPlanesMaxNanoseconds{0U};

    /*!
     * @brief           Number of all-points timings recorded, including zero-
     *                  length ones reported when the publication was skipped.
     */
    std::uint64_t publishAllPointsSamples{0U};

    /*!
     * @brief           Times the all-points publication was reported, whether
     *                  or not it ran.
     */
    std::uint64_t publishAllPointsCalls{0U};

    /*!
     * @brief           Sum of all all-points publication durations,
     *                  nanoseconds.
     */
    std::uint64_t publishAllPointsTotalNanoseconds{0U};

    /*!
     * @brief           Longest single all-points publication duration,
     *                  nanoseconds.
     */
    std::uint64_t publishAllPointsMaxNanoseconds{0U};

    /*!
     * @brief           Number of tracked-points publication timings recorded.
     */
    std::uint64_t publishTrackedPointsSamples{0U};

    /*!
     * @brief           Sum of all tracked-points publication durations,
     *                  nanoseconds.
     */
    std::uint64_t publishTrackedPointsTotalNanoseconds{0U};

    /*!
     * @brief           Longest single tracked-points publication duration,
     *                  nanoseconds.
     */
    std::uint64_t publishTrackedPointsMaxNanoseconds{0U};

    /*!
     * @brief           Number of free-space-cluster publication timings
     *                  recorded.
     */
    std::uint64_t publishFreeSpaceClustersSamples{0U};

    /*!
     * @brief           Sum of all free-space-cluster publication durations,
     *                  nanoseconds.
     */
    std::uint64_t publishFreeSpaceClustersTotalNanoseconds{0U};

    /*!
     * @brief           Longest single free-space-cluster publication duration,
     *                  nanoseconds.
     */
    std::uint64_t publishFreeSpaceClustersMaxNanoseconds{0U};

    /*!
     * @brief           Longest steady-time gap between the ends of consecutive
     *                  processed packets, nanoseconds.
     */
    std::uint64_t processedCompletionGapMaxNanoseconds{0U};

    /*!
     * @brief           Processed packets whose worker end was earlier than the
     *                  previous one and were left out of the windows.
     */
    std::uint64_t processedCompletionOutOfOrderCount{0U};

    /*!
     * @brief           Number of completed one-second windows of
     *                  processed-packet counts.
     */
    std::uint64_t processedTumblingWindowSamples{0U};

    /*!
     * @brief           Fewest processed packets in any completed one-second
     *                  window.
     */
    std::uint64_t processedTumblingWindowMinimumCount{0U};

    /*!
     * @brief           Lower median of processed packets per completed
     *                  one-second window; counts above 1000 read as 1001.
     */
    std::uint64_t processedTumblingWindowMedianCount{0U};

    /*!
     * @brief           Number of consecutive GrabRGBD arrival pairs compared.
     */
    std::uint64_t callbackArrivalGapSamples{0U};

    /*!
     * @brief           Longest steady-time gap between consecutive GrabRGBD
     *                  arrivals, nanoseconds.
     */
    std::uint64_t callbackArrivalGapMaxNanoseconds{0U};

    /*!
     * @brief           Arrivals in completed one-second windows; divide by the
     *                  denominator for the rate. An incomplete trailing window
     *                  is excluded.
     */
    std::uint64_t callbackArrivalRateNumerator{0U};

    /*!
     * @brief           Total length of the completed arrival windows,
     *                  nanoseconds (one second each).
     */
    std::uint64_t callbackArrivalRateDenominatorNanoseconds{0U};

    /*!
     * @brief           Processed packets in completed one-second windows;
     *                  divide by the denominator for the rate. An incomplete
     *                  trailing window is excluded.
     */
    std::uint64_t processedCompletionRateNumerator{0U};

    /*!
     * @brief           Total length of the completed processing windows,
     *                  nanoseconds (one second each).
     */
    std::uint64_t processedCompletionRateDenominatorNanoseconds{0U};

    /*!
     * @brief           Sensor (RGB header) timestamp of the latest processed
     *                  packet, nanoseconds on the ROS clock; 0 before the
     *                  first.
     */
    std::int64_t lastProcessedSensorTimestampNanoseconds{0};
};
} /* namespace vs_graphs::observability */

#endif /* HPP_OBSERVABILITYSNAPSHOT_HPP */
