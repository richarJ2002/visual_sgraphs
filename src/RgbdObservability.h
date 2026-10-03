/*!
 * @file            RgbdObservability.h
 *
 * @brief           Declares package-private RGB-D callback/worker accounting.
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
    std::int64_t  lastProcessedSensorTimestampNanoseconds{0};
};

/*!
 * Thread-safe cumulative accounting for synchronized triples which have
 * entered ImageGrabber::GrabRGBD. DDS and message_filters losses before that
 * function are outside this boundary and remain unknown.
 */
class RgbdObservability
{
  public:
    /*!
     * @brief           Monotonic clock instant used for every timing in this
     *                  class.
     */
    using SteadyTime = std::chrono::steady_clock::time_point;

    RgbdObservability()                                     = default;
    ~RgbdObservability()                                    = default;
    RgbdObservability(const RgbdObservability &)            = delete;
    RgbdObservability &operator=(const RgbdObservability &) = delete;

    /*!
     * @brief           Records a triple entering GrabRGBD, stamped with the
     *                  current steady time. Called from the executor thread
     *                  running GrabRGBD.
     */
    void recordCallbackAdmission();
    /*!
     * @brief           Records a triple entering GrabRGBD at a given arrival
     *                  time: counts it as admitted and in flight, and updates
     *                  the arrival gap and one-second arrival windows.
     *
     * @param[in]       arrival_in
     *                  Steady-clock arrival time of the triple.
     */
    void recordCallbackAdmission(SteadyTime arrival_in);
    /*!
     * @brief           Records an admitted triple dropped for RGB-depth
     *                  timestamp skew; it is no longer in flight. Called from
     *                  GrabRGBD.
     */
    void recordRgbDepthSkewReject();
    /*!
     * @brief           Records an admitted triple dropped for point-cloud-to-
     *                  image timestamp skew; it is no longer in flight. Called
     *                  from GrabRGBD.
     */
    void recordCloudImageSkewReject();
    /*!
     * @brief           Records an admitted triple dropped for a non-increasing
     *                  RGB timestamp; it is no longer in flight. Called from
     *                  GrabRGBD.
     */
    void recordNonMonotonicReject();
    /*!
     * @brief           Records an admitted triple stored as the pending packet;
     *                  it is no longer in flight. Called from GrabRGBD.
     *
     * @param[in]       pendingDepth_in
     *                  Number of packets waiting after the store; raises the
     *                  pending-depth high-water mark if larger.
     *
     * @param[in]       didOverwritePending_in
     *                  True when the store replaced a pending packet the worker
     *                  had not taken.
     */
    void recordPendingStore(std::size_t pendingDepth_in,
                            bool        didOverwritePending_in = false);
    /*!
     * @brief           Records an admitted triple dropped because the pending
     *                  queue was full; it is no longer in flight. The RGB-D
     *                  node keeps one replaceable pending slot and does not
     *                  call this.
     */
    void recordPendingOverflow();
    /*!
     * @brief           Records a started packet dropped after image conversion
     *                  failed; the worker is no longer busy. Called from the
     *                  worker thread.
     */
    void recordImageConversionReject();
    /*!
     * @brief           Records a started packet dropped after point-cloud
     *                  preparation failed; the worker is no longer busy. Called
     *                  from the worker thread.
     */
    void recordCloudConversionReject();
    /*!
     * @brief           Records the worker taking a packet: counts it as a
     *                  worker in flight and adds its queue wait (a negative
     *                  wait counts as zero). Called from the worker thread.
     *
     * @param[in]       callbackArrival_in
     *                  Steady-clock time the packet entered GrabRGBD.
     *
     * @param[in]       workerStart_in
     *                  Steady-clock time the worker started the packet.
     */
    void recordWorkerStart(SteadyTime callbackArrival_in,
                           SteadyTime workerStart_in);
    /*!
     * @brief           Adds one cv_bridge image-conversion timing. A negative
     *                  duration counts as zero. Called from the worker thread.
     *
     * @param[in]       start_in
     *                  Steady-clock start of the stage.
     *
     * @param[in]       end_in
     *                  Steady-clock end of the stage.
     */
    void recordImageConversion(SteadyTime start_in, SteadyTime end_in);
    /*!
     * @brief           Adds one point-cloud-preparation timing. A negative
     *                  duration counts as zero. Called from the worker thread.
     *
     * @param[in]       start_in
     *                  Steady-clock start of the stage.
     *
     * @param[in]       end_in
     *                  Steady-clock end of the stage.
     */
    void recordCloudPreparation(SteadyTime start_in, SteadyTime end_in);
    /*!
     * @brief           Adds one marker-association timing. A negative duration
     *                  counts as zero. Called from the worker thread.
     *
     * @param[in]       start_in
     *                  Steady-clock start of the stage.
     *
     * @param[in]       end_in
     *                  Steady-clock end of the stage.
     */
    void recordMarkerAssociation(SteadyTime start_in, SteadyTime end_in);
    /*!
     * @brief           Adds one tracker-call timing. A negative duration counts
     *                  as zero. Called from the worker thread.
     *
     * @param[in]       start_in
     *                  Steady-clock start of the stage.
     *
     * @param[in]       end_in
     *                  Steady-clock end of the stage.
     */
    void recordTrackDuration(SteadyTime start_in, SteadyTime end_in);
    /*!
     * @brief           Adds one timing of the whole publishTopics call. A
     *                  negative duration counts as zero. Called from the worker
     *                  thread.
     *
     * @param[in]       start_in
     *                  Steady-clock start of the stage.
     *
     * @param[in]       end_in
     *                  Steady-clock end of the stage.
     */
    void recordPublishTopics(SteadyTime start_in, SteadyTime end_in);
    /*!
     * @brief           Adds one timing for a single published topic, taken from
     *                  the publishTopics timing sink. For the planes topic a
     *                  non-executed report only counts a helper call; every
     *                  other topic is timed whenever reported.
     *
     * @param[in]       topic_in
     *                  Topic the timing belongs to.
     *
     * @param[in]       isActualExecution_in
     *                  True when the publication really ran; false for a call-
     *                  count-only report.
     *
     * @param[in]       start_in
     *                  Steady-clock start of the publication.
     *
     * @param[in]       end_in
     *                  Steady-clock end of the publication.
     */
    void recordPublishTopic(PublishTopic topic_in,
                            bool         isActualExecution_in,
                            SteadyTime   start_in,
                            SteadyTime   end_in);
    /*!
     * @brief           Returns a timing sink that forwards each report to
     *                  recordPublishTopic on this object. The sink borrows this
     *                  object, which must outlive it; exceptions from recording
     *                  are swallowed.
     *
     * @return          Sink whose context is this object.
     */
    [[nodiscard]] PublishTopicsTimingSink publishTopicsTimingSink() noexcept;
    /*!
     * @brief           Records one call to the SLAM tracker. Called from the
     *                  worker thread.
     */
    void                                  recordTrackCall();
    /*!
     * @brief           Records a tracker call that returned normally. Called
     *                  from the worker thread.
     */
    void                                  recordTrackCompletion();
    /*!
     * @brief           Records a tracker call that ended with an exception; the
     *                  worker is no longer busy. Called from the worker thread.
     */
    void                                  recordTrackFailure();
    /*!
     * @brief           Records a publishTopics call that ended with an
     *                  exception; the worker is no longer busy. Called from the
     *                  worker thread.
     */
    void                                  recordPublishTopicsFailure();
    /*!
     * @brief           Records an admitted triple dropped because shutdown was
     *                  requested; it is no longer in flight. Called from
     *                  GrabRGBD.
     */
    void                                  recordShutdownAdmissionReject();
    /*!
     * @brief           Records a pending packet discarded by the worker at
     *                  shutdown. Called from the worker thread.
     */
    void                                  recordShutdownPendingDrop();
    /*!
     * @brief           Records a fully processed packet: the worker is no
     *                  longer busy, its duration and sensor-timestamp gap are
     *                  added, and it is counted in the one-second completion
     *                  windows. A worker end earlier than the previous one is
     *                  counted as out of order and skipped for the windows.
     *                  Called from the worker thread.
     *
     * @param[in]       sensorTimestampNanoseconds_in
     *                  Sensor (RGB header) timestamp of the packet, nanoseconds
     *                  on the ROS clock.
     *
     * @param[in]       workerStart_in
     *                  Steady-clock time the worker started the packet.
     *
     * @param[in]       workerEnd_in
     *                  Steady-clock time the worker finished the packet.
     */
    void recordProcessed(std::int64_t sensorTimestampNanoseconds_in,
                         SteadyTime   workerStart_in,
                         SteadyTime   workerEnd_in);
    /*!
     * @brief           Returns a copy of the cumulative counters, taken under
     *                  the lock. Safe to call from any thread.
     *
     * @return          Copy of the counters at the time of the call.
     */
    [[nodiscard]] RgbdObservabilitySnapshot snapshot() const;

  private:
    /*!
     * @brief           Guards every data member below; held only for the
     *                  duration of one record call or snapshot.
     */
    mutable std::mutex           mutex;
    /*!
     * @brief           Running totals returned by snapshot().
     */
    RgbdObservabilitySnapshot    counters;
    /*!
     * @brief           True once a processed packet has set
     *                  lastProcessedSensorTimestampNanoseconds.
     */
    bool                         hasProcessedSensorTimestamp{false};
    /*!
     * @brief           Sensor timestamp of the previous processed packet,
     *                  nanoseconds; compared with the next to measure the gap.
     */
    std::int64_t                 lastProcessedSensorTimestampNanoseconds{0};
    /*!
     * @brief           True once lastProcessedWorkerEnd holds a value.
     *                  Production has one worker, so processed worker-end times
     *                  are ordered.
     */
    bool                         hasProcessedWorkerEnd{false};
    /*!
     * @brief           Worker-end time of the latest in-order processed packet.
     */
    SteadyTime                   lastProcessedWorkerEnd{};
    /*!
     * @brief           True once the first one-second completion window has
     *                  been opened.
     */
    bool                         hasCompletionWindow{false};
    /*!
     * @brief           Steady-clock start of the open one-second completion
     *                  window; the first window starts at the first processed
     *                  packet's worker end.
     */
    SteadyTime                   completionWindowStart{};
    /*!
     * @brief           Packets finished inside the open completion window.
     */
    std::uint64_t                completionWindowCount{0U};
    /*!
     * @brief           Bins of the window histogram: counts 0 to 1000 each have
     *                  a bin and the last bin takes everything larger.
     */
    static constexpr std::size_t completionHistogramSize{1002U};
    /*!
     * @brief           Number of completed one-second windows seen for each
     *                  packet count; used to find the lower median.
     */
    std::array<std::uint64_t, completionHistogramSize>
                  completionWindowHistogram{};
    /*!
     * @brief           True once the first one-second arrival window has been
     *                  opened.
     */
    bool          hasCallbackArrivalWindow{false};
    /*!
     * @brief           Steady-clock start of the open one-second arrival
     *                  window.
     */
    SteadyTime    callbackArrivalWindowStart{};
    /*!
     * @brief           Arrival time of the previous GrabRGBD callback; used for
     *                  the arrival gap.
     */
    SteadyTime    lastCallbackArrival{};
    /*!
     * @brief           Arrivals inside the open arrival window.
     */
    std::uint64_t callbackArrivalWindowCount{0U};
};

/*!
 * @brief           Formats one stable, single-line, key-value summary.
 */
[[nodiscard]] std::string
    formatRgbdObservabilitySummary(const RgbdObservabilitySnapshot &snapshot_in,
                                   const std::string               &event_in);

} /* namespace vs_graphs::observability */

#endif /* VS_GRAPHS_RGBD_OBSERVABILITY_H */
