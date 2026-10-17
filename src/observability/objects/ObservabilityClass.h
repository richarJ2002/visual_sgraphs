/*!
 * @file            ObservabilityClass.h
 *
 * @brief           Declares RGB-D callback and worker accounting.
 *
 * @date            09/10/2026
 */

#ifndef HPP_OBSERVABILITYCLASS_HPP
#define HPP_OBSERVABILITYCLASS_HPP

/* Function Includes */
/* None */

/* Object Include */
#include "ObservabilitySnapshotStruct.h"
#include "PublishTopicEnum.h"
#include "PublishTopicsTimingSinkStruct.h"

/* Data include */
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>

/* Generic Libraries */
/* None */

namespace vs_graphs::observability
{

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
    void recordTrackCall();

    /*!
     * @brief           Records a tracker call that returned normally. Called
     *                  from the worker thread.
     */
    void recordTrackCompletion();

    /*!
     * @brief           Records a tracker call that ended with an exception; the
     *                  worker is no longer busy. Called from the worker thread.
     */
    void recordTrackFailure();

    /*!
     * @brief           Records a publishTopics call that ended with an
     *                  exception; the worker is no longer busy. Called from the
     *                  worker thread.
     */
    void recordPublishTopicsFailure();

    /*!
     * @brief           Records an admitted triple dropped because shutdown was
     *                  requested; it is no longer in flight. Called from
     *                  GrabRGBD.
     */
    void recordShutdownAdmissionReject();

    /*!
     * @brief           Records a pending packet discarded by the worker at
     *                  shutdown. Called from the worker thread.
     */
    void recordShutdownPendingDrop();

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
    mutable std::mutex mutex;

    /*!
     * @brief           Running totals returned by snapshot().
     */
    RgbdObservabilitySnapshot counters;

    /*!
     * @brief           True once a processed packet has set
     *                  lastProcessedSensorTimestampNanoseconds.
     */
    bool hasProcessedSensorTimestamp{false};

    /*!
     * @brief           Sensor timestamp of the previous processed packet,
     *                  nanoseconds; compared with the next to measure the gap.
     */
    std::int64_t lastProcessedSensorTimestampNanoseconds{0};

    /*!
     * @brief           True once lastProcessedWorkerEnd holds a value.
     *                  Production has one worker, so processed worker-end times
     *                  are ordered.
     */
    bool hasProcessedWorkerEnd{false};

    /*!
     * @brief           Worker-end time of the latest in-order processed packet.
     */
    SteadyTime lastProcessedWorkerEnd{};

    /*!
     * @brief           True once the first one-second completion window has
     *                  been opened.
     */
    bool hasCompletionWindow{false};

    /*!
     * @brief           Steady-clock start of the open one-second completion
     *                  window; the first window starts at the first processed
     *                  packet's worker end.
     */
    SteadyTime completionWindowStart{};

    /*!
     * @brief           Packets finished inside the open completion window.
     */
    std::uint64_t completionWindowCount{0U};

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
    bool hasCallbackArrivalWindow{false};

    /*!
     * @brief           Steady-clock start of the open one-second arrival
     *                  window.
     */
    SteadyTime callbackArrivalWindowStart{};

    /*!
     * @brief           Arrival time of the previous GrabRGBD callback; used for
     *                  the arrival gap.
     */
    SteadyTime lastCallbackArrival{};

    /*!
     * @brief           Arrivals inside the open arrival window.
     */
    std::uint64_t callbackArrivalWindowCount{0U};
};
} /* namespace vs_graphs::observability */

#endif /* HPP_OBSERVABILITYCLASS_HPP */
