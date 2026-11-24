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

/* Data include */
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
    RgbdObservability()                                     = default;
    ~RgbdObservability()                                    = default;
    RgbdObservability(const RgbdObservability &)            = delete;
    RgbdObservability &operator=(const RgbdObservability &) = delete;

    /*!
     * @brief           Records an admitted triple stored as the pending packet.
     *                  Called from GrabRGBD.
     *
     * @param[in]       didOverwritePending_in
     *                  True when the store replaced a pending packet the worker
     *                  had not taken.
     */
    void recordPendingStore(bool didOverwritePending_in);

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
     *                  worker in flight. Called from the worker thread.
     */
    void recordWorkerStart();

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
     * @brief           Records a pending packet discarded by the worker at
     *                  shutdown. Called from the worker thread.
     */
    void recordShutdownPendingDrop();

    /*!
     * @brief           Records a fully processed packet: the worker is no
     *                  longer busy and its sensor timestamp becomes the latest.
     *                  Called from the worker thread.
     *
     * @param[in]       sensorTimestampNanoseconds_in
     *                  Sensor (RGB header) timestamp of the packet, nanoseconds
     *                  on the ROS clock.
     */
    void recordProcessed(std::int64_t sensorTimestampNanoseconds_in);

    /*!
     * @brief           Returns a copy of the cumulative counters, taken under
     *                  the lock. Safe to call from any thread.
     *
     * @return          Copy of the counters at the time of the call.
     */
    [[nodiscard]] RgbdObservabilitySnapshot snapshot() const;

  private:
    /*!
     * @brief           Guards counters; held only for the duration of one
     *                  record call or snapshot.
     */
    mutable std::mutex mutex;

    /*!
     * @brief           Running totals returned by snapshot().
     */
    RgbdObservabilitySnapshot counters;
};
} /* namespace vs_graphs::observability */

#endif /* HPP_OBSERVABILITYCLASS_HPP */
