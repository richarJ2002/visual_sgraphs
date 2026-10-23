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
 * Cumulative RGB-D frontend counters at the GrabRGBD callback boundary. They
 * feed the rgbd_frontend_* fields of the get_mission_health service.
 */
struct RgbdObservabilitySnapshot
{
    /*!
     * @brief           Packets the worker thread has started but not yet
     *                  finished, rejected or failed.
     */
    std::uint64_t workersInFlight{0U};

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
     * @brief           Pending packets discarded by the worker when shutdown
     *                  was requested.
     */
    std::uint64_t shutdownPendingDrops{0U};

    /*!
     * @brief           Sensor (RGB header) timestamp of the latest processed
     *                  packet, nanoseconds on the ROS clock; 0 before the
     *                  first.
     */
    std::int64_t lastProcessedSensorTimestampNanoseconds{0};
};
} /* namespace vs_graphs::observability */

#endif /* HPP_OBSERVABILITYSNAPSHOT_HPP */
