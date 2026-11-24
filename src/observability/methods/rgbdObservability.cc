/*!
 * @file            observability/methods/rgbdObservability.cc
 *
 * @brief           Implements package-private RGB-D callback/worker accounting.
 */

#include "observability/observability.h"

#include <cstdint>
#include <mutex>

namespace vs_graphs::observability
{

void RgbdObservability::recordPendingStore(const bool didOverwritePending_in)
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.pendingStores;
    if (didOverwritePending_in)
    {
        ++counters.pendingOverwrites;
    }
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

void RgbdObservability::recordWorkerStart()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.workersInFlight;
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

void RgbdObservability::recordShutdownPendingDrop()
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.shutdownPendingDrops;
}

void RgbdObservability::recordProcessed(
    const std::int64_t sensorTimestampNanoseconds_in)
{
    const std::lock_guard<std::mutex> lock(mutex);
    ++counters.processedPackets;
    if (counters.workersInFlight > 0U)
    {
        --counters.workersInFlight;
    }
    counters.lastProcessedSensorTimestampNanoseconds =
        sensorTimestampNanoseconds_in;
}

RgbdObservabilitySnapshot RgbdObservability::snapshot() const
{
    const std::lock_guard<std::mutex> lock(mutex);
    return counters;
}

} /* namespace vs_graphs::observability */
