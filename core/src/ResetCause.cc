/**
 * @file ResetCause.cc
 * @brief Implements stable internal reset/new-map attribution values.
 */

#include "ResetCause.h"

#include <iostream>
#include <mutex>
#include <unordered_map>

namespace vs_graphs
{
namespace core
{
namespace
{
std::mutex                                            resetCauseMutex;
std::unordered_map<const void *, ResetCauseRetention> resetCausesByOwner;
} /* namespace */

const char *resetCauseToString(const ResetCause cause_in) noexcept
{
    switch (cause_in)
    {
    case ResetCause::UNATTRIBUTED_PUBLIC_REQUEST:
        return "unattributed_public_request";
    case ResetCause::MULTIPLE_COALESCED_REQUESTS:
        return "multiple_coalesced_requests";
    case ResetCause::LOCAL_MAPPER_BAD_IMU:
        return "local_mapper_bad_imu";
    case ResetCause::NON_MONOTONIC_SENSOR_TIMESTAMP:
        return "non_monotonic_sensor_timestamp";
    case ResetCause::TIMESTAMP_JUMP_BEFORE_IMU_INITIALIZATION:
        return "timestamp_jump_before_imu_initialization";
    case ResetCause::TIMESTAMP_JUMP_BEFORE_SECOND_IMU_BA:
        return "timestamp_jump_before_second_imu_ba";
    case ResetCause::TIMESTAMP_JUMP_AFTER_SECOND_IMU_BA:
        return "timestamp_jump_after_second_imu_ba";
    case ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP:
        return "visual_tracking_lost_small_map";
    case ResetCause::VISUAL_TRACKING_LOST_BEFORE_IMU_INITIALIZATION:
        return "visual_tracking_lost_before_imu_initialization";
    case ResetCause::VISUAL_TRACKING_LOST_NEW_MAP:
        return "visual_tracking_lost_new_map";
    case ResetCause::INITIALIZATION_INSUFFICIENT_POINTS:
        return "initialization_insufficient_points";
    case ResetCause::INITIALIZATION_INVALID_MONOCULAR_MAP:
        return "initialization_invalid_monocular_map";
    case ResetCause::IMU_DELIVERY_GAP:
        return "imu_delivery_gap";
    case ResetCause::SENSOR_PROCESSING_OVERLOAD:
        return "sensor_processing_overload";
    case ResetCause::VIEWER_REQUEST:
        return "viewer_request";
    case ResetCause::DATASET_CHANGE_SMALL_MAP:
        return "dataset_change_small_map";
    case ResetCause::DATASET_CHANGE_NEW_MAP:
        return "dataset_change_new_map";
    }
    return "unknown";
}

void ResetCauseRetention::retain(const ResetCause cause_in) noexcept
{
    if (!hasCause)
    {
        cause    = cause_in;
        hasCause = true;
    }
    else if (cause != cause_in)
    {
        cause = ResetCause::MULTIPLE_COALESCED_REQUESTS;
    }
}

ResetCause ResetCauseRetention::consume() noexcept
{
    const ResetCause retainedCause =
        hasCause ? cause : ResetCause::UNATTRIBUTED_PUBLIC_REQUEST;
    hasCause = false;
    cause    = ResetCause::UNATTRIBUTED_PUBLIC_REQUEST;
    return retainedCause;
}

bool ResetCauseRetention::hasRetainedCause() const noexcept
{
    return hasCause;
}

void retainResetCause(const void *const p_owner_in, const ResetCause cause_in)
{
    const std::lock_guard<std::mutex> lock(resetCauseMutex);
    resetCausesByOwner[p_owner_in].retain(cause_in);
}

ResetCause consumeResetCause(const void *const p_owner_in)
{
    const std::lock_guard<std::mutex> lock(resetCauseMutex);
    const std::unordered_map<const void *, ResetCauseRetention>::iterator
        entry = resetCausesByOwner.find(p_owner_in);
    if (entry == resetCausesByOwner.end())
    {
        return ResetCause::UNATTRIBUTED_PUBLIC_REQUEST;
    }

    const ResetCause cause = entry->second.consume();
    resetCausesByOwner.erase(entry);
    return cause;
}

void clearResetCause(const void *const p_owner_in) noexcept
{
    const std::lock_guard<std::mutex> lock(resetCauseMutex);
    resetCausesByOwner.erase(p_owner_in);
}

const char *resetActionToString(const ResetAction action_in) noexcept
{
    switch (action_in)
    {
    case ResetAction::RESET_ACTIVE_MAP_REQUEST:
        return "reset_active_map_request";
    case ResetAction::RESET_ACTIVE_MAP_EXECUTION:
        return "reset_active_map_execution";
    case ResetAction::CREATE_MAP_EXECUTION:
        return "create_map_execution";
    }
    return "unknown";
}

std::string formatResetAttribution(const ResetCause  cause_in,
                                   const ResetAction action_in)
{
    return std::string("VSG_RESET_ATTRIBUTION cause=") +
           resetCauseToString(cause_in) +
           " action=" + resetActionToString(action_in);
}

void reportResetAttribution(const ResetCause  cause_in,
                            const ResetAction action_in)
{
    std::cout << formatResetAttribution(cause_in, action_in) << std::endl;
}

} // namespace core
} // namespace vs_graphs
