/*!
 * @file         resetCauseToString.cc
 *
 * @brief        Implements resetCauseToString declared in ResetCause.h.
 */

#include "ResetCause.h"

namespace vs_graphs
{
namespace core
{

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

} // namespace core
} // namespace vs_graphs
