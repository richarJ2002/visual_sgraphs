/*!
 * @file            resetCauseToString.cc
 *
 * @brief           Implements resetCauseToString declared in ResetCause.h.
 */

#include "ResetCause.h"

namespace vs_graphs
{
namespace core
{

ResetCauseStatus resetCauseToString(const ResetCause cause_in,
                                    const char     *&p_text_out) noexcept
{
    switch (cause_in)
    {
    case ResetCause::UNATTRIBUTED_PUBLIC_REQUEST:
    {
        p_text_out = "unattributed_public_request";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::MULTIPLE_COALESCED_REQUESTS:
    {
        p_text_out = "multiple_coalesced_requests";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::LOCAL_MAPPER_BAD_IMU:
    {
        p_text_out = "local_mapper_bad_imu";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::NON_MONOTONIC_SENSOR_TIMESTAMP:
    {
        p_text_out = "non_monotonic_sensor_timestamp";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::TIMESTAMP_JUMP_BEFORE_IMU_INITIALIZATION:
    {
        p_text_out = "timestamp_jump_before_imu_initialization";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::TIMESTAMP_JUMP_BEFORE_SECOND_IMU_BA:
    {
        p_text_out = "timestamp_jump_before_second_imu_ba";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::TIMESTAMP_JUMP_AFTER_SECOND_IMU_BA:
    {
        p_text_out = "timestamp_jump_after_second_imu_ba";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP:
    {
        p_text_out = "visual_tracking_lost_small_map";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::VISUAL_TRACKING_LOST_BEFORE_IMU_INITIALIZATION:
    {
        p_text_out = "visual_tracking_lost_before_imu_initialization";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::VISUAL_TRACKING_LOST_NEW_MAP:
    {
        p_text_out = "visual_tracking_lost_new_map";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::INITIALIZATION_INSUFFICIENT_POINTS:
    {
        p_text_out = "initialization_insufficient_points";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::INITIALIZATION_INVALID_MONOCULAR_MAP:
    {
        p_text_out = "initialization_invalid_monocular_map";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::IMU_DELIVERY_GAP:
    {
        p_text_out = "imu_delivery_gap";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::SENSOR_PROCESSING_OVERLOAD:
    {
        p_text_out = "sensor_processing_overload";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::VIEWER_REQUEST:
    {
        p_text_out = "viewer_request";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::DATASET_CHANGE_SMALL_MAP:
    {
        p_text_out = "dataset_change_small_map";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetCause::DATASET_CHANGE_NEW_MAP:
    {
        p_text_out = "dataset_change_new_map";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    }
    p_text_out = "unknown";
    return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
