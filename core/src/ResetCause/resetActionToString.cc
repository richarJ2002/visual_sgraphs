/*!
 * @file            resetActionToString.cc
 *
 * @brief           Implements resetActionToString declared in ResetCause.h.
 */

#include "ResetCause.h"

namespace vs_graphs
{
namespace core
{

ResetCauseStatus resetActionToString(const ResetAction action_in,
                                     const char      *&p_text_out) noexcept
{
    switch (action_in)
    {
    case ResetAction::RESET_ACTIVE_MAP_REQUEST:
    {
        p_text_out = "reset_active_map_request";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetAction::RESET_ACTIVE_MAP_EXECUTION:
    {
        p_text_out = "reset_active_map_execution";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    case ResetAction::CREATE_MAP_EXECUTION:
    {
        p_text_out = "create_map_execution";
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }
    }
    p_text_out = "unknown";
    return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
