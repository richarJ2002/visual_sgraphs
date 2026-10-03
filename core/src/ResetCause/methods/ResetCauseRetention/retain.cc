/*!
 * @file            retain.cc
 *
 * @brief           Implements ResetCauseRetention::retain declared in
 *                  ResetCause.h.
 */

#include "ResetCause.h"

namespace vs_graphs
{
namespace core
{

ResetCauseRetentionStatus
    ResetCauseRetention::retain(const ResetCause cause_in) noexcept
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

    return ResetCauseRetentionStatus::RESET_CAUSE_RETENTION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
