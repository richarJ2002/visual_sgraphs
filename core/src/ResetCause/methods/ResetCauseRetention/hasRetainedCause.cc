/*!
 * @file         hasRetainedCause.cc
 *
 * @brief        Implements ResetCauseRetention::hasRetainedCause declared in
 *               ResetCause.h.
 */

#include "ResetCause.h"

namespace vs_graphs
{
namespace core
{

ResetCauseRetentionStatus ResetCauseRetention::hasRetainedCause(
    bool &hasRetainedCause_out) const noexcept
{
    hasRetainedCause_out = hasCause;
    return ResetCauseRetentionStatus::RESET_CAUSE_RETENTION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
