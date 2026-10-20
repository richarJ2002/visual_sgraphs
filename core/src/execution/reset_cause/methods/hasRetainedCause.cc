/*!
 * @file            hasRetainedCause.cc
 *
 * @brief           Implements ResetCauseRetention::hasRetainedCause declared in
 *                  ResetCause.h.
 */

#include "execution/reset_cause/objects/ResetCauseRetentionClass.h"

namespace vs_graphs
{
namespace core
{
namespace reset_cause
{

ResetCauseRetentionStatus ResetCauseRetention::hasRetainedCause(
    bool &hasRetainedCause_out) const noexcept
{
    hasRetainedCause_out = hasCause;
    return ResetCauseRetentionStatus::RESET_CAUSE_RETENTION_STATUS_SUCCESS;
}

} /* namespace reset_cause */
} /* namespace core */
} /* namespace vs_graphs */
