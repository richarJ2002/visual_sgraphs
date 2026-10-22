/*!
 * @file            consume.cc
 *
 * @brief           Implements ResetCauseRetention::consume declared in
 *                  ResetCause.h.
 */

#include "execution/reset_cause/objects/ResetCauseRetentionClass.h"

namespace vs_graphs
{
namespace core
{
namespace reset_cause
{

ResetCauseRetentionStatus
    ResetCauseRetention::consume(ResetCause &resetCause_out) noexcept
{
    const ResetCause retainedCause =
        hasCause ? cause : ResetCause::UNATTRIBUTED_PUBLIC_REQUEST;
    hasCause       = false;
    cause          = ResetCause::UNATTRIBUTED_PUBLIC_REQUEST;
    resetCause_out = retainedCause;
    return ResetCauseRetentionStatus::RESET_CAUSE_RETENTION_STATUS_SUCCESS;
}

} /* namespace reset_cause */
} /* namespace core */
} /* namespace vs_graphs */
