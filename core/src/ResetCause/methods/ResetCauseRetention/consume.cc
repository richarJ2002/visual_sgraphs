/*!
 * @file         consume.cc
 *
 * @brief        Implements ResetCauseRetention::consume declared in
 *               ResetCause.h.
 */

#include "ResetCause.h"

namespace vs_graphs
{
namespace core
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

} // namespace core
} // namespace vs_graphs
