/*!
 * @file         consumeResetCause.cc
 *
 * @brief        Implements consumeResetCause declared in ResetCause.h.
 */

#include "ResetCause.h"

#include <mutex>
#include <unordered_map>

namespace vs_graphs
{
namespace core
{

/* Shared with retainResetCause.cc (registry home). */
extern std::mutex                                            resetCauseMutex;
extern std::unordered_map<const void *, ResetCauseRetention> resetCausesByOwner;

ResetCauseStatus consumeResetCause(const void *const p_owner_in,
                                   ResetCause       &resetCause_out)
{
    const std::lock_guard<std::mutex> lock(resetCauseMutex);
    const std::unordered_map<const void *, ResetCauseRetention>::iterator
        entry = resetCausesByOwner.find(p_owner_in);
    if (entry == resetCausesByOwner.end())
    {
        resetCause_out = ResetCause::UNATTRIBUTED_PUBLIC_REQUEST;
        return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
    }

    ResetCause cause{};
    if (entry->second.consume(cause) !=
        ResetCauseRetentionStatus::RESET_CAUSE_RETENTION_STATUS_SUCCESS)
    {
        // consume cannot fail; continue as before.
    }
    resetCausesByOwner.erase(entry);
    resetCause_out = cause;
    return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
