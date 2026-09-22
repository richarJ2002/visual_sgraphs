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
extern std::mutex resetCauseMutex;
extern std::unordered_map<const void *, ResetCauseRetention> resetCausesByOwner;

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

} // namespace core
} // namespace vs_graphs
