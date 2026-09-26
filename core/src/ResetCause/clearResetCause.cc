/*!
 * @file         clearResetCause.cc
 *
 * @brief        Implements clearResetCause declared in ResetCause.h.
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

void clearResetCause(const void *const p_owner_in) noexcept
{
    const std::lock_guard<std::mutex> lock(resetCauseMutex);
    resetCausesByOwner.erase(p_owner_in);
}

} // namespace core
} // namespace vs_graphs
