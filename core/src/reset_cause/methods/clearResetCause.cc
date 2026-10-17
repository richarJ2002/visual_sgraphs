/*!
 * @file            clearResetCause.cc
 *
 * @brief           Implements clearResetCause declared in
 * reset_cause/objects/ResetCauseRetentionClass.h.
 */

#include "reset_cause/reset_cause.h"

#include <mutex>
#include <unordered_map>

namespace vs_graphs
{
namespace core
{
namespace reset_cause
{

/* Shared with retainResetCause.cc (registry home). */
extern std::mutex                                            resetCauseMutex;
extern std::unordered_map<const void *, ResetCauseRetention> resetCausesByOwner;

ResetCauseStatus clearResetCause(const void *const p_owner_in) noexcept
{
    const std::lock_guard<std::mutex> lock(resetCauseMutex);
    resetCausesByOwner.erase(p_owner_in);

    return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
}

} /* namespace reset_cause */
} /* namespace core */
} /* namespace vs_graphs */
