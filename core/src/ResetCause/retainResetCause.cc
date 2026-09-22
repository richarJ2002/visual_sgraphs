/*!
 * @file         retainResetCause.cc
 *
 * @brief        Implements retainResetCause declared in ResetCause.h.
 */

#include "ResetCause.h"

#include <mutex>
#include <unordered_map>

namespace vs_graphs
{
namespace core
{

/* Registry home: the mutex and owner map formerly lived in an anonymous
 * namespace inside ResetCause.cc. They are defined once here with external
 * linkage so the split registry translation units share them; behavior is
 * unchanged (same mutex, same map, same lock discipline). */
std::mutex resetCauseMutex;
std::unordered_map<const void *, ResetCauseRetention> resetCausesByOwner;

void retainResetCause(const void *const p_owner_in, const ResetCause cause_in)
{
    const std::lock_guard<std::mutex> lock(resetCauseMutex);
    resetCausesByOwner[p_owner_in].retain(cause_in);
}

} // namespace core
} // namespace vs_graphs
