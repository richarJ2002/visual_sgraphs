/*!
 * @file         retainResetCause.cc
 *
 * @brief        Implements retainResetCause declared in ResetCause.h.
 */

#include "ResetCause.h"

#include <mutex>
#include <rclcpp/logging.hpp>
#include <unordered_map>

namespace vs_graphs
{
namespace core
{

/* Registry home: the mutex and owner map formerly lived in an anonymous
 * namespace inside ResetCause.cc. They are defined once here with external
 * linkage so the split registry translation units share them; behavior is
 * unchanged (same mutex, same map, same lock discipline). */
/*!
 * @brief        Mutex that guards resetCausesByOwner; held by retain, consume
 *               and clear for the whole access.
 */
std::mutex resetCauseMutex;

/*!
 * @brief        Deferred reset cause of each owner, keyed by the owner's
 *               address (used only as an identity, never dereferenced).
 *               Guarded by resetCauseMutex.
 */
std::unordered_map<const void *, ResetCauseRetention> resetCausesByOwner;

ResetCauseStatus retainResetCause(const void *const p_owner_in,
                                  const ResetCause  cause_in)
{
    const std::lock_guard<std::mutex> lock(resetCauseMutex);
    if (resetCausesByOwner[p_owner_in].retain(cause_in) !=
        ResetCauseRetentionStatus::RESET_CAUSE_RETENTION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: retain returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
