/**
 * @file ResetCause.h
 * @brief Declares stable internal reset/new-map attribution values.
 */

#ifndef VS_GRAPHS_CORE_RESET_CAUSE_H
#define VS_GRAPHS_CORE_RESET_CAUSE_H

#include <cstdint>
#include <string>

namespace vs_graphs
{
namespace core
{

/**
 * Internal causes observable at reset/new-map call sites.
 *
 * The public System::Reset() and System::ResetActiveMap() API has unknown
 * external callers and retains no cause while deferring work to the next
 * tracking call. Those execution points therefore remain explicitly unknown;
 * known package-owned callers report the exact request here instead of
 * changing System or Tracking object layout.
 */
enum class ResetCause : std::uint8_t
{
    UNATTRIBUTED_PUBLIC_REQUEST                    = 0U,
    MULTIPLE_COALESCED_REQUESTS                    = 1U,
    LOCAL_MAPPER_BAD_IMU                           = 2U,
    NON_MONOTONIC_SENSOR_TIMESTAMP                 = 3U,
    TIMESTAMP_JUMP_BEFORE_IMU_INITIALIZATION       = 4U,
    TIMESTAMP_JUMP_BEFORE_SECOND_IMU_BA            = 5U,
    TIMESTAMP_JUMP_AFTER_SECOND_IMU_BA             = 6U,
    VISUAL_TRACKING_LOST_SMALL_MAP                 = 7U,
    VISUAL_TRACKING_LOST_BEFORE_IMU_INITIALIZATION = 8U,
    VISUAL_TRACKING_LOST_NEW_MAP                   = 9U,
    INITIALIZATION_INSUFFICIENT_POINTS             = 10U,
    INITIALIZATION_INVALID_MONOCULAR_MAP           = 11U,
    IMU_DELIVERY_GAP                               = 12U,
    SENSOR_PROCESSING_OVERLOAD                     = 13U,
    VIEWER_REQUEST                                 = 14U,
    DATASET_CHANGE_SMALL_MAP                       = 15U,
    DATASET_CHANGE_NEW_MAP                         = 16U
};

/** Retains the truthful cause of one deferred active-map reset execution. */
class ResetCauseRetention
{
  public:
    ResetCauseRetention()  = default;
    ~ResetCauseRetention() = default;

    void                     retain(ResetCause cause_in) noexcept;
    [[nodiscard]] ResetCause consume() noexcept;
    [[nodiscard]] bool       hasRetainedCause() const noexcept;

  private:
    bool       hasCause{false};
    ResetCause cause{ResetCause::UNATTRIBUTED_PUBLIC_REQUEST};
};

/** Retains a deferred reset cause without changing the owner's object layout.
 */
void retainResetCause(const void *p_owner_in, ResetCause cause_in);

/** Returns and clears the deferred reset cause for one owner. */
[[nodiscard]] ResetCause consumeResetCause(const void *p_owner_in);

/** Clears any deferred reset cause when its owner is destroyed. */
void clearResetCause(const void *p_owner_in) noexcept;

/** Action requested or performed at an attributed call site. */
enum class ResetAction : std::uint8_t
{
    RESET_ACTIVE_MAP_REQUEST   = 0U,
    RESET_ACTIVE_MAP_EXECUTION = 1U,
    CREATE_MAP_EXECUTION       = 2U
};

[[nodiscard]] const char *resetCauseToString(ResetCause cause_in) noexcept;
[[nodiscard]] const char *resetActionToString(ResetAction action_in) noexcept;
[[nodiscard]] std::string formatResetAttribution(ResetCause  cause_in,
                                                 ResetAction action_in);
void reportResetAttribution(ResetCause cause_in, ResetAction action_in);

} // namespace core
} // namespace vs_graphs

#endif /* VS_GRAPHS_CORE_RESET_CAUSE_H */
