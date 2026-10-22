/*!
 * @file            ResetCauseRetentionClass.h
 *
 * @brief           Retains the truthful cause of one deferred reset execution.
 */

#ifndef VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_CAUSE_RETENTION_CLASS_H
#define VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_CAUSE_RETENTION_CLASS_H

#include "execution/reset_cause/objects/ResetCauseEnum.h"
#include "execution/reset_cause/objects/ResetCauseRetentionStatusEnum.h"

namespace vs_graphs::core::reset_cause
{
/*!
 * @brief           Retains one deferred reset cause; callers serialize access.
 */
class ResetCauseRetention
{
  public:
    /*!
     * @brief           Creates an empty retention record.
     */
    ResetCauseRetention() = default;

    /*!
     * @brief           Retains the first cause; differing later causes coalesce.
     *
     * @param[in]       cause_in
     *                  Cause of the deferred reset.
     *
     * @return          RESET_CAUSE_RETENTION_STATUS_SUCCESS.
     */
    [[nodiscard]] ResetCauseRetentionStatus retain(ResetCause cause_in) noexcept;

    /*!
     * @brief           Returns the retained cause and clears the record.
     *
     * @param[out]      resetCause_out
     *                  Retained cause, or UNATTRIBUTED_PUBLIC_REQUEST if empty.
     *
     * @return          RESET_CAUSE_RETENTION_STATUS_SUCCESS.
     */
    [[nodiscard]] ResetCauseRetentionStatus
        consume(ResetCause &resetCause_out) noexcept;

    /*!
     * @brief           Checks whether a cause is currently retained.
     *
     * @param[out]      hasRetainedCause_out
     *                  True when the record holds a retained cause.
     *
     * @return          RESET_CAUSE_RETENTION_STATUS_SUCCESS.
     */
    [[nodiscard]] ResetCauseRetentionStatus
        hasRetainedCause(bool &hasRetainedCause_out) const noexcept;

  private:
    /*! @brief           Whether a cause is currently retained. */
    bool hasCause{false};

    /*! @brief           Retained cause; meaningful only when hasCause is set. */
    ResetCause cause{ResetCause::UNATTRIBUTED_PUBLIC_REQUEST};
};
} /* namespace vs_graphs::core::reset_cause */

#endif /* VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_CAUSE_RETENTION_CLASS_H */
