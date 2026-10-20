/*!
 * @file            reset_cause.h
 *
 * @brief           Public interface of the reset_cause module.
 */

#ifndef VS_GRAPHS_CORE_SRC_RESET_CAUSE_RESET_CAUSE_H
#define VS_GRAPHS_CORE_SRC_RESET_CAUSE_RESET_CAUSE_H

#include "execution/reset_cause/objects/ResetActionEnum.h"
#include "execution/reset_cause/objects/ResetCauseEnum.h"
#include "execution/reset_cause/objects/ResetCauseRetentionClass.h"
#include "execution/reset_cause/objects/ResetCauseRetentionStatusEnum.h"
#include "execution/reset_cause/objects/ResetCauseStatusEnum.h"

#include <string>

namespace vs_graphs
{
namespace core
{
namespace reset_cause
{

/*!
 * @brief           Retains a deferred reset cause without changing the owner's
 *                  object layout. Thread-safe through an internal mutex.
 *
 * @param[in]       p_owner_in
 *                  Owning object the cause is attributed to; may be null.
 *
 * @param[in]       cause_in
 *                  Cause to retain.
 *
 * @return          RESET_CAUSE_STATUS_SUCCESS.
 */
[[nodiscard]] ResetCauseStatus retainResetCause(const void *p_owner_in,
                                                ResetCause  cause_in);
/*!
 * @brief           Returns and clears the deferred reset cause for one owner.
 *                  Thread-safe through an internal mutex.
 *
 * @param[in]       p_owner_in
 *                  Owning object the cause was attributed to; may be null.
 *
 * @param[out]      resetCause_out
 *                  Retained cause, or UNATTRIBUTED_PUBLIC_REQUEST when the
 *                  owner has no retained cause.
 *
 * @return          RESET_CAUSE_STATUS_SUCCESS.
 */
[[nodiscard]] ResetCauseStatus consumeResetCause(const void *p_owner_in,
                                                 ResetCause &resetCause_out);
/*!
 * @brief           Clears any deferred reset cause when its owner is destroyed.
 *                  Thread-safe through an internal mutex.
 *
 * @param[in]       p_owner_in
 *                  Owning object whose cause is discarded; may be null.
 *
 * @return          RESET_CAUSE_STATUS_SUCCESS.
 */
[[nodiscard]] ResetCauseStatus clearResetCause(const void *p_owner_in) noexcept;
/*!
 * @brief           Maps a reset cause to its stable log name.
 *
 * @param[in]       cause_in
 *                  Cause to name.
 *
 * @param[out]      p_text_out
 *                  Pointer to a static snake-case name, or "unknown" for an
 *                  unmapped value.
 *
 * @return          RESET_CAUSE_STATUS_SUCCESS.
 */
[[nodiscard]] ResetCauseStatus
    resetCauseToString(ResetCause cause_in, const char *&p_text_out) noexcept;
/*!
 * @brief           Maps a reset action to its stable log name.
 *
 * @param[in]       action_in
 *                  Action to name.
 *
 * @param[out]      p_text_out
 *                  Pointer to a static snake-case name, or "unknown" for an
 *                  unmapped value.
 *
 * @return          RESET_CAUSE_STATUS_SUCCESS.
 */
[[nodiscard]] ResetCauseStatus
    resetActionToString(ResetAction  action_in,
                        const char *&p_text_out) noexcept;
/*!
 * @brief           Formats one log line attributing a reset execution.
 *
 * @param[in]       cause_in
 *                  Attributed cause.
 *
 * @param[in]       action_in
 *                  Attributed action.
 *
 * @param[out]      resetAttribution_out
 *                  Text of the form "VSG_RESET_ATTRIBUTION cause=<cause>
 *                  action=<action>".
 *
 * @return          RESET_CAUSE_STATUS_SUCCESS.
 */
[[nodiscard]] ResetCauseStatus
                               formatResetAttribution(ResetCause   cause_in,
                                                      ResetAction  action_in,
                                                      std::string &resetAttribution_out);
/*!
 * @brief           Writes one reset-attribution line to standard output.
 *
 * @param[in]       cause_in
 *                  Attributed cause.
 *
 * @param[in]       action_in
 *                  Attributed action.
 *
 * @return          RESET_CAUSE_STATUS_SUCCESS.
 */
[[nodiscard]] ResetCauseStatus reportResetAttribution(ResetCause  cause_in,
                                                      ResetAction action_in);

} /* namespace reset_cause */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_RESET_CAUSE_RESET_CAUSE_H */
