/*!
 * @file         formatResetAttribution.cc
 *
 * @brief        Implements formatResetAttribution declared in ResetCause.h.
 */

#include "ResetCause.h"

namespace vs_graphs
{
namespace core
{

ResetCauseStatus formatResetAttribution(const ResetCause  cause_in,
                                        const ResetAction action_in,
                                        std::string      &resetAttribution_out)
{
    const char *p_text = nullptr;
    if (resetCauseToString(cause_in, p_text) !=
        ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
    {
        // resetCauseToString cannot fail; continue as before.
    }
    const char *p_text2 = nullptr;
    if (resetActionToString(action_in, p_text2) !=
        ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
    {
        // resetActionToString cannot fail; continue as before.
    }
    resetAttribution_out = std::string("VSG_RESET_ATTRIBUTION cause=") +
                           p_text + " action=" + p_text2;
    return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
