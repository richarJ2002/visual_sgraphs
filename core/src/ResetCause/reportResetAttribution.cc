/*!
 * @file         reportResetAttribution.cc
 *
 * @brief        Implements reportResetAttribution declared in ResetCause.h.
 */

#include "ResetCause.h"

#include <iostream>

namespace vs_graphs
{
namespace core
{

ResetCauseStatus reportResetAttribution(const ResetCause  cause_in,
                                        const ResetAction action_in)
{
    std::string resetAttribution{};
    if (formatResetAttribution(cause_in, action_in, resetAttribution) !=
        ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
    {
        // formatResetAttribution cannot fail; continue as before.
    }
    std::cout << resetAttribution << std::endl;

    return ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
