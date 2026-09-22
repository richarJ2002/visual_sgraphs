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

std::string formatResetAttribution(const ResetCause  cause_in,
                                   const ResetAction action_in)
{
    return std::string("VSG_RESET_ATTRIBUTION cause=") +
           resetCauseToString(cause_in) +
           " action=" + resetActionToString(action_in);
}

} // namespace core
} // namespace vs_graphs
