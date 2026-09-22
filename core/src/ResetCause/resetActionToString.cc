/*!
 * @file         resetActionToString.cc
 *
 * @brief        Implements resetActionToString declared in ResetCause.h.
 */

#include "ResetCause.h"

namespace vs_graphs
{
namespace core
{

const char *resetActionToString(const ResetAction action_in) noexcept
{
    switch (action_in)
    {
    case ResetAction::RESET_ACTIVE_MAP_REQUEST:
        return "reset_active_map_request";
    case ResetAction::RESET_ACTIVE_MAP_EXECUTION:
        return "reset_active_map_execution";
    case ResetAction::CREATE_MAP_EXECUTION:
        return "create_map_execution";
    }
    return "unknown";
}

} // namespace core
} // namespace vs_graphs
