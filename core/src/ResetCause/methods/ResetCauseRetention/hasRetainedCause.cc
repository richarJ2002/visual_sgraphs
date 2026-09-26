/*!
 * @file         hasRetainedCause.cc
 *
 * @brief        Implements ResetCauseRetention::hasRetainedCause declared in
 *               ResetCause.h.
 */

#include "ResetCause.h"

namespace vs_graphs
{
namespace core
{

bool ResetCauseRetention::hasRetainedCause() const noexcept
{
    return hasCause;
}

} // namespace core
} // namespace vs_graphs
