/*!
 * @file         consume.cc
 *
 * @brief        Implements ResetCauseRetention::consume declared in ResetCause.h.
 */

#include "ResetCause.h"

namespace vs_graphs
{
namespace core
{

ResetCause ResetCauseRetention::consume() noexcept
{
    const ResetCause retainedCause =
        hasCause ? cause : ResetCause::UNATTRIBUTED_PUBLIC_REQUEST;
    hasCause = false;
    cause    = ResetCause::UNATTRIBUTED_PUBLIC_REQUEST;
    return retainedCause;
}

} // namespace core
} // namespace vs_graphs
