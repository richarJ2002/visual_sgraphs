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

void reportResetAttribution(const ResetCause cause_in, const ResetAction action_in)
{
    std::cout << formatResetAttribution(cause_in, action_in) << std::endl;
}

} // namespace core
} // namespace vs_graphs
