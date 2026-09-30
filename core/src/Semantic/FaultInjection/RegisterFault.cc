/*!
 * @file            RegisterFault.cc
 *
 * @brief           Implements registerFault(), declared in
 *                  Semantic/FaultInjection.h.
 */

/* Matching Declaration Include */

#include "Semantic/FaultInjection.h"

namespace vs_graphs
{
namespace testing
{

void registerFault(const std::string &name_in, FaultAction action_in)
{
#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
    getFaultRegistry()[name_in] = action_in;
#else
    (void)name_in;
    (void)action_in;
#endif
}

} // namespace testing
} // namespace vs_graphs
