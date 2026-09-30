/*!
 * @file            FaultRegistry.cc
 *
 * @brief           Implements getFaultRegistry(), declared in
 *                  Semantic/FaultInjection.h.
 */

/* Matching Declaration Include */

#include "Semantic/FaultInjection.h"

namespace vs_graphs
{
namespace testing
{

std::unordered_map<std::string, FaultAction> &getFaultRegistry()
{
#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
    thread_local std::unordered_map<std::string, FaultAction> registry;
    return registry;
#else
    /* Disabled builds still satisfy the linker; the map stays empty and
    checkFault below never consults it. */
    static std::unordered_map<std::string, FaultAction> emptyRegistry;
    return emptyRegistry;
#endif
}

} // namespace testing
} // namespace vs_graphs
