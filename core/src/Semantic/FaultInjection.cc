/*!
 * @file            FaultInjection.cc
 *
 * @brief           Implements the thread-local fault-injection registry.
 *
 * @date            15/09/2026
 */

/* Matching Declaration Include */
#include "Semantic/FaultInjection.h"

namespace vs_graphs
{
namespace testing
{

std::unordered_map<std::string, FaultAction> &FaultRegistry()
{
#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
    thread_local std::unordered_map<std::string, FaultAction> registry;
    return registry;
#else
    /* Disabled builds still satisfy the linker; the map stays empty and
    CheckFault below never consults it. */
    static std::unordered_map<std::string, FaultAction> emptyRegistry;
    return emptyRegistry;
#endif
}

void RegisterFault(const std::string &name_in, FaultAction action_in)
{
#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
    FaultRegistry()[name_in] = action_in;
#else
    (void)name_in;
    (void)action_in;
#endif
}

void ClearFaults()
{
#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
    FaultRegistry().clear();
#endif
}

bool CheckFault(const std::string &name_in)
{
#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
    const auto foundIt = FaultRegistry().find(name_in);
    if (foundIt == FaultRegistry().end())
    {
        return false;
    }
    return foundIt->second();
#else
    (void)name_in;
    return false;
#endif
}

} /* namespace testing */
} /* namespace vs_graphs */
