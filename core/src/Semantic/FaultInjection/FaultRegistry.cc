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

} // namespace testing
} // namespace vs_graphs
