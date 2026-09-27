/* Matching Declaration Include */

#include "Semantic/FaultInjection.h"

namespace vs_graphs
{
namespace testing
{

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

} // namespace testing
} // namespace vs_graphs
