/*!
 * @file            CheckFault.cc
 *
 * @brief           Implements checkFault(), declared in
 *                  Semantic/FaultInjection.h.
 */

/* Matching Declaration Include */

#include "Semantic/FaultInjection.h"

namespace vs_graphs
{
namespace testing
{

bool checkFault(const std::string &name_in)
{
#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
    const std::unordered_map<std::string, std::function<bool()>>::const_iterator
        foundIt = getFaultRegistry().find(name_in);
    if (foundIt == getFaultRegistry().end())
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
