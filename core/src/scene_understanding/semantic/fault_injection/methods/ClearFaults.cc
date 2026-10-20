/*!
 * @file            ClearFaults.cc
 *
 * @brief           Implements clearFaults(), declared in
 *                  Semantic/FaultInjection.h.
 */

/* Matching Declaration Include */

#include "scene_understanding/semantic/fault_injection/fault_injection.h"

namespace vs_graphs
{
namespace testing
{

void clearFaults()
{
#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
    getFaultRegistry().clear();
#endif
}

} /* namespace testing */
} /* namespace vs_graphs */
