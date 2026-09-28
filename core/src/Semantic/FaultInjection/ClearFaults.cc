/* Matching Declaration Include */

#include "Semantic/FaultInjection.h"

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

} // namespace testing
} // namespace vs_graphs
