/* Matching Declaration Include */

#include "Semantic/FaultInjection.h"

namespace vs_graphs
{
namespace testing
{

void ClearFaults()
{
#ifdef VS_GRAPHS_ENABLE_FAULT_INJECTION
    FaultRegistry().clear();
#endif
}

} // namespace testing
} // namespace vs_graphs
