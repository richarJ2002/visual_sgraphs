

#include "Semantic/SemanticVerify.h"

#include "Geometric/Plane.h"
#include "LoopClosing.h"
#include "Map.h"
#include "OptimizableTypes.h"
#include "Semantic/Room.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"
#include "Types/objects/SystemParams.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

const char *
    SemanticVerify::mergeDecisionName(const SemanticMergeDecision decision_in)
{
    switch (decision_in)
    {
    case SemanticMergeDecision::ACCEPT:
        return "ACCEPT";
    case SemanticMergeDecision::DEFER:
        return "DEFER";
    case SemanticMergeDecision::REJECT:
        return "REJECT";
    }
    return "UNKNOWN";
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
