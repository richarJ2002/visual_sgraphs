

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

const char *SemanticVerify::mergeReasonName(const SemanticMergeReason reason_in)
{
    switch (reason_in)
    {
    case SemanticMergeReason::ALIGNED:
        return "ALIGNED";
    case SemanticMergeReason::INVALID_INPUT:
        return "INVALID_INPUT";
    case SemanticMergeReason::FLOOR_EVIDENCE_MISSING:
        return "FLOOR_EVIDENCE_MISSING";
    case SemanticMergeReason::FLOOR_CONTRADICTION:
        return "FLOOR_CONTRADICTION";
    case SemanticMergeReason::SHARED_ROOM_IDENTITY_MISSING:
        return "SHARED_ROOM_IDENTITY_MISSING";
    case SemanticMergeReason::WALL_EVIDENCE_MISSING:
        return "WALL_EVIDENCE_MISSING";
    case SemanticMergeReason::WALL_ALIGNMENT_CONTRADICTION:
        return "WALL_ALIGNMENT_CONTRADICTION";
    case SemanticMergeReason::PASSAGE_EVIDENCE_MISSING:
        return "PASSAGE_EVIDENCE_MISSING";
    case SemanticMergeReason::PASSAGE_IDENTITY_CONTRADICTION:
        return "PASSAGE_IDENTITY_CONTRADICTION";
    case SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION:
        return "PASSAGE_ENDPOINT_CONTRADICTION";
    case SemanticMergeReason::PASSAGE_DIRECTION_CONTRADICTION:
        return "PASSAGE_DIRECTION_CONTRADICTION";
    }
    return "UNKNOWN";
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
