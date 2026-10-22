/*!
 * @file            mergeDecisionName.cc
 *
 * @brief           Implements SemanticVerify::mergeDecisionName().
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include "mapping/geometric/Plane/Plane.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"
#include "map_state/map/map.h"

#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>
#include "configuration/types/types.h"
#include "scene_understanding/semantic/room/objects/RoomClass.h"

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

SemanticVerifyStatus
    SemanticVerify::mergeDecisionName(const SemanticMergeDecision decision_in,
                                      const char                *&p_name_out)
{
    switch (decision_in)
    {
    case SemanticMergeDecision::ACCEPT:
    {
        p_name_out = "ACCEPT";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeDecision::DEFER:
    {
        p_name_out = "DEFER";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    case SemanticMergeDecision::REJECT:
    {
        p_name_out = "REJECT";
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    }
    p_name_out = "UNKNOWN";
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
