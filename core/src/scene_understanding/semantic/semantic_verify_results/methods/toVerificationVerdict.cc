/*!
 * @file            toVerificationVerdict.cc
 *
 * @brief           Implements SemanticVerifyResult::toVerificationVerdict(),
 *                  declared in semantic/SemanticVerify.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"
#include "scene_understanding/semantic/room_tracker/room_tracker.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyResultStatusEnum.h"

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

SemanticVerifyResultStatus SemanticVerifyResult::toVerificationVerdict(
    VerificationVerdict &verificationVerdict_out) const
{
    VerificationVerdict verdict;
    verdict.status      = status;
    verdict.hasPassed   = hasPassed && hasFloorGatePassed;
    verdict.inlierCount = static_cast<unsigned int>(inliers.size());
    verdict.inlierRatio = inlierRatio;
    verdict.normalisedConditionNumber = normalisedConditionNumber;
    verdict.angularResidual_rad       = angularResidual_rad;
    verdict.confidence                = confidence;
    verificationVerdict_out           = verdict;
    return SemanticVerifyResultStatus::SEMANTIC_VERIFY_RESULT_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
