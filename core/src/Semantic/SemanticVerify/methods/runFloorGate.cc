/*!
 * @file            runFloorGate.cc
 *
 * @brief           Implements SemanticVerify::runFloorGate(), declared in
 *                  Semantic/SemanticVerify.h.
 */

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
#include <rclcpp/logging.hpp>
#include <set>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus SemanticVerify::runFloorGate(
    SemanticVerifyResult    &result_inout,
    core::Map               *p_survivingMap_in,
    core::Map               *p_absorbedMap_in,
    const Eigen::Isometry3d &transform_absorbedToSurviving_in,
    bool                    &hasPassed_out)
{
    const g2o::Sim3 transform(transform_absorbedToSurviving_in.linear(),
                              transform_absorbedToSurviving_in.translation(),
                              1.0);
    std::string     resultText;
    bool            passed{};
    if (verifyLoopMergeFloors(p_survivingMap_in,
                              p_absorbedMap_in,
                              transform,
                              resultText,
                              passed) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: verifyLoopMergeFloors returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    result_inout.hasFloorGateRun    = true;
    result_inout.hasFloorGatePassed = passed;
    result_inout.floorGateResult    = resultText;
    hasPassed_out                   = passed;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
