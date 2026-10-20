/*!
 * @file            runFloorGate.cc
 *
 * @brief           Implements SemanticVerify::runFloorGate(), declared in
 *                  Semantic/SemanticVerify.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"
#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingStatusEnum.h"

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
    map::Map               *p_survivingMap_in,
    map::Map               *p_absorbedMap_in,
    const Eigen::Isometry3d &mergeTransform_absorbedToSurviving_in,
    bool                    &hasPassed_out)
{
    const g2o::Sim3 transform(
        mergeTransform_absorbedToSurviving_in.linear(),
        mergeTransform_absorbedToSurviving_in.translation(),
        1.0);
    std::string resultText;
    bool        passed{};
    if (verifyLoopMergeFloors(p_survivingMap_in,
                              p_absorbedMap_in,
                              transform,
                              resultText,
                              passed) !=
        loop_closing::LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
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

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
