/*!
 * @file            transformAbsorbedPoint.cc
 *
 * @brief           Implements transformAbsorbedPoint(), declared in
 *                  Semantic/SemanticVerify/private_functions.h.
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

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus transformAbsorbedPoint(const g2o::Sim3       &transform_in,
                                            const Eigen::Vector3d &point_in,
                                            Eigen::Vector3d       &mapped_out)
{
    const double          scale = transform_in.scale();
    const Eigen::Matrix3d rotation =
        transform_in.rotation().toRotationMatrix().cast<double>();
    const Eigen::Vector3d translation =
        transform_in.translation().cast<double>();
    if (!std::isfinite(scale) || scale <= 0.0 || !rotation.allFinite() ||
        !translation.allFinite() || !point_in.allFinite())
    {
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_INVALID_ARGUMENT;
    }
    mapped_out = scale * (rotation * point_in) + translation;
    if (!mapped_out.allFinite())
    {
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_NUMERICAL_FAILURE;
    }
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
