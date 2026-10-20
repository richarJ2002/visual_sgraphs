/*!
 * @file            wallSamplesSpanInterval.cc
 *
 * @brief           Implements wallSamplesSpanInterval(), declared in
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

SemanticVerifyStatus
    wallSamplesSpanInterval(const VerifyWallObservation &wall_in,
                            const Eigen::Vector3d       &axis_in,
                            const Eigen::Vector3d       &origin_in,
                            double                      &minimum_out,
                            double                      &maximum_out,
                            bool                        &hasFiniteSample_out)
{
    bool   hasSample = false;
    double minimum   = std::numeric_limits<double>::infinity();
    double maximum   = -std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d &sample : wall_in.supportSample_world)
    {
        if (!sample.allFinite())
        {
            continue;
        }
        const double coordinate = (sample - origin_in).dot(axis_in);
        minimum                 = std::min(minimum, coordinate);
        maximum                 = std::max(maximum, coordinate);
        hasSample               = true;
    }
    if (!hasSample)
    {
        hasFiniteSample_out = false;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    minimum_out         = minimum;
    maximum_out         = maximum;
    hasFiniteSample_out = true;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
