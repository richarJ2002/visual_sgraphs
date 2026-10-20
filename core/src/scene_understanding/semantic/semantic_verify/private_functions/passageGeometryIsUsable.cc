/*!
 * @file            passageGeometryIsUsable.cc
 *
 * @brief           Implements passageGeometryIsUsable(), declared in
 *                  Semantic/SemanticVerify/private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"
#include "scene_understanding/semantic/room/room.h"

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

SemanticVerifyStatus passageGeometryIsUsable(const PassageContext &context_in,
                                             bool                 &isUsable_out)
{
    isUsable_out =
        context_in.passageCentroid_world.allFinite() &&
        std::isfinite(context_in.width_m) && context_in.width_m > 0.0 &&
        std::isfinite(context_in.height_m) && context_in.height_m > 0.0;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
