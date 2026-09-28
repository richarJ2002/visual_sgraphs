

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

SemanticVerifyStatus SemanticVerify::mapMergeConfigFromSystemParams(
    SemanticVerify::MapMergeConfig &configuration_out)
{
    MapMergeConfig       configuration;
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    configuration.passage_match_tolerance_m =
        static_cast<double>(p_params->mapMerge.passageMatchTolerance_m);
    types::SystemParams *p_params2 = nullptr;
    if (types::SystemParams::getParams(p_params2) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    configuration.wall_coplanar_angle_deg =
        static_cast<double>(p_params2->mapMerge.wallCoplanarAngle_deg);
    types::SystemParams *p_params3 = nullptr;
    if (types::SystemParams::getParams(p_params3) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    configuration.wall_edge_overlap_m =
        static_cast<double>(p_params3->mapMerge.wallEdgeOverlap_m);
    types::SystemParams *p_params4 = nullptr;
    if (types::SystemParams::getParams(p_params4) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    configuration.floor_match_tolerance_m =
        static_cast<double>(p_params4->mapMerge.floorMatchTolerance_m);
    types::SystemParams *p_params5 = nullptr;
    if (types::SystemParams::getParams(p_params5) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    configuration.room_centroid_tolerance_m =
        static_cast<double>(p_params5->mapMerge.roomCentroidTolerance_m);
    configuration_out = configuration;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
