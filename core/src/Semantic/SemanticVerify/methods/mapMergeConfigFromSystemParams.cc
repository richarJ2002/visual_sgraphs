

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
    MapMergeConfig configuration;
    configuration.passage_match_tolerance_m = static_cast<double>(
        types::SystemParams::getParams()->mapMerge.passageMatchTolerance_m);
    configuration.wall_coplanar_angle_deg = static_cast<double>(
        types::SystemParams::getParams()->mapMerge.wallCoplanarAngle_deg);
    configuration.wall_edge_overlap_m = static_cast<double>(
        types::SystemParams::getParams()->mapMerge.wallEdgeOverlap_m);
    configuration.floor_match_tolerance_m = static_cast<double>(
        types::SystemParams::getParams()->mapMerge.floorMatchTolerance_m);
    configuration.room_centroid_tolerance_m = static_cast<double>(
        types::SystemParams::getParams()->mapMerge.roomCentroidTolerance_m);
    configuration_out = configuration;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
