/*!
 * @file            mapMergeConfigFromSystemParams.cc
 *
 * @brief           Implements SemanticVerify::mapMergeConfigFromSystemParams(),
 *                  declared in semantic/SemanticVerify.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"
#include "configuration/types/objects/SystemParamsStatusEnum.h"

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

SemanticVerifyStatus SemanticVerify::mapMergeConfigFromSystemParams(
    SemanticVerify::MapMergeConfig &configuration_out)
{
    MapMergeConfig       configuration;
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    configuration.passage_match_tolerance_m =
        static_cast<double>(p_params->mapMerge.passageMatchTolerance_m);
    types::SystemParams *p_params2 = nullptr;
    if (types::SystemParams::getParams(p_params2) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    configuration.wall_coplanar_angle_deg =
        static_cast<double>(p_params2->mapMerge.wallCoplanarAngle_deg);
    types::SystemParams *p_params3 = nullptr;
    if (types::SystemParams::getParams(p_params3) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    configuration.wall_edge_overlap_m =
        static_cast<double>(p_params3->mapMerge.wallEdgeOverlap_m);
    types::SystemParams *p_params4 = nullptr;
    if (types::SystemParams::getParams(p_params4) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    configuration.floor_match_tolerance_m =
        static_cast<double>(p_params4->mapMerge.floorMatchTolerance_m);
    types::SystemParams *p_params5 = nullptr;
    if (types::SystemParams::getParams(p_params5) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    configuration.room_centroid_tolerance_m =
        static_cast<double>(p_params5->mapMerge.roomCentroidTolerance_m);
    configuration_out = configuration;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
