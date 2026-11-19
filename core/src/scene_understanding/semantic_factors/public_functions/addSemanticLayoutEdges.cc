/*!
 * @file            addSemanticLayoutEdges.cc
 *
 * @brief           Implements addSemanticLayoutEdges(), declared in
 *                  public_functions.h.
 */

#include "scene_understanding/semantic_factors/public_functions/public_functions.h"

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/logging.hpp>
#include <vs_graphs_g2o/core/sparse_optimizer.h>

#include "configuration/types/types.h"
#include "map_state/map/map.h"
#include "scene_understanding/semantic_factors/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

[[nodiscard]] optimizer::OptimizerStatus addSemanticLayoutEdges(
    g2o::SparseOptimizer                         &optimizer_inout,
    const std::vector<geometric::Plane *>        &planes_in,
    const std::function<int(geometric::Plane *)> &getVertexId_in,
    const std::vector<RoomSnapshot>              &rooms_in,
    map::Map                                     *p_map_in,
    const types::SystemParams                    *p_systemParams_in,
    const char                                   *p_optimisationName_in)
{
    if (p_systemParams_in == nullptr || p_map_in == nullptr)
    {
        return optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
    }
    const types::SystemParams::Semantics::Factors &factorParameters =
        p_systemParams_in->semantics.factors;
    if (!factorParameters.wallEdge.isEnabled)
    {
        return optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
    }

    /* Wall ends are measured horizontally, across the map's up direction. */
    Eigen::Vector3d upDirection_world{};
    bool            isUpFromImuGravity{};
    if (p_map_in->getUpDirection(upDirection_world, isUpFromImuGravity) !=
            map::MapStatus::MAP_STATUS_SUCCESS ||
        !upDirection_world.allFinite() || upDirection_world.norm() < 1e-8)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: the map has no usable up direction; no semantic "
                     "layout edges.",
                     __func__);
        return optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
    }
    upDirection_world.normalize();

    std::vector<WallSegment3d> wallSegments{};
    buildWallSegments(optimizer_inout,
                      planes_in,
                      getVertexId_in,
                      rooms_in,
                      upDirection_world,
                      p_systemParams_in,
                      wallSegments);

    /*!
     * Huber deltas: square roots of the 95 % chi2 quantiles, as the other
     * bundle-adjustment edges use them.
     */
    const double huberThreshold2d = std::sqrt(5.991);

    std::string edgeCountText{};
    std::size_t cornerEdgeCount{};
    double      cornerInitialChi2{};
    addWallCornerEdges(optimizer_inout,
                       wallSegments,
                       upDirection_world,
                       p_systemParams_in,
                       huberThreshold2d,
                       cornerEdgeCount,
                       cornerInitialChi2);
    edgeCountText += " wall_edge=" + std::to_string(cornerEdgeCount) +
                     " wall_edge_chi2=" + std::to_string(cornerInitialChi2);

    RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                "[SemanticFactors] %s walls=%zu%s",
                p_optimisationName_in,
                wallSegments.size(),
                edgeCountText.c_str());
    return optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
