/*!
 * @file            buildWallSegments.cc
 *
 * @brief           Implements buildWallSegments(), declared in
 *                  private_functions.h.
 */

#include "scene_understanding/semantic_factors/private_functions/private_functions.h"

#include <cmath>
#include <functional>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/types/vertex_plane.h>

#include "configuration/types/types.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/public_functions/public_functions.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

void buildWallSegments(
    g2o::SparseOptimizer                         &optimizer_in,
    const std::vector<geometric::Plane *>        &planes_in,
    const std::function<int(geometric::Plane *)> &getVertexId_in,
    const std::vector<RoomSnapshot>              &rooms_in,
    const Eigen::Vector3d                        &upDirection_world_in,
    const types::SystemParams                    *p_systemParams_in,
    std::vector<WallSegment3d>                   &wallSegments_out)
{
    wallSegments_out.clear();

    /*!
     * The segments are measured on the two ground axes, which are the
     * in-plane axes of a plane facing up.
     */
    const geometric::InPlaneAxes groundAxes =
        geometric::computeInPlaneAxes(upDirection_world_in);
    const types::SystemParams::RoomSeg::BoundaryTopology &topologyParameters =
        p_systemParams_in->roomSeg.boundaryTopology;

    /* Rooms of each wall; the snapshots come in ascending room id. */
    std::unordered_map<int, std::vector<int>> roomIdsByWallId{};
    for (const RoomSnapshot &room : rooms_in)
    {
        for (const int wallPlaneId : room.wallPlaneIds)
        {
            roomIdsByWallId[wallPlaneId].push_back(room.id);
        }
    }

    for (geometric::Plane *p_plane : planes_in)
    {
        bool                           isPlaneBad{};
        geometric::Plane::PlaneVariant planeType{};
        if (p_plane == nullptr ||
            p_plane->isBad(isPlaneBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            isPlaneBad ||
            p_plane->getPlaneType(planeType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            planeType != geometric::Plane::PlaneVariant::WALL)
        {
            continue;
        }

        /* g2o ids are shared across vertex types; check the type. */
        const int vertexId = getVertexId_in(p_plane);
        if (vertexId < 0)
        {
            continue;
        }
        g2o::VertexPlane *p_wallVertex =
            dynamic_cast<g2o::VertexPlane *>(optimizer_in.vertex(vertexId));
        if (p_wallVertex == nullptr)
        {
            continue;
        }

        FiniteWallSegment2d groundSegment{};
        bool                isSegmentBuilt{};
        if (buildFiniteWallSegment2d(p_plane,
                                     upDirection_world_in,
                                     groundAxes.planeAxisU_world,
                                     groundAxes.planeAxisV_world,
                                     topologyParameters.endpointTrimRatio,
                                     topologyParameters.minimumWallLength_m,
                                     groundSegment,
                                     isSegmentBuilt) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: buildFiniteWallSegment2d returned a failure "
                         "status although it cannot fail; skipping the wall.",
                         __func__);
            continue;
        }
        Eigen::Vector3d wallCentroid_world_m{};
        WallSegment3d   wallSegment{};
        if (!isSegmentBuilt ||
            p_plane->getCentroid(wallCentroid_world_m) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            p_plane->getId(wallSegment.wallId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            continue;
        }

        /*!
         * The segment lies at the centroid height: its ends are the
         * centroid moved along the wall's horizontal tangent.
         */
        const Eigen::Vector3d heightOffset_world_m =
            wallCentroid_world_m.dot(upDirection_world_in) *
            upDirection_world_in;
        wallSegment.wallStart_world_m =
            groundSegment.wallStart_world_m.x() * groundAxes.planeAxisU_world +
            groundSegment.wallStart_world_m.y() * groundAxes.planeAxisV_world +
            heightOffset_world_m;
        wallSegment.wallEnd_world_m =
            groundSegment.wallEnd_world_m.x() * groundAxes.planeAxisU_world +
            groundSegment.wallEnd_world_m.y() * groundAxes.planeAxisV_world +
            heightOffset_world_m;

        const Eigen::Vector3d wallNormal_world =
            p_wallVertex->estimate().normal();
        const Eigen::Vector3d horizontalNormal_world =
            wallNormal_world -
            wallNormal_world.dot(upDirection_world_in) * upDirection_world_in;
        if (horizontalNormal_world.norm() < 1e-3)
        {
            continue;
        }
        wallSegment.horizontalNormal_world =
            horizontalNormal_world.normalized();
        const std::unordered_map<int, std::vector<int>>::const_iterator
            wallRoomsIterator = roomIdsByWallId.find(wallSegment.wallId);
        if (wallRoomsIterator != roomIdsByWallId.end())
        {
            wallSegment.roomIds = wallRoomsIterator->second;
        }
        wallSegment.p_wall       = p_plane;
        wallSegment.p_wallVertex = p_wallVertex;
        wallSegments_out.push_back(wallSegment);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
