/*!
 * @file            addRoomOverlapEdges.cc
 *
 * @brief           Implements addRoomOverlapEdges(), declared in
 *                  private_functions.h.
 */

#include "scene_understanding/semantic_factors/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <vector>

#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/types/vertex_plane.h>

#include "configuration/types/types.h"
#include "scene_understanding/semantic_factors/EdgeRoomWallOverlap/EdgeRoomWallOverlap.h"
#include "scene_understanding/semantic_factors/public_functions/public_functions.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

void addRoomOverlapEdges(g2o::SparseOptimizer                  &optimizer_inout,
                         const std::vector<geometric::Plane *> &planes_in,
                         const std::vector<WallSegment3d>      &wallSegments_in,
                         const std::vector<RoomSnapshot>       &rooms_in,
                         const Eigen::Vector3d     &upDirection_world_in,
                         const types::SystemParams *p_systemParams_in,
                         double                     huberThreshold1d_in,
                         std::size_t               &edgeCount_out,
                         double                    &initialChi2_out)
{
    edgeCount_out   = 0U;
    initialChi2_out = 0.0;
    const types::SystemParams::Semantics::Factors::RoomOverlap
        &overlapParameters = p_systemParams_in->semantics.factors.roomOverlap;
    const double margin_m  = overlapParameters.margin_m;
    const double maxWallInset_m = overlapParameters.maxWallInset_m;

    std::unordered_map<int, geometric::Plane *> planesById{};
    indexPlanesById(planes_in, planesById);

    for (const RoomSnapshot &room : rooms_in)
    {
        /*!
         * Only a closed boundary bounds the room's floor area, and the box
         * is taken only from all of the room's walls: in local BA a room
         * outside the window has walls without vertices.
         */
        std::vector<geometric::Plane *> roomWalls{};
        resolveRoomWallPlanes(room, planesById, roomWalls);
        if (!room.isBoundaryComplete || roomWalls.empty() ||
            roomWalls.size() != room.wallPlaneIds.size())
        {
            continue;
        }
        RoomWallFootprint roomFootprint{};
        bool              hasRoomFootprint{};
        if (computeRoomWallFootprint(roomWalls,
                                     upDirection_world_in,
                                     roomFootprint,
                                     hasRoomFootprint) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS ||
            !hasRoomFootprint)
        {
            continue;
        }

        /*!
         * The box is the room only when every wall of the room lies on a
         * box side; a wall well inside (an L-shaped room) means the box
         * also covers the neighbours' floor. Each wall is measured along
         * the box axis nearer to its normal.
         */
        bool isRoomBox = true;
        for (const WallSegment3d &wallSegment : wallSegments_in)
        {
            if (!std::binary_search(wallSegment.roomIds.begin(),
                                    wallSegment.roomIds.end(),
                                    room.id))
            {
                continue;
            }
            const bool isFirstAxis =
                std::abs(wallSegment.horizontalNormal_world.dot(
                    roomFootprint.firstAxis_world)) >=
                std::abs(wallSegment.horizontalNormal_world.dot(
                    roomFootprint.secondAxis_world));
            const Eigen::Vector3d wallMidpoint_world_m =
                0.5 *
                (wallSegment.wallStart_world_m + wallSegment.wallEnd_world_m);
            const double wallPosition_m =
                isFirstAxis
                    ? wallMidpoint_world_m.dot(roomFootprint.firstAxis_world)
                    : wallMidpoint_world_m.dot(roomFootprint.secondAxis_world);
            const double boxMinimum_m = isFirstAxis
                                            ? roomFootprint.minimumFirst_m
                                            : roomFootprint.minimumSecond_m;
            const double boxMaximum_m = isFirstAxis
                                            ? roomFootprint.maximumFirst_m
                                            : roomFootprint.maximumSecond_m;
            isRoomBox = isRoomBox && std::min(wallPosition_m - boxMinimum_m,
                                              boxMaximum_m - wallPosition_m) <=
                                         maxWallInset_m;
        }
        if (!isRoomBox)
        {
            continue;
        }

        for (const WallSegment3d &wallSegment : wallSegments_in)
        {
            /* Only walls of another room; a wall of no room is not judged. */
            if (wallSegment.roomIds.empty() ||
                std::binary_search(wallSegment.roomIds.begin(),
                                   wallSegment.roomIds.end(),
                                   room.id))
            {
                continue;
            }

            /*!
             * The wall leaves the box along the axis nearer to its normal
             * (the only direction it moves in); it must also overlap the
             * box by more than the margin along the other axis.
             */
            const bool isFirstAxis =
                std::abs(wallSegment.horizontalNormal_world.dot(
                    roomFootprint.firstAxis_world)) >=
                std::abs(wallSegment.horizontalNormal_world.dot(
                    roomFootprint.secondAxis_world));
            const Eigen::Vector3d &exitAxis_world =
                isFirstAxis ? roomFootprint.firstAxis_world
                            : roomFootprint.secondAxis_world;
            const Eigen::Vector3d &alongAxis_world =
                isFirstAxis ? roomFootprint.secondAxis_world
                            : roomFootprint.firstAxis_world;
            const double exitMinimum_m  = isFirstAxis
                                              ? roomFootprint.minimumFirst_m
                                              : roomFootprint.minimumSecond_m;
            const double exitMaximum_m  = isFirstAxis
                                              ? roomFootprint.maximumFirst_m
                                              : roomFootprint.maximumSecond_m;
            const double alongMinimum_m = isFirstAxis
                                              ? roomFootprint.minimumSecond_m
                                              : roomFootprint.minimumFirst_m;
            const double alongMaximum_m = isFirstAxis
                                              ? roomFootprint.maximumSecond_m
                                              : roomFootprint.maximumFirst_m;

            const double wallStartAlong_m =
                wallSegment.wallStart_world_m.dot(alongAxis_world);
            const double wallEndAlong_m =
                wallSegment.wallEnd_world_m.dot(alongAxis_world);
            const double alongOverlap_m =
                std::min(std::max(wallStartAlong_m, wallEndAlong_m),
                         alongMaximum_m) -
                std::max(std::min(wallStartAlong_m, wallEndAlong_m),
                         alongMinimum_m);
            const Eigen::Vector3d wallMidpoint_world_m =
                0.5 *
                (wallSegment.wallStart_world_m + wallSegment.wallEnd_world_m);
            const double wallPosition_m =
                wallMidpoint_world_m.dot(exitAxis_world);
            const double depthFromMinimum_m = wallPosition_m - exitMinimum_m;
            const double depthFromMaximum_m = exitMaximum_m - wallPosition_m;
            if (alongOverlap_m <= margin_m ||
                std::min(depthFromMinimum_m, depthFromMaximum_m) <= margin_m)
            {
                continue;
            }

            optimizable_types::EdgeRoomWallOverlap::EdgeRoomWallOverlap
                *p_overlapEdge = new optimizable_types::EdgeRoomWallOverlap::
                    EdgeRoomWallOverlap();
            p_overlapEdge->setVertex(0, wallSegment.p_wallVertex);
            p_overlapEdge->anchor_world_m = wallMidpoint_world_m;
            p_overlapEdge->exitAxis_world = exitAxis_world;
            p_overlapEdge->boxSide_m = depthFromMaximum_m <= depthFromMinimum_m
                                           ? exitMaximum_m
                                           : exitMinimum_m;
            p_overlapEdge->exitSign =
                depthFromMaximum_m <= depthFromMinimum_m ? 1.0 : -1.0;
            p_overlapEdge->margin_m = margin_m;
            p_overlapEdge->setInformation(
                Eigen::Matrix<double, 1, 1>::Identity() *
                overlapParameters.informationGain);
            g2o::RobustKernelHuber *p_overlapKernel =
                new g2o::RobustKernelHuber;
            p_overlapKernel->setDelta(huberThreshold1d_in);
            p_overlapEdge->setRobustKernel(p_overlapKernel);
            optimizer_inout.addEdge(p_overlapEdge);
            p_overlapEdge->computeError();
            initialChi2_out += p_overlapEdge->chi2();
            edgeCount_out++;
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
