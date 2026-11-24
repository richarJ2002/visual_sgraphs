/*!
 * @file            private_functions.h
 *
 * @brief           Declares the semantic-factor helpers used only inside this
 *                  package: the wall segments of a bundle adjustment and the
 *                  builders of the semantic layout edges.
 */

#ifndef VS_GRAPHS_SEMANTIC_FACTORS_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_SEMANTIC_FACTORS_PRIVATE_FUNCTIONS_H

#include <cstddef>
#include <functional>
#include <vector>

#include <Eigen/Core>

#include "scene_understanding/semantic_factors/objects/WallSegment3dStruct.h"
#include "scene_understanding/semantics_manager/objects/RoomSnapshotStruct.h"

namespace vs_graphs::core::geometric
{
class Plane;
} /* namespace vs_graphs::core::geometric */
namespace vs_graphs::core::types
{
class SystemParams;
} /* namespace vs_graphs::core::types */
namespace g2o
{
class SparseOptimizer;
} /* namespace g2o */

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Builds the horizontal segment of every wall that has a
 *                  plane vertex in the optimiser, from the wall's support
 *                  points (buildFiniteWallSegment2d, with the room-boundary
 *                  trim ratio and minimum wall length).
 *
 * @param[in]       optimizer_in
 *                  g2o optimiser holding the plane vertices.
 *
 * @param[in]       planes_in
 *                  Planes with vertices in this optimisation; borrowed. Bad,
 *                  non-wall and too short planes are skipped.
 *
 * @param[in]       getVertexId_in
 *                  Callable returning a plane's optimiser vertex id.
 *
 * @param[in]       rooms_in
 *                  Snapshots of the map's rooms, giving each wall's rooms.
 *
 * @param[in]       upDirection_world_in
 *                  Unit up direction of the map, world frame.
 *
 * @param[in]       p_systemParams_in
 *                  System parameters (room_seg.boundary_topology); not null.
 *
 * @param[out]      wallSegments_out
 *                  Receives one segment per usable wall; replaced.
 */
void buildWallSegments(
    g2o::SparseOptimizer                         &optimizer_in,
    const std::vector<geometric::Plane *>        &planes_in,
    const std::function<int(geometric::Plane *)> &getVertexId_in,
    const std::vector<RoomSnapshot>              &rooms_in,
    const Eigen::Vector3d                        &upDirection_world_in,
    const types::SystemParams                    *p_systemParams_in,
    std::vector<WallSegment3d>                   &wallSegments_out);

/*!
 * @brief           Adds one EdgeWallCorner between each pair of crossing
 *                  walls (more than 45 degrees apart) of one room whose
 *                  nearest end points lie within
 *                  semantics.factors.wall_edge.max_gap_m of each other,
 *                  horizontally. Walls of different rooms are not paired: at
 *                  a corner of two rooms, a wall's end also lies a wall
 *                  thickness from the neighbouring room's crossing face.
 *
 * @param[in,out]   optimizer_inout
 *                  g2o optimiser owning the plane vertices; takes ownership
 *                  of the new edges and robust kernels.
 *
 * @param[in]       wallSegments_in
 *                  Walls of this optimisation (buildWallSegments).
 *
 * @param[in]       upDirection_world_in
 *                  Unit up direction of the map, world frame.
 *
 * @param[in]       p_systemParams_in
 *                  System parameters (semantics.factors.wall_edge); not
 *                  null.
 *
 * @param[in]       huberThreshold2d_in
 *                  Huber delta for the two-dimensional edges.
 *
 * @param[out]      edgeCount_out
 *                  Number of edges added.
 *
 * @param[out]      initialChi2_out
 *                  Sum of the added edges' chi2 before optimisation.
 */
void addWallCornerEdges(g2o::SparseOptimizer             &optimizer_inout,
                        const std::vector<WallSegment3d> &wallSegments_in,
                        const Eigen::Vector3d            &upDirection_world_in,
                        const types::SystemParams        *p_systemParams_in,
                        double                            huberThreshold2d_in,
                        std::size_t                      &edgeCount_out,
                        double                           &initialChi2_out);

/*!
 * @brief           Adds one EdgeRoomWallOverlap for each wall of another room
 *                  whose segment lies more than
 *                  semantics.factors.room_overlap.margin_m inside the wall
 *                  footprint box (computeRoomWallFootprint) of a COMPLETE
 *                  room. A room is used only when all its walls are in this
 *                  optimisation and each lies within max_wall_inset_m of a
 *                  box side, i.e. the room is a box.
 *
 * @param[in,out]   optimizer_inout
 *                  g2o optimiser owning the plane vertices; takes ownership
 *                  of the new edges and robust kernels.
 *
 * @param[in]       planes_in
 *                  Planes with vertices in this optimisation; borrowed. The
 *                  room walls are resolved among them.
 *
 * @param[in]       wallSegments_in
 *                  Walls of this optimisation (buildWallSegments).
 *
 * @param[in]       rooms_in
 *                  Snapshots of the map's rooms; read only.
 *
 * @param[in]       upDirection_world_in
 *                  Unit up direction of the map, world frame.
 *
 * @param[in]       p_systemParams_in
 *                  System parameters (semantics.factors.room_overlap); not
 *                  null.
 *
 * @param[in]       huberThreshold1d_in
 *                  Huber delta for the one-dimensional edges.
 *
 * @param[out]      edgeCount_out
 *                  Number of edges added.
 *
 * @param[out]      initialChi2_out
 *                  Sum of the added edges' chi2 before optimisation.
 */
void addRoomOverlapEdges(g2o::SparseOptimizer                  &optimizer_inout,
                         const std::vector<geometric::Plane *> &planes_in,
                         const std::vector<WallSegment3d>      &wallSegments_in,
                         const std::vector<RoomSnapshot>       &rooms_in,
                         const Eigen::Vector3d     &upDirection_world_in,
                         const types::SystemParams *p_systemParams_in,
                         double                     huberThreshold1d_in,
                         std::size_t               &edgeCount_out,
                         double                    &initialChi2_out);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTIC_FACTORS_PRIVATE_FUNCTIONS_H */
