/*!
 * @file            EdgeRoomWallOverlapClass.h
 *
 * @brief           Declares EdgeRoomWallOverlap, a g2o unary edge that pushes
 *                  a wall of one room out of another room's footprint.
 */

#ifndef VS_GRAPHS_SEMANTIC_FACTORS_EDGE_ROOM_WALL_OVERLAP_OBJECTS_EDGE_ROOM_WALL_OVERLAP_CLASS_H
#define VS_GRAPHS_SEMANTIC_FACTORS_EDGE_ROOM_WALL_OVERLAP_OBJECTS_EDGE_ROOM_WALL_OVERLAP_CLASS_H

#include <Eigen/Core>
#include <istream>
#include <ostream>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/types/vertex_plane.h>

namespace vs_graphs
{
namespace core
{
namespace optimizable_types
{
namespace EdgeRoomWallOverlap
{

/*!
 * @brief           g2o unary edge on the VertexPlane vertex of a wall that
 *                  lies inside the footprint box of a room it does not
 *                  belong to. The box is frozen; only the wall moves, along
 *                  its normal. One-dimensional one-sided hinge, metres:
 *
 *                  position    = exit axis . (anchor moved onto the plane)
 *                  penetration = exit sign * (box side - position)
 *                  error       = max(0, penetration - margin)
 *
 *                  The exit side is the box side nearer to the wall when the
 *                  edge is built. When the wall is at most margin inside the
 *                  box the edge contributes nothing.
 */
class EdgeRoomWallOverlap
    : public g2o::BaseUnaryEdge<1, double, g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates an edge with a zero box; the graph-build code
     *                  sets it.
     */
    EdgeRoomWallOverlap() :
        g2o::BaseUnaryEdge<1, double, g2o::VertexPlane>()
    {}

    /*!
     * @brief           Reads this edge from a g2o graph file stream: the
     *                  information matrix.
     *
     * @param[in,out]   inputStream_inout
     *                  Stream positioned at this edge's data.
     *
     * @return          true always.
     */
    bool read(std::istream &inputStream_inout) override;

    /*!
     * @brief           Writes this edge to a g2o graph file stream: the
     *                  information matrix.
     *
     * @param[in,out]   outputStream_inout
     *                  Stream the edge is appended to.
     *
     * @return          What writeInformationMatrix reports: true when the
     *                  stream is still good.
     */
    bool write(std::ostream &outputStream_inout) const override;

    /*!
     * @brief           Computes the overlap error described on the class
     *                  from the current plane estimate. Uses g2o's default
     *                  numeric Jacobian.
     */
    void computeError() override
    {
        const g2o::VertexPlane *p_wallVertex =
            static_cast<const g2o::VertexPlane *>(_vertices[0]);
        const Eigen::Vector4d wallPlane_world =
            p_wallVertex->estimate().coeffs();
        const Eigen::Vector3d wallNormal_world = wallPlane_world.head<3>();

        /*!
         * Plane3D keeps a unit normal, so the anchor moves onto the current
         * plane by its signed distance along the normal.
         */
        const Eigen::Vector3d anchorOnWall_world_m =
            anchor_world_m -
            (wallNormal_world.dot(anchor_world_m) + wallPlane_world(3)) *
                wallNormal_world;
        const double wallPosition_m = exitAxis_world.dot(anchorOnWall_world_m);
        const double penetration_m =
            exitSign * (boxSide_m - wallPosition_m) - margin_m;
        _error[0] = penetration_m > 0.0 ? penetration_m : 0.0;
    }

    /*!
     * @brief           Midpoint of the wall's horizontal segment when the
     *                  edge was built. Frozen measurement.
     *
     * @frame           World
     *
     * @units           metres
     */
    Eigen::Vector3d anchor_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Footprint axis the wall leaves the box along, unit
     *                  length, horizontal. Frozen measurement.
     *
     * @frame           World
     */
    Eigen::Vector3d exitAxis_world{Eigen::Vector3d::UnitX()};

    /*!
     * @brief           Position of the exit box side along exitAxis_world.
     *                  Frozen measurement.
     *
     * @frame           World, projected onto exitAxis_world
     *
     * @units           metres
     */
    double boxSide_m{0.0};

    /*!
     * @brief           +1 when the exit side is the box maximum along
     *                  exitAxis_world, -1 when it is the minimum.
     */
    double exitSign{1.0};

    /*!
     * @brief           Overlap tolerated before the penalty starts, >= 0.
     *
     * @units           metres
     */
    double margin_m{0.0};
};

} /* namespace EdgeRoomWallOverlap */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTIC_FACTORS_EDGE_ROOM_WALL_OVERLAP_OBJECTS_EDGE_ROOM_WALL_OVERLAP_CLASS_H \
        */
