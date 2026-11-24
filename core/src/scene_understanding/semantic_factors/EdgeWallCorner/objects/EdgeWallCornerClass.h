/*!
 * @file            EdgeWallCornerClass.h
 *
 * @brief           Declares EdgeWallCorner, a g2o binary edge that makes two
 *                  crossing walls meet at the corner their end points mark.
 */

#ifndef VS_GRAPHS_SEMANTIC_FACTORS_EDGE_WALL_CORNER_OBJECTS_EDGE_WALL_CORNER_CLASS_H
#define VS_GRAPHS_SEMANTIC_FACTORS_EDGE_WALL_CORNER_OBJECTS_EDGE_WALL_CORNER_CLASS_H

#include <Eigen/Core>
#include <istream>
#include <ostream>
#include <vs_graphs_g2o/core/base_binary_edge.h>
#include <vs_graphs_g2o/types/vertex_plane.h>

namespace vs_graphs
{
namespace core
{
namespace optimizable_types
{
namespace EdgeWallCorner
{

/*!
 * @brief           g2o binary edge between the VertexPlane vertices of two
 *                  crossing walls whose end points nearly meet. Each wall's
 *                  end point, moved onto the wall's current plane, shall lie
 *                  on the other wall's plane: then the walls meet at the
 *                  corner. Two-dimensional error, metres:
 *
 *                  error[0] = signed distance of wall A's end point to
 *                             plane B;
 *                  error[1] = signed distance of wall B's end point to
 *                             plane A.
 *
 *                  The sign of a plane normal does not matter: the squared
 *                  error is the same for either sign.
 */
class EdgeWallCorner
    : public g2o::
          BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPlane, g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates an edge with both end points at the origin; the
     *                  graph-build code sets them.
     */
    EdgeWallCorner() :
        g2o::BaseBinaryEdge<2,
                            Eigen::Vector2d,
                            g2o::VertexPlane,
                            g2o::VertexPlane>()
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
     * @brief           Computes the corner error described on the class from
     *                  the current plane estimates. Uses g2o's default
     *                  numeric Jacobian.
     */
    void computeError() override
    {
        const g2o::VertexPlane *p_wallVertexA =
            static_cast<const g2o::VertexPlane *>(_vertices[0]);
        const g2o::VertexPlane *p_wallVertexB =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);
        const Eigen::Vector4d planeA_world = p_wallVertexA->estimate().coeffs();
        const Eigen::Vector4d planeB_world = p_wallVertexB->estimate().coeffs();
        const Eigen::Vector3d normalA_world = planeA_world.head<3>();
        const Eigen::Vector3d normalB_world = planeB_world.head<3>();

        /*!
         * Plane3D keeps unit normals, so normal . point + offset is the
         * signed distance in metres. The end points are moved onto their own
         * wall first, so each wall's own motion along its normal carries its
         * end point with it.
         */
        const Eigen::Vector3d endPointOnA_world_m =
            endPointA_world_m -
            (normalA_world.dot(endPointA_world_m) + planeA_world(3)) *
                normalA_world;
        const Eigen::Vector3d endPointOnB_world_m =
            endPointB_world_m -
            (normalB_world.dot(endPointB_world_m) + planeB_world(3)) *
                normalB_world;
        _error[0] = normalB_world.dot(endPointOnA_world_m) + planeB_world(3);
        _error[1] = normalA_world.dot(endPointOnB_world_m) + planeA_world(3);
    }

    /*!
     * @brief           End point of wall A nearest to wall B, at wall A's
     *                  centroid height. Frozen measurement, set by the
     *                  graph-build code.
     *
     * @frame           World
     *
     * @units           metres
     */
    Eigen::Vector3d endPointA_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           End point of wall B nearest to wall A, at wall B's
     *                  centroid height. Frozen measurement, set by the
     *                  graph-build code.
     *
     * @frame           World
     *
     * @units           metres
     */
    Eigen::Vector3d endPointB_world_m{Eigen::Vector3d::Zero()};
};

} /* namespace EdgeWallCorner */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTIC_FACTORS_EDGE_WALL_CORNER_OBJECTS_EDGE_WALL_CORNER_CLASS_H \
        */
