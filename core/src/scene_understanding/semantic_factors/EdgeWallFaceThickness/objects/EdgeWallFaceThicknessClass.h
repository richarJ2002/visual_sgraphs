/*!
 * @file            EdgeWallFaceThicknessClass.h
 *
 * @brief           Declares EdgeWallFaceThickness, a g2o binary edge enforcing
 *                  minimum thickness between twin wall faces.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEWALLFACETHICKNESS_OBJECTS_EDGEWALLFACETHICKNESSCLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEWALLFACETHICKNESS_OBJECTS_EDGEWALLFACETHICKNESSCLASS_H

#include <Eigen/Geometry>
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
namespace EdgeWallFaceThickness
{

/*!
 * @brief           g2o binary edge that enforces a minimum thickness between
 *                  two VertexPlane vertices believed to be the opposite faces
 *                  of one physical wall.
 *
 * The residual is e = max(0, t_min − s) where s = −(fA + fB) and
 * fA = σA·(nA·m + cA), fB = σB·(nB·m + cB) are the signed distances of the
 * edge anchor from each face plane, with σ chosen so each normal points into
 * its own room. When the faces are already at least t_min apart the edge
 * contributes nothing (one-sided hinge).
 */
class EdgeWallFaceThickness
    : public g2o::BaseBinaryEdge<1, double, g2o::VertexPlane, g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates an empty edge.
     */
    EdgeWallFaceThickness() :
        g2o::BaseBinaryEdge<1, double, g2o::VertexPlane, g2o::VertexPlane>()
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
     * @brief           Midpoint of the two twin-face centroids, world frame.
     *                  Frozen measurement, set by the graph-build code.
     */
    Eigen::Vector3d anchor_m;
    /*!
     * @brief           σA · nA, where nA is face A's unit normal and σA is the
     *                  sign chosen so the normal points into its own room.
     *                  Frozen measurement, set by the graph-build code.
     */
    Eigen::Vector3d signA_world_m;
    /*!
     * @brief           σB · nB, where nB is face B's unit normal and σB is the
     *                  sign chosen so the normal points into its own room.
     *                  Frozen measurement, set by the graph-build code.
     */
    Eigen::Vector3d signB_world_m;
    /*!
     * @brief           Minimum enforced wall thickness, metres (t_min).
     */
    double          minimumThickness_m = 0.08;

    /*!
     * @brief           Computes the thickness error: e = max(0, t_min − s)
     *                  where s is the separation between the faces along the
     *                  anchor. Uses g2o's default numeric Jacobian.
     */
    void computeError() override
    {
        const g2o::VertexPlane *p_plane1Vertex =
            static_cast<const g2o::VertexPlane *>(_vertices[0]);
        const g2o::VertexPlane *p_plane2Vertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        const Eigen::Vector4d coeffsA = p_plane1Vertex->estimate().coeffs();
        const Eigen::Vector4d coeffsB = p_plane2Vertex->estimate().coeffs();

        const Eigen::Vector3d nA = coeffsA.head<3>();
        const Eigen::Vector3d nB = coeffsB.head<3>();
        const double          dA = coeffsA(3);
        const double          dB = coeffsB(3);

        /* signA_world_m == σA · nA at formation; recover the scalar sign,
         * then evaluate the current plane so the residual stays exact when the
         * face rotates during optimisation. */
        const double sigmaA = (signA_world_m.dot(nA) >= 0.0) ? 1.0 : -1.0;
        const double sigmaB = (signB_world_m.dot(nB) >= 0.0) ? 1.0 : -1.0;

        /* f = σ · (n·m + d) */
        const double fA = sigmaA * (nA.dot(anchor_m) + dA);
        const double fB = sigmaB * (nB.dot(anchor_m) + dB);

        /* s = separation between faces; e = max(0, t_min − s) */
        const double s = -(fA + fB);
        _error[0] = (minimumThickness_m > s) ? (minimumThickness_m - s) : 0.0;
    }
};

} /* namespace EdgeWallFaceThickness */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEWALLFACETHICKNESS_OBJECTS_EDGEWALLFACETHICKNESSCLASS_H \
        */
