/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            EdgeVertexPlaneProjectSE3MClass.h
 *
 * @brief           Declares EdgeVertexPlaneProjectSE3M, a g2o edge/vertex type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXPLANEPROJECTSE3M_OBJECTS_EDGEVERTEXPLANEPROJECTSE3MCLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXPLANEPROJECTSE3M_OBJECTS_EDGEVERTEXPLANEPROJECTSE3MCLASS_H

#include <vs_graphs_g2o/core/base_multi_edge.h>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/types/isometry3d_mappings.h>
#include <Eigen/Geometry>
#include <vs_graphs_g2o/types/sim3.h>
#include <vs_graphs_g2o/types/types_six_dof_expmap.h>
#include <vs_graphs_g2o/types/vertex_plane.h>
#include <istream>
#include <ostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace optimizable_types
{
namespace EdgeVertexPlaneProjectSE3M
{

/*!
 * The edge used to connect a Plane vertex (VertexPlane) to a Marker vertex
 * (SE3) [Note]: it creates constraint for four measurements, i.e., (x, y, z, d)
 * 🚧 [vS-Graphs v1.5] Unused.
 */
class EdgeVertexPlaneProjectSE3M
    : public g2o::BaseBinaryEdge<4,
                                 Eigen::Vector4d,
                                 g2o::VertexSE3Expmap,
                                 g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates an empty edge.
     */
    EdgeVertexPlaneProjectSE3M() :
        g2o::BaseBinaryEdge<4,
                            Eigen::Vector4d,
                            g2o::VertexSE3Expmap,
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
     * @brief           Computes the 4D error between the plane seen from the
     *                  marker and the marker normal (three components), plus
     *                  the plane distance d in the marker frame.
     */
    void computeError() override
    {
        /* Marker's global pose */
        const g2o::VertexSE3Expmap *p_markerGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        /* Plane's global pose */
        const g2o::VertexPlane *p_planeGpVertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        /* Calculating poses (in global frame) */
        g2o::Isometry3D markerPose  = p_markerGpVertex->estimate();
        g2o::Vector4D   planeCoeffs = p_planeGpVertex->estimate().coeffs();

        /* Normalize the plane vector if necessary */
        if (planeCoeffs(3) < 0)
            planeCoeffs *= -1;

        /* Create the plane plane in global frame */
        g2o::Plane3D plane_g(planeCoeffs);

        /* Calculate the plane in marker's frame */
        g2o::Plane3D plane_m = markerPose.inverse() * plane_g;

        /* Calculate the normal of the marker in marker's frame */
        g2o::Plane3D markerNormal_m =
            markerPose.inverse() * markerPose.matrix().col(2);

        /* Calculate the difference of the planes */
        Eigen::Vector3d planeDiff =
            plane_m.coeffs().head(3) - markerNormal_m.coeffs().head(3);

        _error[0] = planeDiff(0);
        _error[1] = planeDiff(1);
        _error[2] = planeDiff(2);
        _error[3] = plane_m.coeffs()(3); /* Distance (d) of the cacmera */
    }
};

} /* namespace EdgeVertexPlaneProjectSE3M */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXPLANEPROJECTSE3M_OBJECTS_EDGEVERTEXPLANEPROJECTSE3MCLASS_H \
        */
