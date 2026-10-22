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
 * @file            EdgeSE3KFPointToPlaneClass.h
 *
 * @brief           Declares EdgeSE3KFPointToPlane, a g2o edge/vertex type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3KFPOINTTOPLANE_OBJECTS_EDGESE3KFPOINTTOPLANECLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3KFPOINTTOPLANE_OBJECTS_EDGESE3KFPOINTTOPLANECLASS_H

#include <vs_graphs_g2o/core/base_multi_edge.h>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/types/isometry3d_mappings.h>
#include "optimization/optimizable_types/EdgeSE3KFPointToPlane/objects/EdgeSE3KFPointToPlaneStatusEnum.h"
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
namespace EdgeSE3KFPointToPlane
{

/*!
 * The edge used to connect a Plane vertex (VertexPlane) to a KeyFrame point
 * (SE3) [Note]: it creates constraint connecting the points in a plane
 * observation to the plane
 */
class EdgeSE3KFPointToPlane : public g2o::BaseBinaryEdge<1,
                                                         Eigen::Matrix4d,
                                                         g2o::VertexSE3Expmap,
                                                         g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates an empty edge.
     */
    EdgeSE3KFPointToPlane() :
        g2o::BaseBinaryEdge<1,
                            Eigen::Matrix4d,
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
     * @brief           Stores the measurement matrix that the point-to-plane
     *                  error is evaluated against.
     *
     * @param[in]       m_in
     *                  4x4 measurement matrix of the plane observation.
     */
    void setMeasurement(const Eigen::Matrix4d &m_in) override
    {
        _measurement = m_in;
    }

    /*!
     * @brief           Computes the scalar point-to-plane error from the plane
     *                  coefficients, the inverse key frame pose and the
     *                  measurement matrix.
     */
    void computeError() override
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::VertexPlane *v2 =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        Eigen::Matrix4d Ti  = v1->estimate().inverse().to_homogeneous_matrix();
        Eigen::Vector4d Pj  = v2->estimate().coeffs();
        Eigen::Matrix4d Gij = _measurement;
        _error              = Pj.transpose() * Ti * Gij * Ti.transpose() * Pj;
    }

    /*!
     * @brief           Checks whether the plane distance d, seen from the key
     *                  frame, is positive.
     *
     * @param[out]      isDistanceCorrect_out
     *                  True when the plane distance in the key frame is
     *                  positive.
     *
     * @return          EDGE_SE3_KFPOINT_TO_PLANE_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeSE3KFPointToPlaneStatus
        isDistanceCorrect(bool &isDistanceCorrect_out)
    {
        const g2o::VertexSE3Expmap *p_keyFrameGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::VertexPlane *p_planeGpVertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        /* local plane equation */
        Eigen::Isometry3d keyFramePose = p_keyFrameGpVertex->estimate();
        g2o::Plane3D localPlane = keyFramePose * p_planeGpVertex->estimate();

        isDistanceCorrect_out = (localPlane.coeffs()(3) > 0);
        return EdgeSE3KFPointToPlaneStatus::
            EDGE_SE3_KFPOINT_TO_PLANE_STATUS_SUCCESS;
    }
};

} /* namespace EdgeSE3KFPointToPlane */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3KFPOINTTOPLANE_OBJECTS_EDGESE3KFPOINTTOPLANECLASS_H \
        */
