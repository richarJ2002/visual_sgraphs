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
 * @file            EdgeVertexPlaneProjectSE3KFClass.h
 *
 * @brief           Declares EdgeVertexPlaneProjectSE3KF, a g2o edge/vertex
 * type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXPLANEPROJECTSE3KF_OBJECTS_EDGEVERTEXPLANEPROJECTSE3KFCLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXPLANEPROJECTSE3KF_OBJECTS_EDGEVERTEXPLANEPROJECTSE3KFCLASS_H

#include <vs_graphs_g2o/core/base_multi_edge.h>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/types/isometry3d_mappings.h>
#include "optimizable_types/EdgeVertexPlaneProjectSE3KF/objects/EdgeVertexPlaneProjectSE3KFStatusEnum.h"
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
namespace EdgeVertexPlaneProjectSE3KF
{

/*!
 * The edge used to connect a Plane vertex (VertexPlane) to a KeyFrame vertex
 * (SE3) [Note]: it creates constraint for three measurements, i.e., (x, y, z)
 */
class EdgeVertexPlaneProjectSE3KF
    : public g2o::BaseBinaryEdge<3,
                                 g2o::Plane3D,
                                 g2o::VertexSE3Expmap,
                                 g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates an empty edge.
     */
    EdgeVertexPlaneProjectSE3KF() :
        g2o::BaseBinaryEdge<3,
                            g2o::Plane3D,
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
     * @brief           Stores the plane that the key frame is expected to
     *                  observe.
     *
     * @param[in]       m_in
     *                  Measured plane in the key frame.
     */
    void setMeasurement(const g2o::Plane3D &m_in) override
    {
        _measurement = m_in;
    }

    /*!
     * @brief           Computes the plane error: the plane vertex seen from the
     *                  key frame, minus the measured plane, using the g2o plane
     *                  difference.
     */
    void computeError() override
    {
        /* KeyFrame's global pose */
        const g2o::VertexSE3Expmap *p_keyFrameGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        /* Plane's global pose */
        const g2o::VertexPlane *p_planeGpVertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        /* Calculating poses (in global frame) */
        Eigen::Isometry3d keyFramePose = p_keyFrameGpVertex->estimate();
        g2o::Plane3D localPlane = keyFramePose * p_planeGpVertex->estimate();

        /* Calculating the error */
        _error = localPlane.ominus(_measurement);
    }

    /*!
     * @brief           Checks whether the plane distance d, seen from the key
     *                  frame, is positive.
     *
     * @param[out]      isDistanceCorrect_out
     *                  True when the plane distance in the key frame is
     *                  positive.
     *
     * @return          EDGE_VERTEX_PLANE_PROJECT_SE3_KFSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeVertexPlaneProjectSE3KFStatus
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
        return EdgeVertexPlaneProjectSE3KFStatus::
            EDGE_VERTEX_PLANE_PROJECT_SE3_KFSTATUS_SUCCESS;
    }
};

} /* namespace EdgeVertexPlaneProjectSE3KF */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXPLANEPROJECTSE3KF_OBJECTS_EDGEVERTEXPLANEPROJECTSE3KFCLASS_H \
        */
