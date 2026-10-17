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
 * @file            EdgeSE3ProjectSE3Class.h
 *
 * @brief           Declares EdgeSE3ProjectSE3, a g2o edge/vertex type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3PROJECTSE3_OBJECTS_EDGESE3PROJECTSE3CLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3PROJECTSE3_OBJECTS_EDGESE3PROJECTSE3CLASS_H

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
namespace EdgeSE3ProjectSE3
{

/*!
 * The edge used to connect a Marker vertex (SE3) to a KeyFrame vertex (SE3)
 * [Note]: it creates constraint for six measurements, i.e., (x, y, z, roll,
 * pitch, yaw) 🚧 [vS-Graphs v1.5] Deprecated, but extended for other
 * optimization constraints.
 */
class EdgeSE3ProjectSE3 : public g2o::BaseBinaryEdge<6,
                                                     g2o::Isometry3D,
                                                     g2o::VertexSE3Expmap,
                                                     g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3ProjectSE3() :
        g2o::BaseBinaryEdge<6,
                            g2o::Isometry3D,
                            g2o::VertexSE3Expmap,
                            g2o::VertexSE3Expmap>()
    {}
    /*!
     * @brief           Reads the measurement pose (translation and unit
     *                  quaternion) and the information matrix from a g2o graph
     *                  file stream.
     *
     * @param[in,out]   inputStream_inout
     *                  Stream positioned at this edge's data.
     *
     * @return          false if the stream is bad after reading the pose;
     *                  otherwise true when the stream is still good or has
     *                  reached the end.
     */
    bool read(std::istream &inputStream_inout) override;
    /*!
     * @brief           Writes this edge to a g2o graph file stream: the
     *                  measurement pose (translation and quaternion), then the
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
     * @brief           Stores the measured pose of the marker in the key frame.
     *
     * @param[in]       m_in
     *                  Measured pose as an isometry.
     */
    void setMeasurement(const g2o::Isometry3D &m_in) override
    {
        _measurement = m_in;
    }

    /*!
     * @brief           Computes the 6D pose error: the measured pose compared
     *                  with the marker pose seen from the key frame, i.e. key
     *                  frame global pose times marker global pose.
     */
    void computeError() override
    {
        /* Marker's global pose */
        const g2o::VertexSE3Expmap *p_markerGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        /* KeyFrame's global pose */
        const g2o::VertexSE3Expmap *p_keyFrameGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);

        /* Calculate the local pose of the marker w.r.t. the keyframe */
        g2o::SE3Quat markerLp =
            p_keyFrameGpVertex->estimate() * p_markerGpVertex->estimate();

        g2o::Isometry3D markerLpIso = g2o::Isometry3D::Identity();
        markerLpIso.matrix()        = markerLp.to_homogeneous_matrix();
        /*!
         * Calculating the transformation between the measuremenent and the
         * marker's local pose
         */
        g2o::Isometry3D delta = _measurement.inverse() * markerLpIso;

        /* Calculating the final error */
        _error = g2o::internal::toVectorMQT(delta);
    }
};

} /* namespace EdgeSE3ProjectSE3 */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3PROJECTSE3_OBJECTS_EDGESE3PROJECTSE3CLASS_H \
        */
