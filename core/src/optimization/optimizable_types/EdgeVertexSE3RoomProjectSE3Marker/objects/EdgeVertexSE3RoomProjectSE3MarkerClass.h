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
 * @file            EdgeVertexSE3RoomProjectSE3MarkerClass.h
 *
 * @brief           Declares EdgeVertexSE3RoomProjectSE3Marker, a g2o
 * edge/vertex type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXSE3ROOMPROJECTSE3MARKER_OBJECTS_EDGEVERTEXSE3ROOMPROJECTSE3MARKERCLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXSE3ROOMPROJECTSE3MARKER_OBJECTS_EDGEVERTEXSE3ROOMPROJECTSE3MARKERCLASS_H

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
namespace EdgeVertexSE3RoomProjectSE3Marker
{

/*!
 * The edge used to connect a Room's centroid (SE3) to a Marker vertex (SE3)
 * [Note]: it creates constraint for four measurements, i.e., (x, y, z, d)
 */
class EdgeVertexSE3RoomProjectSE3Marker
    : public g2o::BaseBinaryEdge<4,
                                 Eigen::Vector4d,
                                 g2o::VertexSE3Expmap,
                                 g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates an empty edge.
     */
    EdgeVertexSE3RoomProjectSE3Marker() :
        g2o::BaseBinaryEdge<4,
                            Eigen::Vector4d,
                            g2o::VertexSE3Expmap,
                            g2o::VertexSE3Expmap>()
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
     * @brief           Computes the 4D error: the marker position minus the
     *                  room centre (three components), plus the marker distance
     *                  from the origin.
     */
    void computeError() override
    {
        /* Marker's global pose */
        const g2o::VertexSE3Expmap *p_markerGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        /* Room's center point in global pose */
        const g2o::VertexSE3Expmap *p_roomCenterGpVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);

        /* Calculating poses (in global frame) */
        g2o::Isometry3D markerPose = p_markerGpVertex->estimate();
        Eigen::Vector3d roomPose =
            p_roomCenterGpVertex->estimate().translation();

        /* Calculating the error */
        _error[0] = markerPose.translation()(0) - roomPose(0);
        _error[1] = markerPose.translation()(1) - roomPose(1);
        _error[2] = markerPose.translation()(2) - roomPose(2);
        _error[3] = markerPose.translation().norm();
    }
};

} /* namespace EdgeVertexSE3RoomProjectSE3Marker */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXSE3ROOMPROJECTSE3MARKER_OBJECTS_EDGEVERTEXSE3ROOMPROJECTSE3MARKERCLASS_H \
        */
