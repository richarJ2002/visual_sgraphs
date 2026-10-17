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
 * @file            EdgeVertexNPlaneProjectSE3RoomClass.h
 *
 * @brief           Declares EdgeVertexNPlaneProjectSE3Room, a g2o edge/vertex
 * type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXNPLANEPROJECTSE3ROOM_OBJECTS_EDGEVERTEXNPLANEPROJECTSE3ROOMCLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXNPLANEPROJECTSE3ROOM_OBJECTS_EDGEVERTEXNPLANEPROJECTSE3ROOMCLASS_H

#include <vs_graphs_g2o/core/base_multi_edge.h>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/types/isometry3d_mappings.h>
#include "optimizable_types/EdgeVertexNPlaneProjectSE3Room/objects/EdgeVertexNPlaneProjectSE3RoomStatusEnum.h"
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
namespace EdgeVertexNPlaneProjectSE3Room
{

/*!
 * The edge used to connect an N-wall Room's centroid (SE3) to its Wall vertices
 * (VertexPlane) [Note]: it creates constraint for three measurements, i.e., (x,
 * y, z)
 */
class EdgeVertexNPlaneProjectSE3Room
    : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
    EdgeVertexNPlaneProjectSE3Room()
    {
        /* Dynamically sized edge: at least one SE3 (room center) + N planes */
        resize(1);
    }

    /*!
     * @brief           Computes the 3D error between the room centre (first
     *                  vertex) and the mean of the positions implied by its
     *                  wall planes (remaining vertices).
     */
    void computeError() override
    {
        /* First vertex is always the room pose (SE3) */
        const g2o::VertexSE3Expmap *p_roomVertex =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d roomPose = p_roomVertex->estimate().translation();

        /* Remaining vertices are walls */
        std::vector<Eigen::Vector4d> walls;
        for (size_t vertexIndex = 1; vertexIndex < _vertices.size();
             ++vertexIndex)
        {
            const g2o::VertexPlane *p_wallVertex =
                static_cast<const g2o::VertexPlane *>(_vertices[vertexIndex]);
            Eigen::Vector4d plane = p_wallVertex->estimate().coeffs();
            if (correctPlaneDirection(plane) !=
                EdgeVertexNPlaneProjectSE3RoomStatus::
                    EDGE_VERTEX_NPLANE_PROJECT_SE3_ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: correctPlaneDirection returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            walls.push_back(plane);
        }

        /*!
         * Compute representative position from all walls enclosing the cluster
         */
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const Eigen::Vector4d &wall : walls)
        {
            /* Each wall equation ax+by+cz+d=0 → normal = (a,b,c), offset = d */
            Eigen::Vector3d normal = wall.head<3>();
            double          d      = wall(3);

            /* Approximate contribution: project along the normal */
            Eigen::Vector3d contrib = -d * normal;
            centroid += contrib;
        }

        if (!walls.empty())
            centroid /= static_cast<double>(walls.size());

        /* Final error */
        _error = roomPose - centroid;
    }

  protected:
    /*!
     * @brief           Flips the plane so its distance coefficient is not
     *                  positive.
     *
     * @param[in,out]   plane_inout
     *                  Plane coefficients (a, b, c, d); negated when d is
     *                  positive.
     *
     * @return          EDGE_VERTEX_NPLANE_PROJECT_SE3_ROOM_STATUS_SUCCESS
     *                  always.
     */
    [[nodiscard]] EdgeVertexNPlaneProjectSE3RoomStatus
        correctPlaneDirection(Eigen::Vector4d &plane_inout)
    {
        if (plane_inout(3) > 0)
            plane_inout *= -1;

        return EdgeVertexNPlaneProjectSE3RoomStatus::
            EDGE_VERTEX_NPLANE_PROJECT_SE3_ROOM_STATUS_SUCCESS;
    }
};

} /* namespace EdgeVertexNPlaneProjectSE3Room */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXNPLANEPROJECTSE3ROOM_OBJECTS_EDGEVERTEXNPLANEPROJECTSE3ROOMCLASS_H \
        */
