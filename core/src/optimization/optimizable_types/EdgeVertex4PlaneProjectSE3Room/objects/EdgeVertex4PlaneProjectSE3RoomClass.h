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
 * @file            EdgeVertex4PlaneProjectSE3RoomClass.h
 *
 * @brief           Declares EdgeVertex4PlaneProjectSE3Room, a g2o edge/vertex
 * type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEX4PLANEPROJECTSE3ROOM_OBJECTS_EDGEVERTEX4PLANEPROJECTSE3ROOMCLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEX4PLANEPROJECTSE3ROOM_OBJECTS_EDGEVERTEX4PLANEPROJECTSE3ROOMCLASS_H

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
namespace EdgeVertex4PlaneProjectSE3Room
{

/*!
 * The edge used to connect a Four-wall Room's centroid (SE3) to Wall vertices
 * (VertexPlane) [Note]: it creates constraint for three measurements, i.e., (x,
 * y, z) 🚧 [vS-Graphs v1.5] Deprecated with the introduction of n-wall rooms.
 */
class EdgeVertex4PlaneProjectSE3Room
    : public g2o::BaseMultiEdge<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates an empty edge.
     */
    EdgeVertex4PlaneProjectSE3Room() :
        g2o::BaseMultiEdge<3, Eigen::Vector3d>()
    {
        resize(5);
    }
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
     * @brief           Computes the 3D error between the room centre and the
     *                  position estimated from the four wall planes (two x
     *                  planes, two y planes).
     */
    void computeError() override
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::VertexPlane *v2 =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);
        const g2o::VertexPlane *v3 =
            static_cast<const g2o::VertexPlane *>(_vertices[2]);
        const g2o::VertexPlane *v4 =
            static_cast<const g2o::VertexPlane *>(_vertices[3]);
        const g2o::VertexPlane *v5 =
            static_cast<const g2o::VertexPlane *>(_vertices[4]);

        Eigen::Vector3d roomPose = v1->estimate().translation();
        Eigen::Vector4d xPlane1  = v2->estimate().coeffs();
        Eigen::Vector4d xPlane2  = v3->estimate().coeffs();
        Eigen::Vector4d yPlane1  = v4->estimate().coeffs();
        Eigen::Vector4d yPlane2  = v5->estimate().coeffs();

        correctPlaneDirection(xPlane1);
        correctPlaneDirection(xPlane2);
        correctPlaneDirection(yPlane1);
        correctPlaneDirection(yPlane2);

        Eigen::Vector3d vectorX, vectorY;
        if (fabs(xPlane1(3)) > fabs(xPlane2(3)))
            vectorX = (0.5 * (fabs(xPlane1(3)) * xPlane1.head(3) -
                              fabs(xPlane2(3)) * xPlane2.head(3))) +
                      fabs(xPlane2(3)) * xPlane2.head(3);
        else
            vectorX = (0.5 * (fabs(xPlane2(3)) * xPlane2.head(3) -
                              fabs(xPlane1(3)) * xPlane1.head(3))) +
                      fabs(xPlane1(3)) * xPlane1.head(3);

        if (fabs(yPlane1(3)) > fabs(yPlane2(3)))
            vectorY = (0.5 * (fabs(yPlane1(3)) * yPlane1.head(3) -
                              fabs(yPlane2(3)) * yPlane2.head(3))) +
                      fabs(yPlane2(3)) * yPlane2.head(3);
        else
            vectorY = (0.5 * (fabs(yPlane2(3)) * yPlane2.head(3) -
                              fabs(yPlane1(3)) * yPlane1.head(3))) +
                      fabs(yPlane1(3)) * yPlane1.head(3);

        Eigen::Vector3d finalPose = vectorX + vectorY;
        _error                    = roomPose - finalPose;
    }

  protected:
    /*!
     * @brief           Flips the plane so its distance coefficient is not
     *                  positive.
     *
     * @param[in,out]   plane_inout
     *                  Plane coefficients (a, b, c, d); negated when d is
     *                  positive.
     */
    virtual void correctPlaneDirection(Eigen::Vector4d &plane_inout)
    {
        if (plane_inout(3) > 0)
            plane_inout *= -1;
    }
};

} /* namespace EdgeVertex4PlaneProjectSE3Room */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEX4PLANEPROJECTSE3ROOM_OBJECTS_EDGEVERTEX4PLANEPROJECTSE3ROOMCLASS_H \
        */
