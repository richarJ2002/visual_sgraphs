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
 * @file            EdgeVertexPlaneProjectPointXYZClass.h
 *
 * @brief           Declares EdgeVertexPlaneProjectPointXYZ, a g2o edge/vertex
 * type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXPLANEPROJECTPOINTXYZ_OBJECTS_EDGEVERTEXPLANEPROJECTPOINTXYZCLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXPLANEPROJECTPOINTXYZ_OBJECTS_EDGEVERTEXPLANEPROJECTPOINTXYZCLASS_H

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
namespace EdgeVertexPlaneProjectPointXYZ
{

/*!
 * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Plane vertex
 * (VertexPlane)
 */
class EdgeVertexPlaneProjectPointXYZ
    : public g2o::
          BaseBinaryEdge<1, double, g2o::VertexSBAPointXYZ, g2o::VertexPlane>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates an empty edge.
     */
    EdgeVertexPlaneProjectPointXYZ() :
        g2o::BaseBinaryEdge<1,
                            double,
                            g2o::VertexSBAPointXYZ,
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
     * @brief           Computes the signed distance, in metres, from the map
     *                  point to the plane (normalised plane equation n.x + d).
     */
    void computeError() override
    {
        /* Map Point's position */
        const g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        /* Plane's global pose */
        const g2o::VertexPlane *p_planeGpVertex =
            static_cast<const g2o::VertexPlane *>(_vertices[1]);

        /*!
         * Calculating the error
         * plane equation is already normalized -> D = n.x + d
         */
        _error[0] = p_planeGpVertex->estimate().coeffs().head(3).dot(
                        p_pointVertex->estimate().head(3)) +
                    p_planeGpVertex->estimate().coeffs()(3);
    }
};

} /* namespace EdgeVertexPlaneProjectPointXYZ */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEVERTEXPLANEPROJECTPOINTXYZ_OBJECTS_EDGEVERTEXPLANEPROJECTPOINTXYZCLASS_H \
        */
