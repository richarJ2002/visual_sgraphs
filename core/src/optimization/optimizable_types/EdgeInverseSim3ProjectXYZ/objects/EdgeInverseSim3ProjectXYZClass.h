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
 * @file            EdgeInverseSim3ProjectXYZClass.h
 *
 * @brief           Declares EdgeInverseSim3ProjectXYZ, a g2o edge/vertex type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEINVERSESIM3PROJECTXYZ_OBJECTS_EDGEINVERSESIM3PROJECTXYZCLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEINVERSESIM3PROJECTXYZ_OBJECTS_EDGEINVERSESIM3PROJECTXYZCLASS_H

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

#include "optimization/optimizable_types/VertexSim3Expmap/VertexSim3Expmap.h"

namespace vs_graphs
{
namespace core
{
namespace optimizable_types
{
namespace EdgeInverseSim3ProjectXYZ
{

/*!
 * @brief           Edge connecting a map point vertex to a similarity-transform
 *                  vertex: the pixel error of the point projected through the
 *                  second camera using the inverse transform.
 */
class EdgeInverseSim3ProjectXYZ
    : public g2o::BaseBinaryEdge<2,
                                 Eigen::Vector2d,
                                 g2o::VertexSBAPointXYZ,
                                 optimizable_types::VertexSim3Expmap::VertexSim3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ() :
        g2o::BaseBinaryEdge<2,
                            Eigen::Vector2d,
                            g2o::VertexSBAPointXYZ,
                            optimizable_types::VertexSim3Expmap::VertexSim3Expmap>()
    {}
    /*!
     * @brief           Reads this edge from a g2o graph file stream: the 2D
     *                  pixel measurement (u, v), then the information matrix.
     *
     * @param[in,out]   inputStream_inout
     *                  Stream positioned at this edge's data.
     *
     * @return          true always.
     */
    bool read(std::istream &inputStream_inout) override;
    /*!
     * @brief           Writes this edge to a g2o graph file stream: the pixel
     *                  measurement (u, v), then the information matrix.
     *
     * @param[in,out]   outputStream_inout
     *                  Stream the edge is appended to.
     *
     * @return          What writeInformationMatrix reports: true when the
     *                  stream is still good.
     */
    bool write(std::ostream &outputStream_inout) const override;

    /*!
     * @brief           Computes the reprojection error (u, v), in pixels: the
     *                  measured pixel minus the pixel the map point vertex
     *                  projects to after the inverse similarity transform,
     *                  through the second camera.
     */
    void computeError() override
    {
        const optimizable_types::VertexSim3Expmap::VertexSim3Expmap *v1 =
            static_cast<const optimizable_types::VertexSim3Expmap::VertexSim3Expmap *>(
                _vertices[1]);
        const g2o::VertexSBAPointXYZ *v2 =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

        Eigen::Vector2d observation(_measurement);
        _error =
            observation - v1->p_secondCamera->project(
                              (v1->estimate().inverse().map(v2->estimate())));
    }
};

} /* namespace EdgeInverseSim3ProjectXYZ */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGEINVERSESIM3PROJECTXYZ_OBJECTS_EDGEINVERSESIM3PROJECTXYZCLASS_H \
        */
