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
 * @file            EdgeSE3ProjectXYZOnlyPoseClass.h
 *
 * @brief           Declares EdgeSE3ProjectXYZOnlyPose, a g2o edge/vertex type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3PROJECTXYZONLYPOSE_OBJECTS_EDGESE3PROJECTXYZONLYPOSECLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3PROJECTXYZONLYPOSE_OBJECTS_EDGESE3PROJECTXYZONLYPOSECLASS_H

#include <vs_graphs_g2o/core/base_multi_edge.h>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/types/isometry3d_mappings.h>
#include "sensor_models/camera_models/geometric_camera/geometric_camera.h"
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
namespace EdgeSE3ProjectXYZOnlyPose
{

/*!
 * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Camera vertex
 * (SE3) [Note]: it creates constraint for two measurements, i.e., (u, v)
 */
class EdgeSE3ProjectXYZOnlyPose
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates an empty edge.
     */
    EdgeSE3ProjectXYZOnlyPose() {}

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
     *                  measured pixel minus the pixel the map point Xw projects
     *                  to through the camera pose vertex.
     */
    void computeError() override
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d observation(_measurement);
        _error = observation - p_camera->project(v1->estimate().map(Xw));
    }

    /*!
     * @brief           Tells whether the observed map point lies in front of
     *                  the camera, i.e. its depth z in the camera frame is
     *                  positive.
     *
     * @return          true if the depth is greater than zero, false otherwise.
     */
    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return (v1->estimate().map(Xw))(2) > 0.0;
    }

    /*!
     * @brief           Computes the Jacobian of the reprojection error with
     *                  respect to the camera pose vertex.
     */
    void linearizeOplus() override;

    /*!
     * @brief           Position of the observed map point in the world frame,
     *                  metres.
     */
    Eigen::Vector3d                                   Xw;
    /*!
     * @brief           Camera model used to project points. Borrowed from the
     *                  key frame or frame that owns it, never null once set by
     *                  the code that builds the edge.
     */
    camera_models::geometric_camera::GeometricCamera *p_camera;
};

} /* namespace EdgeSE3ProjectXYZOnlyPose */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3PROJECTXYZONLYPOSE_OBJECTS_EDGESE3PROJECTXYZONLYPOSECLASS_H \
        */
