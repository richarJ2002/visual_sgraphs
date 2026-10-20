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
 * @file            EdgeSE3ProjectXYZOnlyPoseToBodyClass.h
 *
 * @brief           Declares EdgeSE3ProjectXYZOnlyPoseToBody, a g2o edge/vertex
 * type.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3PROJECTXYZONLYPOSETOBODY_OBJECTS_EDGESE3PROJECTXYZONLYPOSETOBODYCLASS_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3PROJECTXYZONLYPOSETOBODY_OBJECTS_EDGESE3PROJECTXYZONLYPOSETOBODYCLASS_H

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
namespace EdgeSE3ProjectXYZOnlyPoseToBody
{

/*!
 * The edge used to connect a MapPoint vertex (SBAPointXYZ) to a Camera vertex
 * (SE3) [Note]: it creates constraint for two measurements, i.e., (u, v)
 */
class EdgeSE3ProjectXYZOnlyPoseToBody
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates an empty edge.
     */
    EdgeSE3ProjectXYZOnlyPoseToBody() {}

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
     * @brief           Computes the reprojection error (u, v), in pixels, for
     *                  the right camera of a stereo pair: the measured pixel
     *                  minus the pixel the map point Xw projects to after the
     *                  pose vertex and the left-to-right camera transform mTrl.
     */
    void computeError() override
    {
        const g2o::VertexSE3Expmap *v1 =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d observation(_measurement);
        _error =
            observation - p_camera->project((mTrl * v1->estimate()).map(Xw));
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
        return ((mTrl * v1->estimate()).map(Xw))(2) > 0.0;
    }

    /*!
     * @brief           Computes the Jacobian of the reprojection error with
     *                  respect to the camera pose vertex, through the left-to-
     *                  right camera transform mTrl.
     */
    void linearizeOplus() override;

    /*!
     * @brief           Position of the observed map point in the world frame,
     *                  metres.
     */
    Eigen::Vector3d                                   Xw;
    /*!
     * @brief           Camera model of the right camera used to project points.
     *                  Borrowed from the frame or key frame that owns it, never
     *                  null once set by the code that builds the edge.
     */
    camera_models::geometric_camera::GeometricCamera *p_camera;

    /*!
     * @brief           Rigid transform from the left (pose vertex) camera frame
     *                  to the right camera frame, set by the code that builds
     *                  the edge from the stereo extrinsics.
     */
    g2o::SE3Quat mTrl;
};

} /* namespace EdgeSE3ProjectXYZOnlyPoseToBody */
} /* namespace optimizable_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZABLE_TYPES_EDGESE3PROJECTXYZONLYPOSETOBODY_OBJECTS_EDGESE3PROJECTXYZONLYPOSETOBODYCLASS_H \
        */
