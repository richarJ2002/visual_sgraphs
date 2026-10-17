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
 * @file            EdgeStereoClass.h
 *
 * @brief           Declares the EdgeStereo
 *                  Binary edge between a map point and a pose vertex holding
 *                  the stereo reprojection error for one camera of the rig.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_STEREO_OBJECTS_EDGE_STEREO_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_STEREO_OBJECTS_EDGE_STEREO_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vs_graphs_g2o/core/base_binary_edge.h>
#include <vs_graphs_g2o/types/types_sba.h>

#include "g2o_types/unsupported_graph_io/unsupported_graph_io.h"
#include "g2o_types/vertex_pose/vertex_pose.h"

#include "g2o_types/edge_stereo/objects/EdgeStereoStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace edge_stereo
{

/*!
 * @brief           Stereo reprojection edge between a point and pose.
 */
class EdgeStereo : public unsupported_graph_io::UnsupportedGraphIo<
                       g2o::BaseBinaryEdge<3,
                                           Eigen::Vector3d,
                                           g2o::VertexSBAPointXYZ,
                                           vertex_pose::VertexPose>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates the edge for one camera of the rig.
     *
     * @param[in]       cameraIndex_in
     *                  Camera that made the observation, 0 for the left and 1
     *                  for the right.
     */
    EdgeStereo(int cameraIndex_in = 0) :
        cam_idx(cameraIndex_in)
    {}
    /*!
     * @brief           Sets the error to the observed (u, v, u_right) minus the
     *                  stereo projection of the map point in camera cam_idx.
     *                  Failed projections are only logged.
     */
    void computeError() override;
    /*!
     * @brief           Fills the Jacobians of the error with respect to the map
     *                  point (3 columns, _jacobianOplusXi) and the pose (6
     *                  columns, _jacobianOplusXj, rotation then translation in
     *                  the body frame, as for
     *                  imu_cam_pose::ImuCamPose::update).
     */
    void linearizeOplus() override;
    /*!
     * @brief           Returns the 3 by 9 Jacobian of the error, columns
     *                  ordered map point (3) then pose (6). Relinearises at the
     *                  current estimates first.
     *
     * @param[out]      jacobian_out
     *                  Jacobian of the error.
     *
     * @return          EDGE_STEREO_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeStereoStatus
        getJacobian(Eigen::Matrix<double, 3, 9> &jacobian_out);
    /*!
     * @brief           Returns the 9 by 9 Gauss-Newton Hessian block, ordered
     *                  map point (3) then pose (6). Relinearises at the current
     *                  estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_STEREO_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeStereoStatus
              getHessian(Eigen::Matrix<double, 9, 9> &hessian_out);
    /*!
     * @brief           Camera of the rig that made the observation, 0 for the
     *                  left and 1 for the right.
     */
    const int cam_idx;
};

} /* namespace edge_stereo */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_STEREO_OBJECTS_EDGE_STEREO_CLASS_H \
        */
