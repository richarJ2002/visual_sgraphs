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

#ifndef VS_GRAPHS_G2O_TYPES_VERTEX_POSE_4_DOF_H
#define VS_GRAPHS_G2O_TYPES_VERTEX_POSE_4_DOF_H

#include "optimization/g2o_types/imu_cam_pose/imu_cam_pose.h"
#include "optimization/g2o_types/g2o_types/public_functions/public_functions.h"
#include "optimization/g2o_types/unsupported_graph_io/unsupported_graph_io.h"
#include "optimization/g2o_types/vertex_acc_bias/vertex_acc_bias.h"
#include "optimization/g2o_types/vertex_gyro_bias/vertex_gyro_bias.h"
#include <Eigen/Dense>
#include <vs_graphs_g2o/core/base_binary_edge.h>
#include <vs_graphs_g2o/core/base_unary_edge.h>
#include <vs_graphs_g2o/core/base_vertex.h>
#include <vs_graphs_g2o/types/types_six_dof_expmap.h>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs::core::g2o_types::vertex_pose_4_dof
{

/*!
 * @brief           g2o vertex for pose-graph optimisation whose estimate is an
 *                  ImuCamPose; only the translation and the yaw (rotation about
 *                  the world z axis) are optimised, four degrees of freedom.
 */
class VertexPose4DoF
    : public unsupported_graph_io::UnsupportedGraphIo<
          g2o::BaseVertex<4, g2o_types::imu_cam_pose::ImuCamPose>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose4DoF() {}
    /*!
     * @brief           Creates the vertex with the pose of a key frame as its
     *                  estimate.
     *
     * @param[in]       p_keyFrame_inout
     *                  Key frame to read the pose from; dereferenced without a
     *                  null check, not changed and not kept.
     */
    VertexPose4DoF(keyframe::KeyFrame *p_keyFrame_inout)
    {
        setEstimate(g2o_types::imu_cam_pose::ImuCamPose(p_keyFrame_inout));
    }
    /*!
     * @brief           Creates the vertex with the pose of a frame as its
     *                  estimate.
     *
     * @param[in]       p_pF_inout
     *                  Frame to read the pose from; dereferenced without a null
     *                  check, not changed and not kept.
     */
    VertexPose4DoF(frame::Frame *p_pF_inout)
    {
        setEstimate(g2o_types::imu_cam_pose::ImuCamPose(p_pF_inout));
    }
    /*!
     * @brief           Creates the vertex with a single-camera pose built from
     *                  a camera pose in the world.
     *
     * @param[in]       cameraRotation_cameraToWorld_inout
     *                  Rotation of the camera in the world (camera to world);
     *                  only read.
     *
     * @param[in]       cameraTranslation_cameraToWorld_inout
     *                  Position of the camera origin in the world, metres; only
     *                  read.
     *
     * @param[in]       p_keyFrame_inout
     *                  Key frame supplying the extrinsics, camera model and
     *                  baseline-focal product; dereferenced without a null
     *                  check, not changed and not kept.
     */
    VertexPose4DoF(Eigen::Matrix3d    &cameraRotation_cameraToWorld_inout,
                   Eigen::Vector3d    &cameraTranslation_cameraToWorld_inout,
                   keyframe::KeyFrame *p_keyFrame_inout)
    {

        setEstimate(g2o_types::imu_cam_pose::ImuCamPose(
            cameraRotation_cameraToWorld_inout,
            cameraTranslation_cameraToWorld_inout,
            p_keyFrame_inout));
    }

    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    void setToOriginImpl() override;

    /*!
     * @brief           Applies an optimiser step to the pose in the world frame
     *                  (ImuCamPose::updateW) and refreshes the g2o cache. Roll
     *                  and pitch are left unchanged.
     *
     * @param[in]       p_update_in
     *                  Four values: yaw rotation about the world z axis
     *                  (radians), then translation x, y, z in the world frame
     *                  (metres).
     */
    void oplusImpl(const double *p_update_in) override;
};

} /* namespace vs_graphs::core::g2o_types::vertex_pose_4_dof */

#endif /* VS_GRAPHS_G2O_TYPES_VERTEX_POSE_4_DOF_H */
