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
 * @file            G2oTypes.h
 *
 * @brief           Declares the g2o vertex and edge types for visual-inertial
 *                  optimisation: IMU-camera poses, velocities, biases and IMU
 *                  preintegration constraints.
 */

#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "EdgeAccRWStatus.h"
#include "EdgeGyroRWStatus.h"
#include "EdgeInertialGSStatus.h"
#include "EdgeInertialStatus.h"
#include "EdgeMonoOnlyPoseStatus.h"
#include "EdgeMonoStatus.h"
#include "EdgePriorAccStatus.h"
#include "EdgePriorGyroStatus.h"
#include "EdgePriorPoseImuStatus.h"
#include "EdgeStereoOnlyPoseStatus.h"
#include "EdgeStereoStatus.h"
#include "G2oTypesStatus.h"
#include "GDirectionStatus.h"
#include "ImuCamPoseStatus.h"
#include "InvDepthPointStatus.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"

#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <Frame.h>
#include <KeyFrame.h>

#include "Utils/Converter/objects/Converter.h"
#include <math.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

class KeyFrame;
class Frame;
namespace camera_models
{
namespace geometriccamera
{
class GeometricCamera;
} // namespace geometriccamera
} // namespace camera_models

/*!
 * @brief           Column vector of six doubles.
 */
typedef Eigen::Matrix<double, 6, 1>   Vector6d;
/*!
 * @brief           Column vector of nine doubles.
 */
typedef Eigen::Matrix<double, 9, 1>   Vector9d;
/*!
 * @brief           Column vector of twelve doubles.
 */
typedef Eigen::Matrix<double, 12, 1>  Vector12d;
/*!
 * @brief           Column vector of fifteen doubles, the size of the IMU prior
 *                  residual.
 */
typedef Eigen::Matrix<double, 15, 1>  Vector15d;
/*!
 * @brief           Square matrix of twelve by twelve doubles.
 */
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
/*!
 * @brief           Square matrix of fifteen by fifteen doubles, the size of the
 *                  IMU prior information matrix.
 */
typedef Eigen::Matrix<double, 15, 15> Matrix15d;
/*!
 * @brief           Square matrix of nine by nine doubles, the size of the IMU
 *                  preintegration information matrix.
 */
typedef Eigen::Matrix<double, 9, 9>   Matrix9d;

/*!
 * @brief           Converts a rotation vector given as three components into
 *                  the rotation matrix it describes (exponential map of SO(3)).
 *                  Angles below 1e-5 rad use a second-order approximation; the
 *                  result is re-orthogonalised.
 *
 * @param[in]       angleAxisX_in
 *                  X component of the rotation vector (axis times angle),
 *                  radians.
 *
 * @param[in]       angleAxisY_in
 *                  Y component of the rotation vector, radians.
 *
 * @param[in]       angleAxisZ_in
 *                  Z component of the rotation vector, radians.
 *
 * @param[out]      rotation_out
 *                  Rotation matrix for that rotation vector.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus expSO3(const double     angleAxisX_in,
                                    const double     angleAxisY_in,
                                    const double     angleAxisZ_in,
                                    Eigen::Matrix3d &rotation_out);
/*!
 * @brief           Converts a rotation vector into the rotation matrix it
 *                  describes (exponential map of SO(3)); same as the overload
 *                  taking three components.
 *
 * @param[in]       rotationVector_in
 *                  Rotation vector (axis times angle), radians.
 *
 * @param[out]      rotation_out
 *                  Rotation matrix for that rotation vector.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus expSO3(const Eigen::Vector3d &rotationVector_in,
                                    Eigen::Matrix3d       &rotation_out);

/*!
 * @brief           Converts a rotation matrix into its rotation vector
 *                  (logarithmic map of SO(3)). When the matrix is not a valid
 *                  rotation or its angle is within 1e-5 of 0 or pi, the vector
 *                  part of the matrix is returned without scaling by the angle.
 *
 * @param[in]       rotationMatrix_in
 *                  Rotation matrix to convert.
 *
 * @param[out]      rotationVector_out
 *                  Rotation vector (axis times angle), radians.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus logSO3(const Eigen::Matrix3d &rotationMatrix_in,
                                    Eigen::Vector3d       &rotationVector_out);

/*!
 * @brief           Computes the inverse of the right Jacobian of SO(3) at a
 *                  rotation vector; same as the overload taking three
 *                  components.
 *
 * @param[in]       rotationVector_in
 *                  Rotation vector the Jacobian is evaluated at, radians.
 *
 * @param[out]      inverseRightJacobian_out
 *                  Inverse right Jacobian, 3 by 3.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus
    inverseRightJacobianSO3(const Eigen::Vector3d &rotationVector_in,
                            Eigen::Matrix3d       &inverseRightJacobian_out);
/*!
 * @brief           Computes the right Jacobian of SO(3) at a rotation vector;
 *                  same as the overload taking three components.
 *
 * @param[in]       rotationVector_in
 *                  Rotation vector the Jacobian is evaluated at, radians.
 *
 * @param[out]      rightJacobian_out
 *                  Right Jacobian, 3 by 3.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus
    rightJacobianSO3(const Eigen::Vector3d &rotationVector_in,
                     Eigen::Matrix3d       &rightJacobian_out);
/*!
 * @brief           Computes the right Jacobian of SO(3), which relates a small
 *                  change of a rotation vector to the change of the rotation on
 *                  its right side. It is the identity for angles below 1e-5
 *                  rad.
 *
 * @param[in]       angleAxisX_in
 *                  X component of the rotation vector (axis times angle),
 *                  radians.
 *
 * @param[in]       angleAxisY_in
 *                  Y component of the rotation vector, radians.
 *
 * @param[in]       angleAxisZ_in
 *                  Z component of the rotation vector, radians.
 *
 * @param[out]      rightJacobian_out
 *                  Right Jacobian, 3 by 3.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus
    rightJacobianSO3(const double     angleAxisX_in,
                     const double     angleAxisY_in,
                     const double     angleAxisZ_in,
                     Eigen::Matrix3d &rightJacobian_out);

/*!
 * @brief           Builds the skew-symmetric matrix of a 3D vector, so that the
 *                  matrix times another vector equals the cross product of the
 *                  two.
 *
 * @param[in]       angularVelocity_in
 *                  Vector to convert; the caller passes an angular velocity or
 *                  a rotation vector.
 *
 * @param[out]      skewMatrix_out
 *                  Skew-symmetric matrix of that vector.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus
    computeSkewMatrix(const Eigen::Vector3d &angularVelocity_in,
                      Eigen::Matrix3d       &skewMatrix_out);
/*!
 * @brief           Computes the inverse of the right Jacobian of SO(3) at a
 *                  rotation vector given as three components. It is the
 *                  identity for angles below 1e-5 rad.
 *
 * @param[in]       angleAxisX_in
 *                  X component of the rotation vector (axis times angle),
 *                  radians.
 *
 * @param[in]       angleAxisY_in
 *                  Y component of the rotation vector, radians.
 *
 * @param[in]       angleAxisZ_in
 *                  Z component of the rotation vector, radians.
 *
 * @param[out]      inverseRightJacobian_out
 *                  Inverse right Jacobian, 3 by 3.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
[[nodiscard]] G2oTypesStatus
    inverseRightJacobianSO3(const double     angleAxisX_in,
                            const double     angleAxisY_in,
                            const double     angleAxisZ_in,
                            Eigen::Matrix3d &inverseRightJacobian_out);

/*!
 * @brief           Replaces a nearly orthogonal matrix by the closest
 *                  orthogonal matrix, which is the product of the two
 *                  orthogonal factors of its singular value decomposition. Used
 *                  to remove numerical drift from rotation matrices.
 *
 * @param[in]       R
 *                  Matrix to orthogonalise, expected to be close to a rotation.
 *
 * @param[out]      rotation_out
 *                  Closest orthogonal matrix to R.
 *
 * @return          G2O_TYPES_STATUS_SUCCESS always.
 */
template <typename T = double>
[[nodiscard]] G2oTypesStatus
    normalizeRotation(const Eigen::Matrix<T, 3, 3> &R,
                      Eigen::Matrix<T, 3, 3>       &rotation_out)
{
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R,
                                                 Eigen::ComputeFullU |
                                                     Eigen::ComputeFullV);
    rotation_out = svd.matrixU() * svd.matrixV().transpose();
    return G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS;
}

/*!
 * @brief           Pose of one frame in the visual-inertial optimisation. The
 *                  optimised quantity is the IMU (body) pose in the world; the
 *                  pose of every camera is derived from it and recomputed after
 *                  each update. Holds borrowed pointers to the camera models
 *                  and owns no heap object.
 */
class ImuCamPose
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuCamPose() {}
    /*!
     * @brief           Builds the pose from a key frame: the IMU pose, the pose
     *                  of the left camera (and of the right camera when the key
     *                  frame has a second camera), the camera-body extrinsics
     *                  and the baseline-focal product. The 4DoF reference
     *                  rotation starts as the IMU rotation. Failed reads are
     *                  only logged.
     *
     * @param[in]       p_keyFrame_inout
     *                  Key frame to read from; dereferenced without a null
     *                  check, not changed and not kept. Its camera model
     *                  pointers are kept, not owned.
     */
    ImuCamPose(KeyFrame *p_keyFrame_inout) :
        its(0)
    {
        // Load IMU pose
        Eigen::Vector3f keyFrameImuPosition{};
        if (p_keyFrame_inout->getImuPosition(keyFrameImuPosition) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        twb = keyFrameImuPosition.cast<double>();
        Eigen::Matrix3f keyFrameImuRotation{};
        if (p_keyFrame_inout->getImuRotation(keyFrameImuRotation) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Rwb = keyFrameImuRotation.cast<double>();

        // Load camera poses
        int camCount;
        if (p_keyFrame_inout->p_camera2)
            camCount = 2;
        else
            camCount = 1;

        tcw.resize(camCount);
        Rcw.resize(camCount);
        tcb.resize(camCount);
        Rcb.resize(camCount);
        Rbc.resize(camCount);
        tbc.resize(camCount);
        pCamera.resize(camCount);

        // Left camera
        Eigen::Vector3f keyFrameTranslation{};
        if (p_keyFrame_inout->getTranslation(keyFrameTranslation) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTranslation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        tcw[0] = keyFrameTranslation.cast<double>();
        Eigen::Matrix3f keyFrameRotation{};
        if (p_keyFrame_inout->getRotation(keyFrameRotation) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRotation returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Rcw[0] = keyFrameRotation.cast<double>();
        tcb[0] =
            p_keyFrame_inout->imuCalibration.mTcb.translation().cast<double>();
        Rcb[0] = p_keyFrame_inout->imuCalibration.mTcb.rotationMatrix()
                     .cast<double>();
        Rbc[0] = Rcb[0].transpose();
        tbc[0] =
            p_keyFrame_inout->imuCalibration.mTbc.translation().cast<double>();
        pCamera[0] = p_keyFrame_inout->p_camera;
        bf         = p_keyFrame_inout->mbf;

        if (camCount > 1)
        {
            Sophus::SE3f keyFrameRelativePoseTrl{};
            if (p_keyFrame_inout->getRelativePoseTrl(keyFrameRelativePoseTrl) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRelativePoseTrl returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Matrix4d stereoPose_leftCameraToRightCamera =
                keyFrameRelativePoseTrl.matrix().cast<double>();
            Rcw[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * Rcw[0];
            tcw[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * tcw[0] +
                stereoPose_leftCameraToRightCamera.block<3, 1>(0, 3);
            tcb[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * tcb[0] +
                stereoPose_leftCameraToRightCamera.block<3, 1>(0, 3);
            Rcb[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * Rcb[0];
            Rbc[1]     = Rcb[1].transpose();
            tbc[1]     = -Rbc[1] * tcb[1];
            pCamera[1] = p_keyFrame_inout->p_camera2;
        }

        // For posegraph 4DoF
        Rwb0 = Rwb;
        DR.setIdentity();
    }
    /*!
     * @brief           Builds the pose from a frame: the IMU pose, the pose of
     *                  the left camera (and of the right camera when the frame
     *                  has a second camera), the camera-body extrinsics and the
     *                  baseline-focal product. The 4DoF reference rotation
     *                  starts as the IMU rotation. Failed reads are only
     *                  logged.
     *
     * @param[in]       p_pF_inout
     *                  Frame to read from; dereferenced without a null check,
     *                  not changed and not kept. Its camera model pointers are
     *                  kept, not owned.
     */
    ImuCamPose(Frame *p_pF_inout) :
        its(0)
    {
        // Load IMU pose
        Eigen::Matrix<float, 3, 1> pFGetImuPosition{};
        if (p_pF_inout->getImuPosition(pFGetImuPosition) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        twb = pFGetImuPosition.cast<double>();
        Eigen::Matrix<float, 3, 3> pFImuRotation{};
        if (p_pF_inout->getImuRotation(pFImuRotation) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Rwb = pFImuRotation.cast<double>();

        // Load camera poses
        int camCount;
        if (p_pF_inout->p_camera2)
            camCount = 2;
        else
            camCount = 1;

        tcw.resize(camCount);
        Rcw.resize(camCount);
        tcb.resize(camCount);
        Rcb.resize(camCount);
        Rbc.resize(camCount);
        tbc.resize(camCount);
        pCamera.resize(camCount);

        // Left camera
        Sophus::SE3<float> pFGetPose{};
        if (p_pF_inout->getPose(pFGetPose) != FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        tcw[0] = pFGetPose.translation().cast<double>();
        Sophus::SE3<float> pFGetPose2{};
        if (p_pF_inout->getPose(pFGetPose2) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Rcw[0] = pFGetPose2.rotationMatrix().cast<double>();
        tcb[0] = p_pF_inout->imuCalibration.mTcb.translation().cast<double>();
        Rcb[0] =
            p_pF_inout->imuCalibration.mTcb.rotationMatrix().cast<double>();
        Rbc[0] = Rcb[0].transpose();
        tbc[0] = p_pF_inout->imuCalibration.mTbc.translation().cast<double>();
        pCamera[0] = p_pF_inout->p_camera;
        bf         = p_pF_inout->mbf;

        if (camCount > 1)
        {
            Sophus::SE3f pFRelativePoseTrl{};
            if (p_pF_inout->getRelativePoseTrl(pFRelativePoseTrl) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRelativePoseTrl returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Matrix4d stereoPose_leftCameraToRightCamera =
                pFRelativePoseTrl.matrix().cast<double>();
            Rcw[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * Rcw[0];
            tcw[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * tcw[0] +
                stereoPose_leftCameraToRightCamera.block<3, 1>(0, 3);
            tcb[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * tcb[0] +
                stereoPose_leftCameraToRightCamera.block<3, 1>(0, 3);
            Rcb[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * Rcb[0];
            Rbc[1]     = Rcb[1].transpose();
            tbc[1]     = -Rbc[1] * tcb[1];
            pCamera[1] = p_pF_inout->p_camera2;
        }

        // For posegraph 4DoF
        Rwb0 = Rwb;
        DR.setIdentity();
    }
    /*!
     * @brief           Builds a single-camera pose for pose-graph optimisation
     *                  from a camera pose in the world; the IMU pose follows
     *                  from the camera-body extrinsics of the key frame. Stereo
     *                  is ignored.
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
    ImuCamPose(Eigen::Matrix3d &cameraRotation_cameraToWorld_inout,
               Eigen::Vector3d &cameraTranslation_cameraToWorld_inout,
               KeyFrame        *p_keyFrame_inout) :
        its(0)
    {
        // This is only for posegrpah, we do not care about multicamera
        tcw.resize(1);
        Rcw.resize(1);
        tcb.resize(1);
        Rcb.resize(1);
        Rbc.resize(1);
        tbc.resize(1);
        pCamera.resize(1);

        tcb[0] =
            p_keyFrame_inout->imuCalibration.mTcb.translation().cast<double>();
        Rcb[0] = p_keyFrame_inout->imuCalibration.mTcb.rotationMatrix()
                     .cast<double>();
        Rbc[0] = Rcb[0].transpose();
        tbc[0] =
            p_keyFrame_inout->imuCalibration.mTbc.translation().cast<double>();
        twb = cameraRotation_cameraToWorld_inout * tcb[0] +
              cameraTranslation_cameraToWorld_inout;
        Rwb        = cameraRotation_cameraToWorld_inout * Rcb[0];
        Rcw[0]     = cameraRotation_cameraToWorld_inout.transpose();
        tcw[0]     = -Rcw[0] * cameraTranslation_cameraToWorld_inout;
        pCamera[0] = p_keyFrame_inout->p_camera;
        bf         = p_keyFrame_inout->mbf;

        // For posegraph 4DoF
        Rwb0 = Rwb;
        DR.setIdentity();
    }

    /*!
     * @brief           Replaces the camera poses, the extrinsics and the
     *                  baseline-focal product, and recomputes the IMU pose from
     *                  camera 0. The vectors hold one entry per camera; at
     *                  least one entry is required and the sizes are not
     *                  checked.
     *
     * @param[in]       cameraRotations_worldToCamera_in
     *                  Rotation from the world to each camera.
     *
     * @param[in]       cameraTranslations_worldToCamera_in
     *                  Translation from the world to each camera, metres.
     *
     * @param[in]       extrinsicRotations_cameraToBody_in
     *                  Rotation from each camera to the body frame.
     *
     * @param[in]       extrinsicTranslations_cameraToBody_in
     *                  Position of each camera origin in the body frame,
     *                  metres.
     *
     * @param[in]       baselineFocalProduct_in
     *                  Stereo baseline times focal length, metres times pixels.
     *
     * @return          IMU_CAM_POSE_STATUS_SUCCESS always.
     */
    [[nodiscard]] ImuCamPoseStatus setParam(
        const std::vector<Eigen::Matrix3d> &cameraRotations_worldToCamera_in,
        const std::vector<Eigen::Vector3d> &cameraTranslations_worldToCamera_in,
        const std::vector<Eigen::Matrix3d> &extrinsicRotations_cameraToBody_in,
        const std::vector<Eigen::Vector3d>
                     &extrinsicTranslations_cameraToBody_in,
        const double &baselineFocalProduct_in);

    /*!
     * @brief           Applies an optimiser step to the IMU pose in the body
     *                  frame, then recomputes every camera pose. The
     *                  translation step is rotated into the world by the body
     *                  rotation before it is added.
     *
     * @param[in]       p_updateVector_in
     *                  Six values: rotation vector (radians) then translation
     *                  (metres), both in the body frame.
     *
     * @return          IMU_CAM_POSE_STATUS_SUCCESS always.
     */
    [[nodiscard]] ImuCamPoseStatus update(const double *p_updateVector_in);
    /*!
     * @brief           Applies an optimiser step to the IMU pose in the world
     *                  frame, then recomputes every camera pose. The rotation
     *                  step accumulates in DR and acts on the reference
     *                  rotation Rwb0.
     *
     * @param[in]       p_updateVector_in
     *                  Six values: rotation vector (radians) then translation
     *                  (metres), both in the world frame.
     *
     * @return          IMU_CAM_POSE_STATUS_SUCCESS always.
     */
    [[nodiscard]] ImuCamPoseStatus updateW(const double *p_updateVector_in);
    /*!
     * @brief           Projects a world point into the image of one camera
     *                  (monocular observation).
     *
     * @param[in]       Xw_in
     *                  Point in the world frame, metres.
     *
     * @param[out]      projection_out
     *                  Pixel coordinates (u, v) of the point.
     *
     * @param[in]       cameraIndex_in
     *                  Camera to project into, 0 for the left and 1 for the
     *                  right; not range checked.
     *
     * @return          IMU_CAM_POSE_STATUS_SUCCESS always.
     */
    [[nodiscard]] ImuCamPoseStatus project(const Eigen::Vector3d &Xw_in,
                                           Eigen::Vector2d &projection_out,
                                           int cameraIndex_in = 0) const;
    /*!
     * @brief           Projects a world point into the image of one camera and
     *                  adds the right-image column that a rectified stereo pair
     *                  would see (stereo observation). The depth is not checked
     *                  to be positive.
     *
     * @param[in]       Xw_in
     *                  Point in the world frame, metres.
     *
     * @param[out]      stereo_out
     *                  Pixel coordinates (u, v, u_right), where u_right is u
     *                  minus the baseline-focal product divided by the depth.
     *
     * @param[in]       cameraIndex_in
     *                  Camera to project into, 0 for the left and 1 for the
     *                  right; not range checked.
     *
     * @return          IMU_CAM_POSE_STATUS_SUCCESS always.
     */
    [[nodiscard]] ImuCamPoseStatus projectStereo(const Eigen::Vector3d &Xw_in,
                                                 Eigen::Vector3d &stereo_out,
                                                 int cameraIndex_in = 0) const;
    /*!
     * @brief           Tells whether a world point lies in front of a camera.
     *
     * @param[in]       Xw_in
     *                  Point in the world frame, metres.
     *
     * @param[in]       cameraIndex_in
     *                  Camera to test, 0 for the left and 1 for the right; not
     *                  range checked.
     *
     * @return          true when the depth of the point in that camera frame is
     *                  positive.
     */
    bool                           isDepthPositive(const Eigen::Vector3d &Xw_in,
                                                   int                    cameraIndex_in = 0) const;

  public:
    // For IMU
    /*!
     * @brief           Rotation of the IMU (body) frame in the world frame;
     *                  maps body vectors to world vectors.
     */
    Eigen::Matrix3d Rwb;
    /*!
     * @brief           Position of the IMU (body) origin in the world frame,
     *                  metres.
     */
    Eigen::Vector3d twb;

    // For set of cameras
    /*!
     * @brief           Rotation from the world frame to each camera frame,
     *                  indexed by camera (0 left, 1 right).
     */
    std::vector<Eigen::Matrix3d>                                   Rcw;
    /*!
     * @brief           Translation from the world frame to each camera frame,
     *                  metres, indexed like Rcw.
     */
    std::vector<Eigen::Vector3d>                                   tcw;
    /*!
     * @brief           Rotation from the body frame to each camera frame,
     *                  indexed like Rcw.
     */
    std::vector<Eigen::Matrix3d>                                   Rcb;
    /*!
     * @brief           Rotation from each camera frame to the body frame,
     *                  indexed like Rcw.
     */
    std::vector<Eigen::Matrix3d>                                   Rbc;
    /*!
     * @brief           Position of the body origin in each camera frame,
     *                  metres, indexed like Rcw.
     */
    std::vector<Eigen::Vector3d>                                   tcb;
    /*!
     * @brief           Position of each camera origin in the body frame,
     *                  metres, indexed like Rcw.
     */
    std::vector<Eigen::Vector3d>                                   tbc;
    /*!
     * @brief           Stereo baseline times focal length, metres times pixels,
     *                  of the key frame or frame this pose was built from.
     */
    double                                                         bf;
    /*!
     * @brief           Camera model of each camera, indexed like Rcw. Borrowed
     *                  from the key frame or frame; never deleted here.
     */
    std::vector<camera_models::geometriccamera::GeometricCamera *> pCamera;

    // For posegraph 4DoF
    /*!
     * @brief           Body rotation when the pose was built; the fixed
     *                  reference of the 4DoF update, Rwb = DR * Rwb0.
     */
    Eigen::Matrix3d Rwb0;
    /*!
     * @brief           World-frame rotation accumulated by updateW on top of
     *                  Rwb0; identity until the first such update.
     */
    Eigen::Matrix3d DR;

    /*!
     * @brief           Number of update steps counted towards the periodic
     *                  rotation clean-up; reset to zero when the clean-up runs.
     */
    int its;
};

/*!
 * @brief           Map point described by its inverse depth along the viewing
 *                  ray of a pixel of a host key frame. Only the inverse depth
 *                  is optimised; the pixel and the host camera intrinsics are
 *                  copied at construction.
 */
class InvDepthPoint
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    InvDepthPoint() {}
    /*!
     * @brief           Creates the point from an initial inverse depth and the
     *                  host pixel it was observed at.
     *
     * @param[in]       rho_in
     *                  Initial inverse depth in the host key frame, one over
     *                  metres.
     *
     * @param[in]       u_in
     *                  Pixel column of the observation in the host key frame.
     *
     * @param[in]       v_in
     *                  Pixel row of the observation in the host key frame.
     *
     * @param[in]       p_hostKeyFrame_inout
     *                  Host key frame whose intrinsics and baseline-focal
     *                  product are copied; dereferenced without a null check,
     *                  not changed and not kept.
     */
    InvDepthPoint(double    rho_in,
                  double    u_in,
                  double    v_in,
                  KeyFrame *p_hostKeyFrame_inout) :
        rho(rho_in),
        u(u_in),
        v(v_in),
        fx(p_hostKeyFrame_inout->fx),
        fy(p_hostKeyFrame_inout->fy),
        cx(p_hostKeyFrame_inout->cx),
        cy(p_hostKeyFrame_inout->cy),
        bf(p_hostKeyFrame_inout->mbf)
    {}

    /*!
     * @brief           Adds an optimiser step to the inverse depth.
     *
     * @param[in]       p_inverseDepthDelta_in
     *                  Pointer to one value, the change of the inverse depth in
     *                  one over metres.
     *
     * @return          INV_DEPTH_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] InvDepthPointStatus
        update(const double *p_inverseDepthDelta_in);

    /*!
     * @brief           Inverse depth of the point in the host key frame, one
     *                  over metres; the optimised value.
     */
    double rho;
    /*!
     * @brief           Pixel column of the observation in the host key frame;
     *                  fixed, not optimised.
     */
    double u;
    /*!
     * @brief           Pixel row of the observation in the host key frame;
     *                  fixed, not optimised.
     */
    double v;

    /*!
     * @brief           Focal length of the host key frame along x, pixels.
     */
    double fx;
    /*!
     * @brief           Focal length of the host key frame along y, pixels.
     */
    double fy;
    /*!
     * @brief           Principal point x of the host key frame, pixels.
     */
    double cx;
    /*!
     * @brief           Principal point y of the host key frame, pixels.
     */
    double cy;
    /*!
     * @brief           Stereo baseline times focal length of the host key
     *                  frame, metres times pixels.
     */
    double bf;

    /*!
     * @brief           Update counter; the constructor does not set it and
     *                  update does not use it.
     */
    int its;
};

/*!
 * @brief           g2o vertex whose estimate is an ImuCamPose, so the optimised
 *                  quantity is the IMU (body) pose with six degrees of freedom.
 */
class VertexPose : public g2o::BaseVertex<6, ImuCamPose>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose() {}
    /*!
     * @brief           Creates the vertex with the pose of a key frame as its
     *                  estimate.
     *
     * @param[in]       p_keyFrame_inout
     *                  Key frame to read the pose from; dereferenced without a
     *                  null check, not changed and not kept.
     */
    VertexPose(KeyFrame *p_keyFrame_inout)
    {
        setEstimate(ImuCamPose(p_keyFrame_inout));
    }
    /*!
     * @brief           Creates the vertex with the pose of a frame as its
     *                  estimate.
     *
     * @param[in]       p_pF_inout
     *                  Frame to read the pose from; dereferenced without a null
     *                  check, not changed and not kept.
     */
    VertexPose(Frame *p_pF_inout)
    {
        setEstimate(ImuCamPose(p_pF_inout));
    }

    /*!
     * @brief           Reads the camera poses, extrinsics, camera model
     *                  parameters and baseline-focal product from a stream into
     *                  the estimate, in the order write() produces them. The
     *                  stream state is not checked.
     *
     * @param[in,out]   inputStream_inout
     *                  Stream to read from.
     *
     * @return          true always.
     */
    virtual bool read(std::istream &inputStream_inout);
    /*!
     * @brief           Writes the camera poses, extrinsics, camera model
     *                  parameters and baseline-focal product to a stream,
     *                  separated by spaces.
     *
     * @param[out]      outputStream_out
     *                  Stream to write to.
     *
     * @return          true when the stream is still good after writing.
     */
    virtual bool write(std::ostream &outputStream_out) const;

    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    virtual void setToOriginImpl() {}

    /*!
     * @brief           Applies an optimiser step to the pose in the body frame
     *                  and refreshes the g2o cache.
     *
     * @param[in]       p_update_in
     *                  Six values: rotation vector (radians) then translation
     *                  (metres), as for ImuCamPose::update.
     */
    virtual void oplusImpl(const double *p_update_in)
    {
        if (_estimate.update(p_update_in) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: update returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        updateCache();
    }
};

/*!
 * @brief           g2o vertex for pose-graph optimisation whose estimate is an
 *                  ImuCamPose; only the translation and the yaw (rotation about
 *                  the world z axis) are optimised, four degrees of freedom.
 */
class VertexPose4DoF : public g2o::BaseVertex<4, ImuCamPose>
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
    VertexPose4DoF(KeyFrame *p_keyFrame_inout)
    {
        setEstimate(ImuCamPose(p_keyFrame_inout));
    }
    /*!
     * @brief           Creates the vertex with the pose of a frame as its
     *                  estimate.
     *
     * @param[in]       p_pF_inout
     *                  Frame to read the pose from; dereferenced without a null
     *                  check, not changed and not kept.
     */
    VertexPose4DoF(Frame *p_pF_inout)
    {
        setEstimate(ImuCamPose(p_pF_inout));
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
    VertexPose4DoF(Eigen::Matrix3d &cameraRotation_cameraToWorld_inout,
                   Eigen::Vector3d &cameraTranslation_cameraToWorld_inout,
                   KeyFrame        *p_keyFrame_inout)
    {

        setEstimate(ImuCamPose(cameraRotation_cameraToWorld_inout,
                               cameraTranslation_cameraToWorld_inout,
                               p_keyFrame_inout));
    }

    /*!
     * @brief           Does nothing: reading this vertex from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this vertex to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    virtual void setToOriginImpl() {}

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
    virtual void oplusImpl(const double *p_update_in)
    {
        double update6DoF[6];
        update6DoF[0] = 0;
        update6DoF[1] = 0;
        update6DoF[2] = p_update_in[0];
        update6DoF[3] = p_update_in[1];
        update6DoF[4] = p_update_in[2];
        update6DoF[5] = p_update_in[3];
        if (_estimate.updateW(update6DoF) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateW returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        updateCache();
    }
};

/*!
 * @brief           g2o vertex whose estimate is the IMU velocity in the world
 *                  frame, metres per second.
 */
class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexVelocity() {}
    /*!
     * @brief           Creates the vertex with the velocity of a key frame as
     *                  its estimate.
     *
     * @param[in]       p_keyFrame_inout
     *                  Key frame to read the velocity from; dereferenced
     *                  without a null check, not changed and not kept.
     */
    VertexVelocity(KeyFrame *p_keyFrame_inout)
    {
        Eigen::Vector3f keyFrameVelocity{};
        if (p_keyFrame_inout->getVelocity(keyFrameVelocity) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        setEstimate(keyFrameVelocity.cast<double>());
    }
    /*!
     * @brief           Creates the vertex with the velocity of a frame as its
     *                  estimate.
     *
     * @param[in]       p_pF_inout
     *                  Frame to read the velocity from; dereferenced without a
     *                  null check, not changed and not kept.
     */
    VertexVelocity(Frame *p_pF_inout)
    {
        Eigen::Vector3f pFGetVelocity{};
        if (p_pF_inout->getVelocity(pFGetVelocity) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        setEstimate(pFGetVelocity.cast<double>());
    }

    /*!
     * @brief           Does nothing: reading this vertex from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this vertex to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    virtual void setToOriginImpl() {}

    /*!
     * @brief           Adds an optimiser step to the velocity.
     *
     * @param[in]       p_update_in
     *                  Three values: change of the velocity in the world frame,
     *                  metres per second.
     */
    virtual void oplusImpl(const double *p_update_in)
    {
        Eigen::Vector3d uv;
        uv << p_update_in[0], p_update_in[1], p_update_in[2];
        setEstimate(estimate() + uv);
    }
};

/*!
 * @brief           g2o vertex whose estimate is the gyroscope bias of the IMU,
 *                  radians per second, in the body frame.
 */
class VertexGyroBias : public g2o::BaseVertex<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGyroBias() {}
    /*!
     * @brief           Creates the vertex with the gyroscope bias of a key
     *                  frame as its estimate.
     *
     * @param[in]       p_keyFrame_inout
     *                  Key frame to read the bias from; dereferenced without a
     *                  null check, not changed and not kept.
     */
    VertexGyroBias(KeyFrame *p_keyFrame_inout)
    {
        Eigen::Vector3f keyFrameGyroBias{};
        if (p_keyFrame_inout->getGyroBias(keyFrameGyroBias) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGyroBias returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        setEstimate(keyFrameGyroBias.cast<double>());
    }
    /*!
     * @brief           Creates the vertex with the gyroscope bias of a frame as
     *                  its estimate.
     *
     * @param[in]       p_pF_inout
     *                  Frame to read the bias from; dereferenced without a null
     *                  check, not changed and not kept.
     */
    VertexGyroBias(Frame *p_pF_inout)
    {
        Eigen::Vector3d bg;
        bg << p_pF_inout->imuBias.bwx, p_pF_inout->imuBias.bwy,
            p_pF_inout->imuBias.bwz;
        setEstimate(bg);
    }

    /*!
     * @brief           Does nothing: reading this vertex from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this vertex to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    virtual void setToOriginImpl() {}

    /*!
     * @brief           Adds an optimiser step to the gyroscope bias.
     *
     * @param[in]       p_update_in
     *                  Three values: change of the bias, radians per second.
     */
    virtual void oplusImpl(const double *p_update_in)
    {
        Eigen::Vector3d ubg;
        ubg << p_update_in[0], p_update_in[1], p_update_in[2];
        setEstimate(estimate() + ubg);
    }
};

/*!
 * @brief           g2o vertex whose estimate is the accelerometer bias of the
 *                  IMU, metres per second squared, in the body frame.
 */
class VertexAccBias : public g2o::BaseVertex<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexAccBias() {}
    /*!
     * @brief           Creates the vertex with the accelerometer bias of a key
     *                  frame as its estimate.
     *
     * @param[in]       p_keyFrame_inout
     *                  Key frame to read the bias from; dereferenced without a
     *                  null check, not changed and not kept.
     */
    VertexAccBias(KeyFrame *p_keyFrame_inout)
    {
        Eigen::Vector3f keyFrameAccBias{};
        if (p_keyFrame_inout->getAccBias(keyFrameAccBias) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAccBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        setEstimate(keyFrameAccBias.cast<double>());
    }
    /*!
     * @brief           Creates the vertex with the accelerometer bias of a
     *                  frame as its estimate.
     *
     * @param[in]       p_pF_inout
     *                  Frame to read the bias from; dereferenced without a null
     *                  check, not changed and not kept.
     */
    VertexAccBias(Frame *p_pF_inout)
    {
        Eigen::Vector3d ba;
        ba << p_pF_inout->imuBias.bax, p_pF_inout->imuBias.bay,
            p_pF_inout->imuBias.baz;
        setEstimate(ba);
    }

    /*!
     * @brief           Does nothing: reading this vertex from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this vertex to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    virtual void setToOriginImpl() {}

    /*!
     * @brief           Adds an optimiser step to the accelerometer bias.
     *
     * @param[in]       p_update_in
     *                  Three values: change of the bias, metres per second
     *                  squared.
     */
    virtual void oplusImpl(const double *p_update_in)
    {
        Eigen::Vector3d uba;
        uba << p_update_in[0], p_update_in[1], p_update_in[2];
        setEstimate(estimate() + uba);
    }
};

/*!
 * @brief           Direction of gravity expressed as a rotation from a
 *                  gravity-aligned frame (z axis along gravity) to the world
 *                  frame. Only two degrees of freedom can change, because
 *                  rotating about gravity itself has no effect.
 */
class GDirection
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GDirection() :
        gravityRotation_gravityToWorld(Eigen::Matrix3d::Identity()),
        gravityRotation_worldToGravity(Eigen::Matrix3d::Identity()),
        its(0)
    {}

    /*!
     * @brief           Creates the gravity direction from a known rotation.
     *
     * @param[in]       gravityRotation_gravityToWorld_in
     *                  Rotation from the gravity-aligned frame to the world
     *                  frame.
     */
    explicit GDirection(
        const Eigen::Matrix3d &gravityRotation_gravityToWorld_in) :
        gravityRotation_gravityToWorld(gravityRotation_gravityToWorld_in),
        gravityRotation_worldToGravity(
            gravityRotation_gravityToWorld_in.transpose()),
        its(0)
    {}

    /*!
     * @brief           Applies an optimiser step by rotating the
     *                  gravity-aligned frame about its own x and y axes, then
     *                  refreshes the inverse rotation.
     *
     * @param[in]       p_pu_in
     *                  Two values: rotation about x and about y, radians.
     *
     * @return          GDIRECTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] GDirectionStatus update(const double *p_pu_in)
    {
        Eigen::Matrix3d rotation{};
        if (expSO3(p_pu_in[0], p_pu_in[1], 0.0, rotation) !=
            G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: expSO3 returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        gravityRotation_gravityToWorld =
            gravityRotation_gravityToWorld * rotation;
        gravityRotation_worldToGravity =
            gravityRotation_gravityToWorld.transpose();

        return GDirectionStatus::GDIRECTION_STATUS_SUCCESS;
    }

    /*!
     * @brief           Rotation from the gravity-aligned frame to the world
     *                  frame.
     */
    Eigen::Matrix3d gravityRotation_gravityToWorld;
    /*!
     * @brief           Rotation from the world frame to the gravity-aligned
     *                  frame; the transpose of gravityRotation_gravityToWorld.
     */
    Eigen::Matrix3d gravityRotation_worldToGravity;

    /*!
     * @brief           Number of updates applied; set to zero at construction
     *                  and not used by update.
     */
    int its;
};

/*!
 * @brief           g2o vertex whose estimate is a GDirection, the direction of
 *                  gravity in the world, with two degrees of freedom.
 */
class VertexGDir : public g2o::BaseVertex<2, GDirection>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGDir() {}
    /*!
     * @brief           Creates the vertex with a known gravity rotation as its
     *                  estimate.
     *
     * @param[in]       rwg_in
     *                  Rotation from the gravity-aligned frame to the world
     *                  frame.
     */
    VertexGDir(Eigen::Matrix3d rwg_in)
    {
        setEstimate(GDirection(rwg_in));
    }

    /*!
     * @brief           Does nothing: reading this vertex from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this vertex to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    virtual void setToOriginImpl() {}

    /*!
     * @brief           Applies an optimiser step to the gravity direction and
     *                  refreshes the g2o cache.
     *
     * @param[in]       p_update_in
     *                  Two values: rotation about the x and y axes of the
     *                  gravity-aligned frame, radians.
     */
    virtual void oplusImpl(const double *p_update_in)
    {
        if (_estimate.update(p_update_in) !=
            GDirectionStatus::GDIRECTION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: update returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        updateCache();
    }
};

/*!
 * @brief           g2o vertex whose estimate is a scale factor, which the
 *                  inertial edges multiply onto map distances to compare them
 *                  with metric IMU measurements.
 */
class VertexScale : public g2o::BaseVertex<1, double>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexScale()
    {
        setEstimate(1.0);
    }
    /*!
     * @brief           Creates the vertex with an initial scale.
     *
     * @param[in]       ps_in
     *                  Initial scale factor.
     */
    VertexScale(double ps_in)
    {
        setEstimate(ps_in);
    }

    /*!
     * @brief           Does nothing: reading this vertex from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this vertex to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Resets the scale estimate to 1.0, meaning no scaling.
     */
    virtual void setToOriginImpl()
    {
        setEstimate(1.0);
    }

    /*!
     * @brief           Applies an optimiser step to the scale by multiplying it
     *                  with the exponential of the step, which keeps a positive
     *                  scale positive.
     *
     * @param[in]       p_update_in
     *                  Pointer to one value, the logarithm of the factor to
     *                  multiply the scale by.
     */
    virtual void oplusImpl(const double *p_update_in)
    {
        setEstimate(estimate() * exp(*p_update_in));
    }
};

/*!
 * @brief           g2o vertex whose estimate is an InvDepthPoint, a map point
 *                  with one degree of freedom, its inverse depth in the host
 *                  key frame.
 */
class VertexInvDepth : public g2o::BaseVertex<1, InvDepthPoint>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexInvDepth() {}
    /*!
     * @brief           Creates the vertex with an InvDepthPoint as its
     *                  estimate.
     *
     * @param[in]       invDepth_in
     *                  Initial inverse depth in the host key frame, one over
     *                  metres.
     *
     * @param[in]       u_in
     *                  Pixel column of the observation in the host key frame.
     *
     * @param[in]       v_in
     *                  Pixel row of the observation in the host key frame.
     *
     * @param[in]       p_hostKeyFrame_inout
     *                  Host key frame whose intrinsics are copied; dereferenced
     *                  without a null check, not changed and not kept.
     */
    VertexInvDepth(double    invDepth_in,
                   double    u_in,
                   double    v_in,
                   KeyFrame *p_hostKeyFrame_inout)
    {
        setEstimate(
            InvDepthPoint(invDepth_in, u_in, v_in, p_hostKeyFrame_inout));
    }

    /*!
     * @brief           Does nothing: reading this vertex from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this vertex to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Does nothing; required by g2o, and the estimate keeps
     *                  its current value.
     */
    virtual void setToOriginImpl() {}

    /*!
     * @brief           Applies an optimiser step to the inverse depth and
     *                  refreshes the g2o cache.
     *
     * @param[in]       p_update_in
     *                  Pointer to one value, the change of the inverse depth in
     *                  one over metres.
     */
    virtual void oplusImpl(const double *p_update_in)
    {
        if (_estimate.update(p_update_in) !=
            InvDepthPointStatus::INV_DEPTH_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: update returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        updateCache();
    }
};

/*!
 * @brief           g2o binary edge for a monocular reprojection: connects a map
 *                  point (vertex 0, world frame) and a VertexPose (vertex 1).
 *                  The two-value error is the observed pixel minus the
 *                  projected pixel.
 */
class EdgeMono
    : public g2o::
          BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexPose>
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
    EdgeMono(int cameraIndex_in = 0) :
        cam_idx(cameraIndex_in)
    {}

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the observed pixel minus the pixel the
     *                  map point projects to in camera cam_idx. Failed
     *                  projections are only logged.
     */
    void computeError()
    {
        const g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[1]);
        const Eigen::Vector2d observation(_measurement);
        Eigen::Vector2d       projection{};
        if (p_poseVertex->estimate().project(p_pointVertex->estimate(),
                                             projection,
                                             cam_idx) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: project returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        _error = observation - projection;
    }

    /*!
     * @brief           Fills the Jacobians of the error with respect to the map
     *                  point (3 columns, _jacobianOplusXi) and the pose (6
     *                  columns, _jacobianOplusXj, rotation then translation in
     *                  the body frame, as for ImuCamPose::update).
     */
    virtual void linearizeOplus();

    /*!
     * @brief           Tells whether the map point lies in front of camera
     *                  cam_idx at the current estimates.
     *
     * @return          true when the depth of the point in the camera frame is
     *                  positive.
     */
    bool isDepthPositive()
    {
        const g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[1]);
        return p_poseVertex->estimate().isDepthPositive(
            p_pointVertex->estimate(),
            cam_idx);
    }

    /*!
     * @brief           Returns the 2 by 9 Jacobian of the error, columns
     *                  ordered map point (3) then pose (6). Relinearises at the
     *                  current estimates first.
     *
     * @param[out]      jacobian_out
     *                  Jacobian of the error.
     *
     * @return          EDGE_MONO_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeMonoStatus
        getJacobian(Eigen::Matrix<double, 2, 9> &jacobian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 2, 9> J;
        J.block<2, 3>(0, 0) = _jacobianOplusXi;
        J.block<2, 6>(0, 3) = _jacobianOplusXj;
        jacobian_out        = J;
        return EdgeMonoStatus::EDGE_MONO_STATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 9 by 9 Gauss-Newton Hessian block, the
     *                  Jacobian transposed times the information matrix times
     *                  the Jacobian, ordered map point (3) then pose (6).
     *                  Relinearises at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_MONO_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeMonoStatus
        getHessian(Eigen::Matrix<double, 9, 9> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 2, 9> J;
        J.block<2, 3>(0, 0) = _jacobianOplusXi;
        J.block<2, 6>(0, 3) = _jacobianOplusXj;
        hessian_out         = J.transpose() * information() * J;
        return EdgeMonoStatus::EDGE_MONO_STATUS_SUCCESS;
    }

  public:
    /*!
     * @brief           Camera of the rig that made the observation, 0 for the
     *                  left and 1 for the right.
     */
    const int cam_idx;
};

/*!
 * @brief           g2o unary edge for a monocular reprojection that optimises
 *                  only the pose (vertex 0, a VertexPose) of a map point whose
 *                  world position is held fixed. The error is the observed
 *                  pixel minus the projected pixel.
 */
class EdgeMonoOnlyPose
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates the edge for a fixed map point seen by one
     *                  camera.
     *
     * @param[in]       Xw_in
     *                  Map point in the world frame, metres.
     *
     * @param[in]       cameraIndex_in
     *                  Camera that made the observation, 0 for the left and 1
     *                  for the right.
     */
    EdgeMonoOnlyPose(const Eigen::Vector3f &Xw_in, int cameraIndex_in = 0) :
        Xw(Xw_in.cast<double>()),
        cam_idx(cameraIndex_in)
    {}

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the observed pixel minus the pixel the
     *                  fixed map point projects to in camera cam_idx. Failed
     *                  projections are only logged.
     */
    void computeError()
    {
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[0]);
        const Eigen::Vector2d observation(_measurement);
        Eigen::Vector2d       projection{};
        if (p_poseVertex->estimate().project(Xw, projection, cam_idx) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: project returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        _error = observation - projection;
    }

    /*!
     * @brief           Fills the Jacobian of the error with respect to the pose
     *                  (2 by 6, _jacobianOplusXi, rotation then translation in
     *                  the body frame, as for ImuCamPose::update).
     */
    virtual void linearizeOplus();

    /*!
     * @brief           Tells whether the fixed map point lies in front of
     *                  camera cam_idx at the current pose.
     *
     * @return          true when the depth of the point in the camera frame is
     *                  positive.
     */
    bool isDepthPositive()
    {
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[0]);
        return p_poseVertex->estimate().isDepthPositive(Xw, cam_idx);
    }

    /*!
     * @brief           Returns the 6 by 6 Gauss-Newton Hessian block of the
     *                  pose. Relinearises at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_MONO_ONLY_POSE_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeMonoOnlyPoseStatus
        getHessian(Eigen::Matrix<double, 6, 6> &hessian_out)
    {
        linearizeOplus();
        hessian_out =
            _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
        return EdgeMonoOnlyPoseStatus::EDGE_MONO_ONLY_POSE_STATUS_SUCCESS;
    }

  public:
    /*!
     * @brief           Map point in the world frame, metres; fixed, not
     *                  optimised.
     */
    const Eigen::Vector3d Xw;
    /*!
     * @brief           Camera of the rig that made the observation, 0 for the
     *                  left and 1 for the right.
     */
    const int             cam_idx;
};

/*!
 * @brief           g2o binary edge for a stereo reprojection: connects a map
 *                  point (vertex 0, world frame) and a VertexPose (vertex 1).
 *                  The three-value error is the observed (u, v, u_right) minus
 *                  the projected one.
 */
class EdgeStereo
    : public g2o::
          BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, VertexPose>
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
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the observed (u, v, u_right) minus the
     *                  stereo projection of the map point in camera cam_idx.
     *                  Failed projections are only logged.
     */
    void computeError()
    {
        const g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[1]);
        const Eigen::Vector3d observation(_measurement);
        Eigen::Vector3d       stereo{};
        if (p_poseVertex->estimate().projectStereo(p_pointVertex->estimate(),
                                                   stereo,
                                                   cam_idx) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: projectStereo returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        _error = observation - stereo;
    }

    /*!
     * @brief           Fills the Jacobians of the error with respect to the map
     *                  point (3 columns, _jacobianOplusXi) and the pose (6
     *                  columns, _jacobianOplusXj, rotation then translation in
     *                  the body frame, as for ImuCamPose::update).
     */
    virtual void linearizeOplus();

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
        getJacobian(Eigen::Matrix<double, 3, 9> &jacobian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 9> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 6>(0, 3) = _jacobianOplusXj;
        jacobian_out        = J;
        return EdgeStereoStatus::EDGE_STEREO_STATUS_SUCCESS;
    }

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
        getHessian(Eigen::Matrix<double, 9, 9> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 9> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 6>(0, 3) = _jacobianOplusXj;
        hessian_out         = J.transpose() * information() * J;
        return EdgeStereoStatus::EDGE_STEREO_STATUS_SUCCESS;
    }

  public:
    /*!
     * @brief           Camera of the rig that made the observation, 0 for the
     *                  left and 1 for the right.
     */
    const int cam_idx;
};

/*!
 * @brief           g2o unary edge for a stereo reprojection that optimises only
 *                  the pose (vertex 0, a VertexPose) of a map point whose world
 *                  position is held fixed. The error is the observed (u, v,
 *                  u_right) minus the projected one.
 */
class EdgeStereoOnlyPose
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates the edge for a fixed map point seen by one
     *                  camera.
     *
     * @param[in]       Xw_in
     *                  Map point in the world frame, metres.
     *
     * @param[in]       cameraIndex_in
     *                  Camera that made the observation, 0 for the left and 1
     *                  for the right.
     */
    EdgeStereoOnlyPose(const Eigen::Vector3f &Xw_in, int cameraIndex_in = 0) :
        Xw(Xw_in.cast<double>()),
        cam_idx(cameraIndex_in)
    {}

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the observed (u, v, u_right) minus the
     *                  stereo projection of the fixed map point in camera
     *                  cam_idx. Failed projections are only logged.
     */
    void computeError()
    {
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[0]);
        const Eigen::Vector3d observation(_measurement);
        Eigen::Vector3d       stereo{};
        if (p_poseVertex->estimate().projectStereo(Xw, stereo, cam_idx) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: projectStereo returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        _error = observation - stereo;
    }

    /*!
     * @brief           Fills the Jacobian of the error with respect to the pose
     *                  (3 by 6, _jacobianOplusXi, rotation then translation in
     *                  the body frame, as for ImuCamPose::update).
     */
    virtual void linearizeOplus();

    /*!
     * @brief           Returns the 6 by 6 Gauss-Newton Hessian block of the
     *                  pose. Relinearises at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_STEREO_ONLY_POSE_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeStereoOnlyPoseStatus
        getHessian(Eigen::Matrix<double, 6, 6> &hessian_out)
    {
        linearizeOplus();
        hessian_out =
            _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
        return EdgeStereoOnlyPoseStatus::EDGE_STEREO_ONLY_POSE_STATUS_SUCCESS;
    }

  public:
    /*!
     * @brief           Map point in the world frame, metres; fixed, not
     *                  optimised.
     */
    const Eigen::Vector3d Xw;
    /*!
     * @brief           Camera of the rig that made the observation, 0 for the
     *                  left and 1 for the right.
     */
    const int             cam_idx;
};

/*!
 * @brief           g2o multi-edge that ties two consecutive frames together
 *                  with their IMU preintegration, assuming gravity points along
 *                  the world -z axis. Six vertices in order: previous
 *                  VertexPose, previous VertexVelocity, VertexGyroBias,
 *                  VertexAccBias, current VertexPose, current VertexVelocity.
 *                  The nine-value error is rotation, velocity and position.
 */
class EdgeInertial : public g2o::BaseMultiEdge<9, Vector9d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates the edge from a preintegration. The information
     *                  matrix is the inverse of the preintegrated covariance
     *                  (rotation, velocity and position block) with eigenvalues
     *                  below 1e-12 set to zero.
     *
     * @param[in]       p_int_inout
     *                  IMU preintegration between two consecutive frames; the
     *                  edge keeps this pointer (borrowed, never deleted) and
     *                  copies its bias Jacobians and time span. Dereferenced
     *                  without a null check.
     */
    EdgeInertial(IMU::Preintegrated *p_int_inout) :
        JRg(p_int_inout->JRg.cast<double>()),
        JVg(p_int_inout->JVg.cast<double>()),
        JPg(p_int_inout->JPg.cast<double>()),
        JVa(p_int_inout->JVa.cast<double>()),
        JPa(p_int_inout->JPa.cast<double>()),
        p_preintegrated(p_int_inout),
        dt(p_int_inout->dT)
    {
        // This edge links 6 vertices
        resize(6);
        g << 0, 0, -IMU::GRAVITY_VALUE;

        Matrix9d Info =
            p_int_inout->C.block<9, 9>(0, 0).cast<double>().inverse();
        Info = (Info + Info.transpose()) / 2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(Info);
        Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
        for (int i = 0; i < 9; i++)
            if (eigs[i] < 1e-12)
                eigs[i] = 0;
        Info = es.eigenvectors() * eigs.asDiagonal() *
               es.eigenvectors().transpose();
        setInformation(Info);
    }

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the difference between the motion of
     *                  the two frames and the preintegrated motion corrected
     *                  with the current bias estimates; the nine values are
     *                  rotation, velocity and position.
     */
    void         computeError();
    /*!
     * @brief           Fills the Jacobians of the error, _jacobianOplus[0..5],
     *                  one nine-row block for each vertex in the order of the
     *                  class description.
     */
    virtual void linearizeOplus();

    /*!
     * @brief           Returns the 24 by 24 Gauss-Newton Hessian over all six
     *                  vertices, ordered previous pose (6), previous velocity
     *                  (3), gyroscope bias (3), accelerometer bias (3), current
     *                  pose (6), current velocity (3). Relinearises at the
     *                  current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialStatus
        getHessian(Eigen::Matrix<double, 24, 24> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 24> J;
        J.block<9, 6>(0, 0)  = _jacobianOplus[0];
        J.block<9, 3>(0, 6)  = _jacobianOplus[1];
        J.block<9, 3>(0, 9)  = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        hessian_out          = J.transpose() * information() * J;
        return EdgeInertialStatus::EDGE_INERTIAL_STATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 18 by 18 Gauss-Newton Hessian over all
     *                  vertices except the previous pose, ordered previous
     *                  velocity (3), gyroscope bias (3), accelerometer bias
     *                  (3), current pose (6), current velocity (3).
     *                  Relinearises at the current estimates first.
     *
     * @param[out]      hessianNoPose1_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialStatus
        getHessianNoPose1(Eigen::Matrix<double, 18, 18> &hessianNoPose1_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 18> J;
        J.block<9, 3>(0, 0)  = _jacobianOplus[1];
        J.block<9, 3>(0, 3)  = _jacobianOplus[2];
        J.block<9, 3>(0, 6)  = _jacobianOplus[3];
        J.block<9, 6>(0, 9)  = _jacobianOplus[4];
        J.block<9, 3>(0, 15) = _jacobianOplus[5];
        hessianNoPose1_out   = J.transpose() * information() * J;
        return EdgeInertialStatus::EDGE_INERTIAL_STATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 9 by 9 Gauss-Newton Hessian over the current
     *                  frame only, ordered current pose (6), current velocity
     *                  (3). Relinearises at the current estimates first.
     *
     * @param[out]      hessian2_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialStatus
        getHessian2(Eigen::Matrix<double, 9, 9> &hessian2_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 9> J;
        J.block<9, 6>(0, 0) = _jacobianOplus[4];
        J.block<9, 3>(0, 6) = _jacobianOplus[5];
        hessian2_out        = J.transpose() * information() * J;
        return EdgeInertialStatus::EDGE_INERTIAL_STATUS_SUCCESS;
    }

    /*!
     * @brief           Jacobian of the preintegrated rotation with respect to
     *                  the gyroscope bias, copied from the preintegration.
     */
    const Eigen::Matrix3d JRg;
    /*!
     * @brief           Jacobian of the preintegrated velocity with respect to
     *                  the gyroscope bias, copied from the preintegration.
     */
    const Eigen::Matrix3d JVg;
    /*!
     * @brief           Jacobian of the preintegrated position with respect to
     *                  the gyroscope bias, copied from the preintegration.
     */
    const Eigen::Matrix3d JPg;
    /*!
     * @brief           Jacobian of the preintegrated velocity with respect to
     *                  the accelerometer bias, copied from the preintegration.
     */
    const Eigen::Matrix3d JVa;
    /*!
     * @brief           Jacobian of the preintegrated position with respect to
     *                  the accelerometer bias, copied from the preintegration.
     */
    const Eigen::Matrix3d JPa;
    /*!
     * @brief           IMU preintegration this edge compares against; borrowed,
     *                  never deleted here.
     */
    IMU::Preintegrated   *p_preintegrated;
    /*!
     * @brief           Time span of the preintegration, seconds.
     */
    const double          dt;
    /*!
     * @brief           Gravity in the world frame, metres per second squared;
     *                  fixed at (0, 0, -GRAVITY_VALUE).
     */
    Eigen::Vector3d       g;
};

/*!
 * @brief           Like EdgeInertial, but gravity direction and scale are
 *                  optimised too, so gravity need not point along the world -z
 *                  axis. Eight vertices in order: previous VertexPose, previous
 *                  VertexVelocity, VertexGyroBias, VertexAccBias, current
 *                  VertexPose, current VertexVelocity, VertexGDir, VertexScale.
 */
class EdgeInertialGS : public g2o::BaseMultiEdge<9, Vector9d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // EdgeInertialGS(IMU::Preintegrated* pInt);
    /*!
     * @brief           Creates the edge from a preintegration. The information
     *                  matrix is the inverse of the preintegrated covariance
     *                  (rotation, velocity and position block) with eigenvalues
     *                  below 1e-12 set to zero.
     *
     * @param[in]       p_int_inout
     *                  IMU preintegration between two consecutive frames; the
     *                  edge keeps this pointer (borrowed, never deleted) and
     *                  copies its bias Jacobians and time span. Dereferenced
     *                  without a null check.
     */
    EdgeInertialGS(IMU::Preintegrated *p_int_inout) :
        JRg(p_int_inout->JRg.cast<double>()),
        JVg(p_int_inout->JVg.cast<double>()),
        JPg(p_int_inout->JPg.cast<double>()),
        JVa(p_int_inout->JVa.cast<double>()),
        JPa(p_int_inout->JPa.cast<double>()),
        p_preintegrated(p_int_inout),
        dt(p_int_inout->dT)
    {
        // This edge links 8 vertices
        resize(8);
        gI << 0, 0, -IMU::GRAVITY_VALUE;

        Matrix9d Info =
            p_int_inout->C.block<9, 9>(0, 0).cast<double>().inverse();
        Info = (Info + Info.transpose()) / 2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(Info);
        Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
        for (int i = 0; i < 9; i++)
            if (eigs[i] < 1e-12)
                eigs[i] = 0;
        Info = es.eigenvectors() * eigs.asDiagonal() *
               es.eigenvectors().transpose();
        setInformation(Info);
    }

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the difference between the motion of
     *                  the two frames, with the gravity direction and scale
     *                  estimates applied, and the preintegrated motion
     *                  corrected with the current bias estimates; the nine
     *                  values are rotation, velocity and position. Also
     *                  refreshes g from the gravity direction vertex.
     */
    void         computeError();
    /*!
     * @brief           Fills the Jacobians of the error, _jacobianOplus[0..7],
     *                  one nine-row block for each vertex in the order of the
     *                  class description.
     */
    virtual void linearizeOplus();

    /*!
     * @brief           Jacobian of the preintegrated rotation with respect to
     *                  the gyroscope bias, copied from the preintegration.
     */
    const Eigen::Matrix3d JRg;
    /*!
     * @brief           Jacobian of the preintegrated velocity with respect to
     *                  the gyroscope bias, copied from the preintegration.
     */
    const Eigen::Matrix3d JVg;
    /*!
     * @brief           Jacobian of the preintegrated position with respect to
     *                  the gyroscope bias, copied from the preintegration.
     */
    const Eigen::Matrix3d JPg;
    /*!
     * @brief           Jacobian of the preintegrated velocity with respect to
     *                  the accelerometer bias, copied from the preintegration.
     */
    const Eigen::Matrix3d JVa;
    /*!
     * @brief           Jacobian of the preintegrated position with respect to
     *                  the accelerometer bias, copied from the preintegration.
     */
    const Eigen::Matrix3d JPa;
    /*!
     * @brief           IMU preintegration this edge compares against; borrowed,
     *                  never deleted here.
     */
    IMU::Preintegrated   *p_preintegrated;
    /*!
     * @brief           Time span of the preintegration, seconds.
     */
    const double          dt;
    /*!
     * @brief           Gravity in the world frame, metres per second squared;
     *                  recomputed by computeError as the gravity rotation times
     *                  gI.
     */
    Eigen::Vector3d       g;
    /*!
     * @brief           Gravity in the gravity-aligned frame, metres per second
     *                  squared; fixed at (0, 0, -GRAVITY_VALUE).
     */
    Eigen::Vector3d       gI;

    /*!
     * @brief           Returns the 27 by 27 Gauss-Newton Hessian over all eight
     *                  vertices, ordered previous pose (6), previous velocity
     *                  (3), gyroscope bias (3), accelerometer bias (3), current
     *                  pose (6), current velocity (3), gravity direction (2),
     *                  scale (1). Relinearises at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessian(Eigen::Matrix<double, 27, 27> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 27> J;
        J.block<9, 6>(0, 0)  = _jacobianOplus[0];
        J.block<9, 3>(0, 6)  = _jacobianOplus[1];
        J.block<9, 3>(0, 9)  = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        J.block<9, 2>(0, 24) = _jacobianOplus[6];
        J.block<9, 1>(0, 26) = _jacobianOplus[7];
        hessian_out          = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 27 by 27 Gauss-Newton Hessian over all eight
     *                  vertices in a different order: gyroscope bias (3),
     *                  accelerometer bias (3), gravity direction (2), scale
     *                  (1), previous velocity (3), current velocity (3),
     *                  previous pose (6), current pose (6). Relinearises at the
     *                  current estimates first.
     *
     * @param[out]      hessian2_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessian2(Eigen::Matrix<double, 27, 27> &hessian2_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 27> J;
        J.block<9, 3>(0, 0)  = _jacobianOplus[2];
        J.block<9, 3>(0, 3)  = _jacobianOplus[3];
        J.block<9, 2>(0, 6)  = _jacobianOplus[6];
        J.block<9, 1>(0, 8)  = _jacobianOplus[7];
        J.block<9, 3>(0, 9)  = _jacobianOplus[1];
        J.block<9, 3>(0, 12) = _jacobianOplus[5];
        J.block<9, 6>(0, 15) = _jacobianOplus[0];
        J.block<9, 6>(0, 21) = _jacobianOplus[4];
        hessian2_out         = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 9 by 9 Gauss-Newton Hessian over the
     *                  gyroscope bias (3), accelerometer bias (3), gravity
     *                  direction (2) and scale (1), in that order. Relinearises
     *                  at the current estimates first.
     *
     * @param[out]      hessian3_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessian3(Eigen::Matrix<double, 9, 9> &hessian3_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 9> J;
        J.block<9, 3>(0, 0) = _jacobianOplus[2];
        J.block<9, 3>(0, 3) = _jacobianOplus[3];
        J.block<9, 2>(0, 6) = _jacobianOplus[6];
        J.block<9, 1>(0, 8) = _jacobianOplus[7];
        hessian3_out        = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 1 by 1 Gauss-Newton Hessian of the scale
     *                  vertex. Relinearises at the current estimates first.
     *
     * @param[out]      hessianScale_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessianScale(Eigen::Matrix<double, 1, 1> &hessianScale_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 1> J = _jacobianOplus[7];
        hessianScale_out              = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 3 by 3 Gauss-Newton Hessian of the gyroscope
     *                  bias vertex. Relinearises at the current estimates
     *                  first.
     *
     * @param[out]      hessianBiasGyro_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessianBiasGyro(Eigen::Matrix<double, 3, 3> &hessianBiasGyro_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 3> J = _jacobianOplus[2];
        hessianBiasGyro_out           = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 3 by 3 Gauss-Newton Hessian of the
     *                  accelerometer bias vertex. Relinearises at the current
     *                  estimates first.
     *
     * @param[out]      hessianBiasAcc_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessianBiasAcc(Eigen::Matrix<double, 3, 3> &hessianBiasAcc_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 3> J = _jacobianOplus[3];
        hessianBiasAcc_out            = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 2 by 2 Gauss-Newton Hessian of the gravity
     *                  direction vertex. Relinearises at the current estimates
     *                  first.
     *
     * @param[out]      hessianGDir_out
     *                  Hessian block.
     *
     * @return          EDGE_INERTIAL_GSSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeInertialGSStatus
        getHessianGDir(Eigen::Matrix<double, 2, 2> &hessianGDir_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 2> J = _jacobianOplus[6];
        hessianGDir_out               = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }
};

/*!
 * @brief           g2o binary edge that models the gyroscope bias as a random
 *                  walk: it penalises the change of the bias between two
 *                  consecutive frames (vertex 0 and vertex 1, both
 *                  VertexGyroBias). The three-value error is the second bias
 *                  minus the first.
 */
class EdgeGyroRW
    : public g2o::
          BaseBinaryEdge<3, Eigen::Vector3d, VertexGyroBias, VertexGyroBias>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGyroRW() {}

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the second bias estimate minus the
     *                  first.
     */
    void computeError()
    {
        const VertexGyroBias *p_firstGyroBiasVertex =
            static_cast<const VertexGyroBias *>(_vertices[0]);
        const VertexGyroBias *p_secondGyroBiasVertex =
            static_cast<const VertexGyroBias *>(_vertices[1]);
        _error = p_secondGyroBiasVertex->estimate() -
                 p_firstGyroBiasVertex->estimate();
    }

    /*!
     * @brief           Sets the Jacobians of the error: minus the identity for
     *                  the first bias and the identity for the second.
     */
    virtual void linearizeOplus()
    {
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    /*!
     * @brief           Returns the 6 by 6 Gauss-Newton Hessian over both
     *                  biases, ordered first (3) then second (3). Relinearises
     *                  at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_GYRO_RWSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeGyroRWStatus
        getHessian(Eigen::Matrix<double, 6, 6> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        hessian_out         = J.transpose() * information() * J;
        return EdgeGyroRWStatus::EDGE_GYRO_RWSTATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 3 by 3 Gauss-Newton Hessian of the second
     *                  bias only. Relinearises at the current estimates first.
     *
     * @param[out]      hessian2_out
     *                  Hessian block.
     *
     * @return          EDGE_GYRO_RWSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeGyroRWStatus getHessian2(Eigen::Matrix3d &hessian2_out)
    {
        linearizeOplus();
        hessian2_out =
            _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
        return EdgeGyroRWStatus::EDGE_GYRO_RWSTATUS_SUCCESS;
    }
};

/*!
 * @brief           g2o binary edge that models the accelerometer bias as a
 *                  random walk: it penalises the change of the bias between two
 *                  consecutive frames (vertex 0 and vertex 1, both
 *                  VertexAccBias). The three-value error is the second bias
 *                  minus the first.
 */
class EdgeAccRW
    : public g2o::
          BaseBinaryEdge<3, Eigen::Vector3d, VertexAccBias, VertexAccBias>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAccRW() {}

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the second bias estimate minus the
     *                  first.
     */
    void computeError()
    {
        const VertexAccBias *p_firstAccelerometerBiasVertex =
            static_cast<const VertexAccBias *>(_vertices[0]);
        const VertexAccBias *p_secondAccelerometerBiasVertex =
            static_cast<const VertexAccBias *>(_vertices[1]);
        _error = p_secondAccelerometerBiasVertex->estimate() -
                 p_firstAccelerometerBiasVertex->estimate();
    }

    /*!
     * @brief           Sets the Jacobians of the error: minus the identity for
     *                  the first bias and the identity for the second.
     */
    virtual void linearizeOplus()
    {
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    /*!
     * @brief           Returns the 6 by 6 Gauss-Newton Hessian over both
     *                  biases, ordered first (3) then second (3). Relinearises
     *                  at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_ACC_RWSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeAccRWStatus
        getHessian(Eigen::Matrix<double, 6, 6> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        hessian_out         = J.transpose() * information() * J;
        return EdgeAccRWStatus::EDGE_ACC_RWSTATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 3 by 3 Gauss-Newton Hessian of the second
     *                  bias only. Relinearises at the current estimates first.
     *
     * @param[out]      hessian2_out
     *                  Hessian block.
     *
     * @return          EDGE_ACC_RWSTATUS_SUCCESS always.
     */
    [[nodiscard]] EdgeAccRWStatus getHessian2(Eigen::Matrix3d &hessian2_out)
    {
        linearizeOplus();
        hessian2_out =
            _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
        return EdgeAccRWStatus::EDGE_ACC_RWSTATUS_SUCCESS;
    }
};

/*!
 * @brief           Prior on the full IMU state of one frame: pose, velocity and
 *                  both biases, with the information matrix of that prior.
 *                  Consumed by EdgePriorPoseImu.
 */
class ConstraintPoseImu
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Stores the prior values and the information matrix, with
     *                  eigenvalues of the information matrix below 1e-12 set to
     *                  zero.
     *
     * @param[in]       bodyRotation_bodyToWorld_in
     *                  Prior rotation of the IMU (body) frame in the world.
     *
     * @param[in]       bodyTranslation_bodyToWorld_in
     *                  Prior position of the IMU origin in the world, metres.
     *
     * @param[in]       vwb_in
     *                  Prior velocity of the IMU in the world frame, metres per
     *                  second.
     *
     * @param[in]       bg_in
     *                  Prior gyroscope bias, radians per second.
     *
     * @param[in]       ba_in
     *                  Prior accelerometer bias, metres per second squared.
     *
     * @param[in]       H_in
     *                  Information matrix of the prior over rotation (3),
     *                  translation (3), velocity (3), gyroscope bias (3),
     *                  accelerometer bias (3).
     */
    ConstraintPoseImu(const Eigen::Matrix3d &bodyRotation_bodyToWorld_in,
                      const Eigen::Vector3d &bodyTranslation_bodyToWorld_in,
                      const Eigen::Vector3d &vwb_in,
                      const Eigen::Vector3d &bg_in,
                      const Eigen::Vector3d &ba_in,
                      const Matrix15d       &H_in) :
        Rwb(bodyRotation_bodyToWorld_in),
        twb(bodyTranslation_bodyToWorld_in),
        vwb(vwb_in),
        bg(bg_in),
        ba(ba_in),
        H(H_in)
    {
        H = (H + H) / 2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(H);
        Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
        for (int i = 0; i < 15; i++)
            if (eigs[i] < 1e-12)
                eigs[i] = 0;
        H = es.eigenvectors() * eigs.asDiagonal() *
            es.eigenvectors().transpose();
    }

    /*!
     * @brief           Prior rotation of the IMU (body) frame in the world
     *                  frame.
     */
    Eigen::Matrix3d Rwb;
    /*!
     * @brief           Prior position of the IMU origin in the world frame,
     *                  metres.
     */
    Eigen::Vector3d twb;
    /*!
     * @brief           Prior velocity of the IMU in the world frame, metres per
     *                  second.
     */
    Eigen::Vector3d vwb;
    /*!
     * @brief           Prior gyroscope bias, radians per second.
     */
    Eigen::Vector3d bg;
    /*!
     * @brief           Prior accelerometer bias, metres per second squared.
     */
    Eigen::Vector3d ba;
    /*!
     * @brief           Information matrix of the prior, ordered rotation,
     *                  translation, velocity, gyroscope bias, accelerometer
     *                  bias; the eigenvalue clean-up of the constructor has
     *                  been applied.
     */
    Matrix15d       H;
};

/*!
 * @brief           g2o multi-edge that pulls the IMU state of one frame towards
 *                  a prior. Four vertices in order: VertexPose, VertexVelocity,
 *                  VertexGyroBias, VertexAccBias. The fifteen-value error is
 *                  rotation, translation, velocity, gyroscope bias,
 *                  accelerometer bias.
 */
class EdgePriorPoseImu : public g2o::BaseMultiEdge<15, Vector15d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates the edge from a prior: copies its values and
     *                  uses its information matrix.
     *
     * @param[in]       p_c_inout
     *                  Prior to copy; dereferenced without a null check, only
     *                  read and not kept.
     */
    EdgePriorPoseImu(ConstraintPoseImu *p_c_inout)
    {
        resize(4);
        Rwb = p_c_inout->Rwb;
        twb = p_c_inout->twb;
        vwb = p_c_inout->vwb;
        bg  = p_c_inout->bg;
        ba  = p_c_inout->ba;
        setInformation(p_c_inout->H);
    }

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the current estimates minus the prior:
     *                  rotation error, translation error in the prior body
     *                  frame, then velocity and the two biases.
     */
    void         computeError();
    /*!
     * @brief           Fills the Jacobians of the error, _jacobianOplus[0..3],
     *                  one fifteen-row block for each vertex.
     */
    virtual void linearizeOplus();

    /*!
     * @brief           Returns the 15 by 15 Gauss-Newton Hessian over all four
     *                  vertices, ordered pose (6), velocity (3), gyroscope bias
     *                  (3), accelerometer bias (3). Relinearises at the current
     *                  estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_PRIOR_POSE_IMU_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgePriorPoseImuStatus
        getHessian(Eigen::Matrix<double, 15, 15> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 15, 15> J;
        J.block<15, 6>(0, 0)  = _jacobianOplus[0];
        J.block<15, 3>(0, 6)  = _jacobianOplus[1];
        J.block<15, 3>(0, 9)  = _jacobianOplus[2];
        J.block<15, 3>(0, 12) = _jacobianOplus[3];
        hessian_out           = J.transpose() * information() * J;
        return EdgePriorPoseImuStatus::EDGE_PRIOR_POSE_IMU_STATUS_SUCCESS;
    }

    /*!
     * @brief           Returns the 9 by 9 Gauss-Newton Hessian over velocity
     *                  (3), gyroscope bias (3) and accelerometer bias (3),
     *                  leaving out the pose. Relinearises at the current
     *                  estimates first.
     *
     * @param[out]      hessianNoPose_out
     *                  Hessian block.
     *
     * @return          EDGE_PRIOR_POSE_IMU_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgePriorPoseImuStatus
        getHessianNoPose(Eigen::Matrix<double, 9, 9> &hessianNoPose_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 15, 9> J;
        J.block<15, 3>(0, 0) = _jacobianOplus[1];
        J.block<15, 3>(0, 3) = _jacobianOplus[2];
        J.block<15, 3>(0, 6) = _jacobianOplus[3];
        hessianNoPose_out    = J.transpose() * information() * J;
        return EdgePriorPoseImuStatus::EDGE_PRIOR_POSE_IMU_STATUS_SUCCESS;
    }
    /*!
     * @brief           Prior rotation of the IMU (body) frame in the world
     *                  frame.
     */
    Eigen::Matrix3d Rwb;
    /*!
     * @brief           Prior position of the IMU origin in the world frame,
     *                  metres.
     */
    Eigen::Vector3d twb;
    /*!
     * @brief           Prior velocity of the IMU in the world frame, metres per
     *                  second.
     */
    Eigen::Vector3d vwb;
    /*!
     * @brief           Prior gyroscope bias, radians per second.
     */
    Eigen::Vector3d bg;
    /*!
     * @brief           Prior accelerometer bias, metres per second squared.
     */
    Eigen::Vector3d ba;
};

/*!
 * @brief           g2o unary edge that pulls the accelerometer bias (vertex 0,
 *                  a VertexAccBias) towards a prior value. The three-value
 *                  error is the prior minus the estimate.
 */
class EdgePriorAcc
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexAccBias>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates the edge with the prior bias.
     *
     * @param[in]       bprior_in
     *                  Prior accelerometer bias, metres per second squared.
     */
    EdgePriorAcc(const Eigen::Vector3f &bprior_in) :
        bprior(bprior_in.cast<double>())
    {}

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the prior bias minus the bias
     *                  estimate.
     */
    void computeError()
    {
        const VertexAccBias *p_accelerometerBiasVertex =
            static_cast<const VertexAccBias *>(_vertices[0]);
        _error = bprior - p_accelerometerBiasVertex->estimate();
    }
    /*!
     * @brief           Sets the Jacobian of the error with respect to the bias
     *                  to the identity.
     */
    virtual void linearizeOplus();

    /*!
     * @brief           Returns the 3 by 3 Gauss-Newton Hessian of the bias
     *                  vertex. Relinearises at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_PRIOR_ACC_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgePriorAccStatus
        getHessian(Eigen::Matrix<double, 3, 3> &hessian_out)
    {
        linearizeOplus();
        hessian_out =
            _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
        return EdgePriorAccStatus::EDGE_PRIOR_ACC_STATUS_SUCCESS;
    }

    /*!
     * @brief           Prior accelerometer bias, metres per second squared.
     */
    const Eigen::Vector3d bprior;
};

/*!
 * @brief           g2o unary edge that pulls the gyroscope bias (vertex 0, a
 *                  VertexGyroBias) towards a prior value. The three-value error
 *                  is the prior minus the estimate.
 */
class EdgePriorGyro
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexGyroBias>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates the edge with the prior bias.
     *
     * @param[in]       bprior_in
     *                  Prior gyroscope bias, radians per second.
     */
    EdgePriorGyro(const Eigen::Vector3f &bprior_in) :
        bprior(bprior_in.cast<double>())
    {}

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the prior bias minus the bias
     *                  estimate.
     */
    void computeError()
    {
        const VertexGyroBias *p_gyroBiasVertex =
            static_cast<const VertexGyroBias *>(_vertices[0]);
        _error = bprior - p_gyroBiasVertex->estimate();
    }
    /*!
     * @brief           Sets the Jacobian of the error with respect to the bias
     *                  to the identity.
     */
    virtual void linearizeOplus();

    /*!
     * @brief           Returns the 3 by 3 Gauss-Newton Hessian of the bias
     *                  vertex. Relinearises at the current estimates first.
     *
     * @param[out]      hessian_out
     *                  Hessian block.
     *
     * @return          EDGE_PRIOR_GYRO_STATUS_SUCCESS always.
     */
    [[nodiscard]] EdgePriorGyroStatus
        getHessian(Eigen::Matrix<double, 3, 3> &hessian_out)
    {
        linearizeOplus();
        hessian_out =
            _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
        return EdgePriorGyroStatus::EDGE_PRIOR_GYRO_STATUS_SUCCESS;
    }

    /*!
     * @brief           Prior gyroscope bias, radians per second.
     */
    const Eigen::Vector3d bprior;
};

/*!
 * @brief           g2o binary edge for pose-graph optimisation between two
 *                  VertexPose4DoF vertices (vertex 0 and vertex 1). The
 *                  six-value error is the rotation error followed by the
 *                  translation error against a measured relative transform.
 */
class Edge4DoF
    : public g2o::BaseBinaryEdge<6, Vector6d, VertexPose4DoF, VertexPose4DoF>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates the edge from a measured relative transform and
     *                  splits it into dRij and dtij.
     *
     * @param[in]       deltaT_in
     *                  Measured transform from the camera of vertex 1 to the
     *                  camera of vertex 0, homogeneous 4 by 4.
     */
    Edge4DoF(const Eigen::Matrix4d &deltaT_in)
    {
        dTij = deltaT_in;
        dRij = deltaT_in.block<3, 3>(0, 0);
        dtij = deltaT_in.block<3, 1>(0, 3);
    }

    /*!
     * @brief           Does nothing: reading this edge from a stream is not
     *                  supported.
     *
     * @param[in,out]   is_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    /*!
     * @brief           Does nothing: writing this edge to a stream is not
     *                  supported.
     *
     * @param[in,out]   os_inout
     *                  Unused.
     *
     * @return          false always.
     */
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    /*!
     * @brief           Sets the error to the rotation vector of the relative
     *                  camera rotation against dRij, followed by the position
     *                  of the second camera in the first camera frame minus
     *                  dtij. Failed logarithm maps are only logged.
     */
    void computeError()
    {
        const VertexPose4DoF *VPi =
            static_cast<const VertexPose4DoF *>(_vertices[0]);
        const VertexPose4DoF *VPj =
            static_cast<const VertexPose4DoF *>(_vertices[1]);
        Eigen::Vector3d rotationVector{};
        if (logSO3(VPi->estimate().Rcw[0] * VPj->estimate().Rcw[0].transpose() *
                       dRij.transpose(),
                   rotationVector) != G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: logSO3 returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        _error << rotationVector,
            VPi->estimate().Rcw[0] * (-VPj->estimate().Rcw[0].transpose() *
                                      VPj->estimate().tcw[0]) +
                VPi->estimate().tcw[0] - dtij;
    }

    // virtual void linearizeOplus(); // numerical implementation

    /*!
     * @brief           Measured relative transform from the camera of vertex 1
     *                  to the camera of vertex 0, homogeneous.
     */
    Eigen::Matrix4d dTij;
    /*!
     * @brief           Rotation part of dTij.
     */
    Eigen::Matrix3d dRij;
    /*!
     * @brief           Translation part of dTij, metres.
     */
    Eigen::Vector3d dtij;
};

} // namespace core
} // namespace vs_graphs

#endif // G2OTYPES_H
