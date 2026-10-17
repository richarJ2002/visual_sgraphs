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
 * @file            ImuCamPoseClass.h
 *
 * @brief           Declares the ImuCamPose pose of one frame in the
 *                  visual-inertial optimisation.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_IMU_CAM_POSE_OBJECTS_IMU_CAM_POSE_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_IMU_CAM_POSE_OBJECTS_IMU_CAM_POSE_CLASS_H

#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "sophus/objects/se3.hpp"

#include "camera_models/geometric_camera/geometric_camera.h"

#include "g2o_types/imu_cam_pose/objects/ImuCamPoseStatusEnum.h"

#include "frame/frame.h"
#include "keyframe/keyframe.h"

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace imu_cam_pose
{

/*!
 * @brief        Pose of one frame in the visual-inertial optimisation.
 *               The optimised quantity is the IMU (body) pose in the
 *               world; the pose of every camera is derived from it.
 *               Holds borrowed pointers to the camera models and owns
 *               no heap object.
 */
class ImuCamPose
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief        Creates an empty pose with zeroed estimates.
     */
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
    ImuCamPose(keyframe::KeyFrame *p_keyFrame_inout) :
        its(0)
    {
        /* Load IMU pose */
        Eigen::Vector3f keyFrameImuPosition{};
        if (p_keyFrame_inout->getImuPosition(keyFrameImuPosition) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        twb = keyFrameImuPosition.cast<double>();
        Eigen::Matrix3f keyFrameImuRotation{};
        if (p_keyFrame_inout->getImuRotation(keyFrameImuRotation) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Rwb = keyFrameImuRotation.cast<double>();

        /* Load camera poses */
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

        /* Left camera */
        Eigen::Vector3f keyFrameTranslation{};
        if (p_keyFrame_inout->getTranslation(keyFrameTranslation) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTranslation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        tcw[0] = keyFrameTranslation.cast<double>();
        Eigen::Matrix3f keyFrameRotation{};
        if (p_keyFrame_inout->getRotation(keyFrameRotation) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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

        /* For posegraph 4DoF */
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
    ImuCamPose(frame::Frame *p_pF_inout) :
        its(0)
    {
        /* Load IMU pose */
        Eigen::Matrix<float, 3, 1> pFGetImuPosition{};
        if (p_pF_inout->getImuPosition(pFGetImuPosition) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        twb = pFGetImuPosition.cast<double>();
        Eigen::Matrix<float, 3, 3> pFImuRotation{};
        if (p_pF_inout->getImuRotation(pFImuRotation) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Rwb = pFImuRotation.cast<double>();

        /* Load camera poses */
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

        /* Left camera */
        Sophus::SE3<float> pFGetPose{};
        if (p_pF_inout->getPose(pFGetPose) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        tcw[0] = pFGetPose.translation().cast<double>();
        Sophus::SE3<float> pFGetPose2{};
        if (p_pF_inout->getPose(pFGetPose2) !=
            frame::FrameStatus::FRAME_STATUS_SUCCESS)
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
                frame::FrameStatus::FRAME_STATUS_SUCCESS)
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

        /* For posegraph 4DoF */
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
    ImuCamPose(Eigen::Matrix3d    &cameraRotation_cameraToWorld_inout,
               Eigen::Vector3d    &cameraTranslation_cameraToWorld_inout,
               keyframe::KeyFrame *p_keyFrame_inout) :
        its(0)
    {
        /* This is only for posegrpah, we do not care about multicamera */
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

        /* For posegraph 4DoF */
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
     *                  rotation before it is added. Every third call replaces
     *                  the body rotation by its closest rotation matrix, so
     *                  rounding drift does not accumulate.
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
     *                  rotation Rwb0. Every fifth call keeps only the yaw part
     *                  of DR and replaces DR by its closest rotation matrix;
     *                  the body rotation of that call still uses DR from
     *                  before the clean-up.
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
    /* For IMU */
    /*!
     * @brief           Rotation of the IMU (body) frame in the world frame;
     *                  maps body vectors to world vectors.
     */
    Eigen::Matrix3d                Rwb;
    /*!
     * @brief           Position of the IMU (body) origin in the world frame,
     *                  metres.
     */
    Eigen::Vector3d                twb;
    /* For set of cameras */
    /*!
     * @brief           Rotation from the world frame to each camera frame,
     *                  indexed by camera (0 left, 1 right).
     */
    std::vector<Eigen::Matrix3d>   Rcw;
    /*!
     * @brief           Translation from the world frame to each camera frame,
     *                  metres, indexed like Rcw.
     */
    std::vector<Eigen::Vector3d>   tcw;
    /*!
     * @brief           Rotation from the body frame to each camera frame,
     *                  indexed like Rcw.
     */
    std::vector<Eigen::Matrix3d>   Rcb;
    /*!
     * @brief           Rotation from each camera frame to the body frame,
     *                  indexed like Rcw.
     */
    std::vector<Eigen::Matrix3d>   Rbc;
    /*!
     * @brief           Position of the body origin in each camera frame,
     *                  metres, indexed like Rcw.
     */
    std::vector<Eigen::Vector3d>   tcb;
    /*!
     * @brief           Position of each camera origin in the body frame,
     *                  metres, indexed like Rcw.
     */
    std::vector<Eigen::Vector3d>   tbc;
    /*!
     * @brief           Stereo baseline times focal length, metres times pixels,
     *                  of the key frame or frame this pose was built from.
     */
    double                         bf;
    /*!
     * @brief           Camera model of each camera, indexed like Rcw. Borrowed
     *                  from the key frame or frame; never deleted here.
     */
    std::vector<camera_models::geometric_camera::GeometricCamera *> pCamera;
    /* For posegraph 4DoF */
    /*!
     * @brief           Body rotation when the pose was built; the fixed
     *                  reference of the 4DoF update, Rwb = DR * Rwb0.
     */
    Eigen::Matrix3d                                                 Rwb0;
    /*!
     * @brief           World-frame rotation accumulated by updateW on top of
     *                  Rwb0; identity until the first such update.
     */
    Eigen::Matrix3d                                                 DR;
    /*!
     * @brief           Number of update steps counted towards the periodic
     *                  rotation clean-up; reset to zero when the clean-up runs.
     */
    int                                                             its;
};

} /* namespace imu_cam_pose */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_IMU_CAM_POSE_OBJECTS_IMU_CAM_POSE_CLASS_H \
        */
