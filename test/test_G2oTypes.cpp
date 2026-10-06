/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            test_G2oTypes.cpp
 *
 * @brief           G2oTypes regression tests: the camera-body rotations of an
 *                  ImuCamPose built from a two-camera frame, the periodic
 *                  rotation clean-up of ImuCamPose::update and updateW, the
 *                  symmetric information matrix of ConstraintPoseImu, and the
 *                  VertexPose write/read round trip.
 */

#include "G2oTypes.h"

#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "Frame.h"
#include "ImuTypes.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <sstream>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{

TEST(ImuCamPose, BothCamerasGetTheTransposedBodyRotation)
{
    camera_models::pinhole::Pinhole leftCamera(
        std::vector<float>{500.0F, 500.0F, 320.0F, 240.0F});
    camera_models::pinhole::Pinhole rightCamera(
        std::vector<float>{500.0F, 500.0F, 320.0F, 240.0F});

    // A stereo-inertial frame at the world origin whose IMU is rotated
    // 0.3 rad about z and offset 0.1 m along x from the left camera.
    Frame frame;
    frame.p_camera  = &leftCamera;
    frame.p_camera2 = &rightCamera;
    frame.mbf       = 0.0F;
    const Sophus::SE3f cameraToBody(
        Eigen::AngleAxisf(0.3F, Eigen::Vector3f::UnitZ()).toRotationMatrix(),
        Eigen::Vector3f(0.1F, 0.0F, 0.0F));
    frame.imuCalibration =
        IMU::Calib(cameraToBody, 1.0e-3F, 1.0e-2F, 1.0e-4F, 1.0e-3F);
    ASSERT_EQ(frame.setPose(Sophus::SE3f()), FrameStatus::FRAME_STATUS_SUCCESS);

    const ImuCamPose pose(&frame);
    ASSERT_EQ(pose.Rbc.size(), 2U);
    EXPECT_TRUE(pose.Rbc[0].isApprox(pose.Rcb[0].transpose()));
    EXPECT_TRUE(pose.Rbc[1].isApprox(pose.Rcb[1].transpose()));
}

/*!
 * @brief           Optimiser step that changes nothing: zero rotation vector
 *                  (radians) and zero translation (metres).
 */
const double ZERO_UPDATE_STEP[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/*!
 * @brief           Scale applied to a rotation matrix to stand in for the
 *                  rounding drift that many optimiser steps accumulate.
 */
constexpr double ROTATION_DRIFT_SCALE = 1.01;

/*!
 * @brief           Gives a pose one camera whose frame is the body frame,
 *                  with the body frame at the world origin, no camera model
 *                  and a zero update counter.
 *
 * @param[in,out]   pose_inout
 *                  Pose to set up; its camera models, reference rotation Rwb0
 *                  and accumulated rotation DR are not touched.
 */
void setUpSingleCameraPose(ImuCamPose &pose_inout)
{
    const std::vector<Eigen::Matrix3d> identityRotations(
        1,
        Eigen::Matrix3d::Identity());
    const std::vector<Eigen::Vector3d> zeroTranslations(
        1,
        Eigen::Vector3d::Zero());
    ASSERT_EQ(pose_inout.setParam(identityRotations,
                                  zeroTranslations,
                                  identityRotations,
                                  zeroTranslations,
                                  0.0),
              ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS);
    pose_inout.its = 0;
}

/*!
 * @brief           Checks that the third update replaces a drifted body
 *                  rotation by the closest rotation matrix.
 */
TEST(ImuCamPose, UpdateRestoresAnOrthonormalBodyRotationOnTheThirdCall)
{
    ImuCamPose pose;
    setUpSingleCameraPose(pose);
    const Eigen::Matrix3d bodyRotation_bodyToWorld =
        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.Rwb = ROTATION_DRIFT_SCALE * bodyRotation_bodyToWorld;

    for (int callIndex = 0; callIndex < 3; callIndex++)
    {
        ASSERT_EQ(pose.update(ZERO_UPDATE_STEP),
                  ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS);
    }

    EXPECT_TRUE((pose.Rwb * pose.Rwb.transpose())
                    .isApprox(Eigen::Matrix3d::Identity(), 1.0e-12));
    EXPECT_TRUE(pose.Rwb.isApprox(bodyRotation_bodyToWorld, 1.0e-12));
}

/*!
 * @brief           Checks that the fifth world-frame update replaces a drifted
 *                  accumulated yaw rotation by the closest rotation matrix.
 */
TEST(ImuCamPose, UpdateWRestoresAnOrthonormalYawRotationOnTheFifthCall)
{
    ImuCamPose pose;
    setUpSingleCameraPose(pose);
    pose.Rwb0 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d yawRotation_world =
        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.DR = ROTATION_DRIFT_SCALE * yawRotation_world;

    for (int callIndex = 0; callIndex < 5; callIndex++)
    {
        ASSERT_EQ(pose.updateW(ZERO_UPDATE_STEP),
                  ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS);
    }

    EXPECT_TRUE((pose.DR * pose.DR.transpose())
                    .isApprox(Eigen::Matrix3d::Identity(), 1.0e-12));
    EXPECT_TRUE(pose.DR.isApprox(yawRotation_world, 1.0e-12));
}

/*!
 * @brief           Checks that a prior built from an asymmetric information
 *                  matrix keeps the symmetric part of that matrix.
 */
TEST(ConstraintPoseImu, KeepsTheSymmetricPartOfTheInformationMatrix)
{
    /* Positive definite symmetric part; the asymmetry sits above the
     * diagonal, which the eigen solver alone would never read. */
    Matrix15d informationMatrix = 10.0 * Matrix15d::Identity();
    informationMatrix(0, 1)     = 2.0;

    const ConstraintPoseImu constraint(Eigen::Matrix3d::Identity(),
                                       Eigen::Vector3d::Zero(),
                                       Eigen::Vector3d::Zero(),
                                       Eigen::Vector3d::Zero(),
                                       Eigen::Vector3d::Zero(),
                                       informationMatrix);

    const Matrix15d symmetricPart =
        (informationMatrix + informationMatrix.transpose()) / 2.0;
    EXPECT_TRUE(constraint.H.isApprox(constraint.H.transpose(), 1.0e-12));
    EXPECT_TRUE(constraint.H.isApprox(symmetricPart, 1.0e-12));
}

/*!
 * @brief           Checks that reading what write produced restores the camera
 *                  poses, extrinsics, camera parameters and baseline-focal
 *                  product of a one-camera vertex.
 */
TEST(VertexPose, ReadRestoresWhatWriteProduced)
{
    camera_models::pinhole::Pinhole writtenCamera(
        std::vector<float>{500.0F, 501.0F, 320.0F, 240.0F});
    camera_models::pinhole::Pinhole readCamera(
        std::vector<float>{1.0F, 1.0F, 1.0F, 1.0F});

    const std::vector<Eigen::Matrix3d> cameraRotations_worldToCamera(
        1,
        Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix());
    const std::vector<Eigen::Vector3d> cameraTranslations_worldToCamera(
        1,
        Eigen::Vector3d(0.5, -0.25, 2.0));
    const std::vector<Eigen::Matrix3d> extrinsicRotations_cameraToBody(
        1,
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix());
    const std::vector<Eigen::Vector3d> extrinsicTranslations_cameraToBody(
        1,
        Eigen::Vector3d(0.1, 0.0, 0.05));
    const double baselineFocalProduct = 40.0;

    ImuCamPose writtenPose;
    ASSERT_EQ(writtenPose.setParam(cameraRotations_worldToCamera,
                                   cameraTranslations_worldToCamera,
                                   extrinsicRotations_cameraToBody,
                                   extrinsicTranslations_cameraToBody,
                                   baselineFocalProduct),
              ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS);
    writtenPose.pCamera = {&writtenCamera};
    VertexPose writtenVertex;
    writtenVertex.setEstimate(writtenPose);

    /* Seventeen digits make the text round trip exact. */
    std::stringstream poseStream;
    poseStream.precision(17);
    ASSERT_TRUE(writtenVertex.write(poseStream));

    ImuCamPose readPose;
    setUpSingleCameraPose(readPose);
    readPose.pCamera = {&readCamera};
    VertexPose readVertex;
    readVertex.setEstimate(readPose);
    EXPECT_TRUE(readVertex.read(poseStream));

    const ImuCamPose &restoredPose = readVertex.estimate();
    ASSERT_EQ(restoredPose.Rcw.size(), 1U);
    EXPECT_TRUE(restoredPose.Rcw[0].isApprox(cameraRotations_worldToCamera[0],
                                             1.0e-12));
    EXPECT_TRUE(
        restoredPose.tcw[0].isApprox(cameraTranslations_worldToCamera[0],
                                     1.0e-12));
    EXPECT_TRUE(restoredPose.Rbc[0].isApprox(extrinsicRotations_cameraToBody[0],
                                             1.0e-12));
    EXPECT_TRUE(
        restoredPose.tbc[0].isApprox(extrinsicTranslations_cameraToBody[0],
                                     1.0e-12));
    EXPECT_DOUBLE_EQ(restoredPose.bf, baselineFocalProduct);
    for (int parameterIndex = 0; parameterIndex < 4; parameterIndex++)
    {
        float writtenParameter{};
        float readParameter{};
        ASSERT_EQ(writtenCamera.getParameter(parameterIndex, writtenParameter),
                  camera_models::geometriccamera::GeometricCameraStatus::
                      GEOMETRIC_CAMERA_STATUS_SUCCESS);
        ASSERT_EQ(readCamera.getParameter(parameterIndex, readParameter),
                  camera_models::geometriccamera::GeometricCameraStatus::
                      GEOMETRIC_CAMERA_STATUS_SUCCESS);
        EXPECT_FLOAT_EQ(readParameter, writtenParameter);
    }
}

/*!
 * @brief           Checks that reading a stream that ends too early reports a
 *                  failure.
 */
TEST(VertexPose, ReadReportsATruncatedStream)
{
    camera_models::pinhole::Pinhole readCamera(
        std::vector<float>{1.0F, 1.0F, 1.0F, 1.0F});
    ImuCamPose readPose;
    setUpSingleCameraPose(readPose);
    readPose.pCamera = {&readCamera};
    VertexPose readVertex;
    readVertex.setEstimate(readPose);

    std::stringstream truncatedStream("1 0 0");
    EXPECT_FALSE(readVertex.read(truncatedStream));
}

} // namespace
} // namespace core
} // namespace vs_graphs
