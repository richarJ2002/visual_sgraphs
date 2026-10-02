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
 * @brief           ImuCamPose built from a two-camera frame: each camera's
 *                  camera-to-body rotation (Rbc) is the transpose of its
 *                  body-to-camera rotation (Rcb).
 */

#include "G2oTypes.h"

#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "Frame.h"
#include "ImuTypes.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
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

} // namespace
} // namespace core
} // namespace vs_graphs
