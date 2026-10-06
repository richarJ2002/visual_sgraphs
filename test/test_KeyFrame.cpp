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
 * @file            test_KeyFrame.cpp
 *
 * @brief           KeyFrame regression tests on a key frame whose key points
 *                  have no matched map point: the scene median depth reports
 *                  the failure instead of reading an empty list, and the point
 *                  cloud can be released twice.
 */

#include "KeyFrame.h"

#include "Frame.h"
#include "Map.h"

#include <gtest/gtest.h>

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#include "Thirdparty/Sophus/sophus/se3.hpp"

namespace vs_graphs
{
namespace core
{
namespace
{

/*!
 * @brief           Number of key points of the test key frame.
 */
constexpr int KEY_POINT_COUNT = 3;

/*!
 * @brief           Fixture with a key frame at the world origin that has three
 *                  key points, no matched map point and a one-point colour
 *                  point cloud.
 */
class KeyFrameTest : public ::testing::Test
{
  protected:
    /*!
     * @brief           Map the key frame belongs to.
     */
    Map map{0};

    /*!
     * @brief           Frame the key frame is built from.
     */
    Frame frame;

    /*!
     * @brief           Key frame under test.
     */
    std::unique_ptr<KeyFrame> p_keyFrame;

    /*!
     * @brief           Builds the frame and the key frame.
     */
    void SetUp() override
    {
        frame.p_camera      = nullptr;
        frame.p_camera2     = nullptr;
        frame.keyPointCount = KEY_POINT_COUNT;
        frame.mapPoints     = std::vector<MapPoint *>(KEY_POINT_COUNT, nullptr);
        frame.pointClouds   = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        frame.pointClouds->push_back(pcl::PointXYZRGB());
        ASSERT_EQ(frame.setPose(Sophus::SE3f()),
                  FrameStatus::FRAME_STATUS_SUCCESS);
        p_keyFrame = std::make_unique<KeyFrame>(frame, &map, nullptr);
    }
};

/*!
 * @brief           Checks that the scene median depth of a key frame without
 *                  matched map points reports NO_MAP_POINTS and leaves the
 *                  output unchanged.
 */
TEST_F(KeyFrameTest, SceneMedianDepthReportsMissingMapPoints)
{
    float sceneMedianDepth_m = 7.0F;
    EXPECT_EQ(p_keyFrame->computeSceneMedianDepth(2, sceneMedianDepth_m),
              KeyFrameStatus::KEY_FRAME_STATUS_NO_MAP_POINTS);
    EXPECT_FLOAT_EQ(sceneMedianDepth_m, 7.0F);
}

/*!
 * @brief           Checks that a zero divisor is rejected as an invalid
 *                  argument and leaves the output unchanged.
 */
TEST_F(KeyFrameTest, SceneMedianDepthRejectsAZeroDivisor)
{
    float sceneMedianDepth_m = 7.0F;
    EXPECT_EQ(p_keyFrame->computeSceneMedianDepth(0, sceneMedianDepth_m),
              KeyFrameStatus::KEY_FRAME_STATUS_INVALID_ARGUMENT);
    EXPECT_FLOAT_EQ(sceneMedianDepth_m, 7.0F);
}

/*!
 * @brief           Checks that the point cloud can be released twice and is
 *                  null afterwards.
 */
TEST_F(KeyFrameTest, ClearPointCloudCanRunTwice)
{
    EXPECT_EQ(p_keyFrame->clearPointCloud(),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    EXPECT_EQ(p_keyFrame->clearPointCloud(),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pointCloud;
    ASSERT_EQ(p_keyFrame->getCurrentFramePointCloud(p_pointCloud),
              KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
    EXPECT_EQ(p_pointCloud, nullptr);
}

} // namespace
} // namespace core
} // namespace vs_graphs
