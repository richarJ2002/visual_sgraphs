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
 * @file            test_Sim3Solver.cpp
 *
 * @brief           Sim3Solver regression tests on two synthetic key frames at
 *                  the world origin: the inlier thresholds keep their
 *                  fractional part, the convergence overload of iterate
 *                  returns the identity when it runs no round, and a pure
 *                  shift gives the identity rotation instead of aborting.
 */

#include "Sim3Solver.h"

#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstddef>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "Thirdparty/Sophus/sophus/se3.hpp"

namespace vs_graphs
{
namespace core
{
namespace
{

/*!
 * @brief           Focal length of the synthetic pinhole camera, pixels.
 */
constexpr float FOCAL_LENGTH_PIXELS = 500.0F;

/*!
 * @brief           Principal point of the synthetic pinhole camera, pixels.
 */
constexpr float PRINCIPAL_POINT_X_PIXELS = 320.0F;
constexpr float PRINCIPAL_POINT_Y_PIXELS = 240.0F;

/*!
 * @brief           Number of matched map points between the two key frames.
 */
constexpr int CORRESPONDENCE_COUNT = 7;

/*!
 * @brief           Depth of every synthetic point in front of both cameras,
 *                  metres.
 */
constexpr float POINT_DEPTH_M = 5.0F;

/*!
 * @brief           Position (x, y) of each synthetic point in the world frame,
 *                  metres; no three are collinear.
 */
constexpr float POINT_POSITIONS_XY_M[CORRESPONDENCE_COUNT][2] = {{-0.6F, -0.4F},
                                                                 {0.0F, -0.5F},
                                                                 {0.6F, -0.3F},
                                                                 {-0.5F, 0.2F},
                                                                 {0.1F, 0.1F},
                                                                 {0.5F, 0.4F},
                                                                 {-0.1F, 0.5F}};

/*!
 * @brief           Shift along x, metres, of the last point as seen by the
 *                  second key frame: 3.02 pixels at the point depth, a squared
 *                  error of 9.12 pixels squared, between the truncated
 *                  threshold 9 and the true threshold 9.21.
 */
constexpr float BORDERLINE_SHIFT_M =
    3.02F * POINT_DEPTH_M / FOCAL_LENGTH_PIXELS;

/*!
 * @brief           Sim3Solver that runs its inlier check under the identity
 *                  transform, so a test can read the inlier flags directly.
 */
class Sim3SolverProbe : public Sim3Solver
{
  public:
    using Sim3Solver::Sim3Solver;

    /*!
     * @brief           Sets the current estimate to the identity and counts the
     *                  inliers under it.
     *
     * @param[out]      inlierFlags_out
     *                  Inlier flag of each usable correspondence.
     *
     * @return          The status of checkInliers.
     */
    Sim3SolverStatus checkInliersOfIdentity(std::vector<bool> &inlierFlags_out)
    {
        mT12i                               = Eigen::Matrix4f::Identity();
        mT21i                               = Eigen::Matrix4f::Identity();
        const Sim3SolverStatus inlierStatus = checkInliers();
        inlierFlags_out                     = inlierFlags;
        return inlierStatus;
    }
};

/*!
 * @brief           Fixture with two key frames at the world origin that share
 *                  one pinhole camera, each with one level-0 key point per
 *                  correspondence and no map point yet.
 */
class Sim3SolverTest : public ::testing::Test
{
  protected:
    /*!
     * @brief           Map the key frames and map points refer to.
     */
    Map map{0};

    /*!
     * @brief           Camera model shared by both key frames.
     */
    camera_models::pinhole::Pinhole camera{
        std::vector<float>{FOCAL_LENGTH_PIXELS,
                           FOCAL_LENGTH_PIXELS,
                           PRINCIPAL_POINT_X_PIXELS,
                           PRINCIPAL_POINT_Y_PIXELS}};

    /*!
     * @brief           Frames the key frames are built from.
     */
    Frame firstFrame;
    Frame secondFrame;

    /*!
     * @brief           Key frames whose map points the solver matches.
     */
    std::unique_ptr<KeyFrame> p_firstKeyFrame;
    std::unique_ptr<KeyFrame> p_secondKeyFrame;

    /*!
     * @brief           Map points of both key frames, owned by the fixture.
     */
    std::vector<std::unique_ptr<MapPoint>> ownedMapPoints;

    /*!
     * @brief           Second key frame's map point matched to each key point
     *                  of the first key frame.
     */
    std::vector<MapPoint *> matchedMapPoints;

    /*!
     * @brief           Builds both frames and their key frames.
     */
    void SetUp() override
    {
        for (Frame *p_frame : {&firstFrame, &secondFrame})
        {
            p_frame->p_camera      = &camera;
            p_frame->p_camera2     = nullptr;
            p_frame->keyPointCount = CORRESPONDENCE_COUNT;
            p_frame->keyPointsUndistorted =
                std::vector<cv::KeyPoint>(CORRESPONDENCE_COUNT,
                                          cv::KeyPoint(0.0F, 0.0F, 1.0F));
            p_frame->uRight = std::vector<float>(CORRESPONDENCE_COUNT, -1.0F);
            p_frame->mapPoints =
                std::vector<MapPoint *>(CORRESPONDENCE_COUNT, nullptr);
            p_frame->levelSigmaSquared    = {1.0F};
            p_frame->invLevelSigmaSquared = {1.0F};
            ASSERT_EQ(p_frame->setPose(Sophus::SE3f()),
                      FrameStatus::FRAME_STATUS_SUCCESS);
        }
        p_firstKeyFrame = std::make_unique<KeyFrame>(firstFrame, &map, nullptr);
        p_secondKeyFrame =
            std::make_unique<KeyFrame>(secondFrame, &map, nullptr);
        matchedMapPoints.assign(CORRESPONDENCE_COUNT, nullptr);
    }

    /*!
     * @brief           Matches key point pointIndex_in of both key frames
     *                  through one map point per key frame.
     *
     * @param[in]       pointIndex_in
     *                  Key point index in both key frames.
     *
     * @param[in]       firstPosition_world_in
     *                  Point position seen by the first key frame, world frame,
     *                  metres.
     *
     * @param[in]       secondPosition_world_in
     *                  Point position seen by the second key frame, world
     *                  frame, metres.
     */
    void addCorrespondence(int                    pointIndex_in,
                           const Eigen::Vector3f &firstPosition_world_in,
                           const Eigen::Vector3f &secondPosition_world_in)
    {
        ownedMapPoints.push_back(
            std::make_unique<MapPoint>(firstPosition_world_in,
                                       p_firstKeyFrame.get(),
                                       &map));
        MapPoint *p_firstMapPoint = ownedMapPoints.back().get();
        ownedMapPoints.push_back(
            std::make_unique<MapPoint>(secondPosition_world_in,
                                       p_secondKeyFrame.get(),
                                       &map));
        MapPoint *p_secondMapPoint = ownedMapPoints.back().get();

        ASSERT_EQ(p_firstMapPoint->addObservation(p_firstKeyFrame.get(),
                                                  pointIndex_in),
                  MapPointStatus::MAP_POINT_STATUS_SUCCESS);
        ASSERT_EQ(p_firstKeyFrame->addMapPoint(p_firstMapPoint, pointIndex_in),
                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
        ASSERT_EQ(p_secondMapPoint->addObservation(p_secondKeyFrame.get(),
                                                   pointIndex_in),
                  MapPointStatus::MAP_POINT_STATUS_SUCCESS);
        ASSERT_EQ(
            p_secondKeyFrame->addMapPoint(p_secondMapPoint, pointIndex_in),
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS);
        matchedMapPoints[pointIndex_in] = p_secondMapPoint;
    }

    /*!
     * @brief           Returns the first key frame's position of a synthetic
     *                  point.
     *
     * @param[in]       pointIndex_in
     *                  Row of POINT_POSITIONS_XY_M.
     *
     * @return          Point in the world frame, metres.
     */
    static Eigen::Vector3f getPointPosition_world(int pointIndex_in)
    {
        return Eigen::Vector3f(POINT_POSITIONS_XY_M[pointIndex_in][0],
                               POINT_POSITIONS_XY_M[pointIndex_in][1],
                               POINT_DEPTH_M);
    }
};

/*!
 * @brief           Checks that a correspondence whose squared reprojection
 *                  error lies between 9 and 9.21 pixels squared counts as an
 *                  inlier.
 */
TEST_F(Sim3SolverTest, BorderlineReprojectionErrorIsAnInlier)
{
    for (int pointIndex = 0; pointIndex < CORRESPONDENCE_COUNT; pointIndex++)
    {
        const Eigen::Vector3f firstPosition_world =
            getPointPosition_world(pointIndex);
        const bool isBorderline = pointIndex == CORRESPONDENCE_COUNT - 1;
        const Eigen::Vector3f secondPosition_world =
            firstPosition_world +
            Eigen::Vector3f(isBorderline ? BORDERLINE_SHIFT_M : 0.0F,
                            0.0F,
                            0.0F);
        addCorrespondence(pointIndex,
                          firstPosition_world,
                          secondPosition_world);
    }

    Sim3SolverProbe   solver(p_firstKeyFrame.get(),
                           p_secondKeyFrame.get(),
                           matchedMapPoints,
                           true,
                           std::vector<KeyFrame *>());
    std::vector<bool> inlierFlags;
    ASSERT_EQ(solver.checkInliersOfIdentity(inlierFlags),
              Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS);

    ASSERT_EQ(inlierFlags.size(),
              static_cast<std::size_t>(CORRESPONDENCE_COUNT));
    for (int pointIndex = 0; pointIndex < CORRESPONDENCE_COUNT; pointIndex++)
        EXPECT_TRUE(inlierFlags[pointIndex]) << "point " << pointIndex;
}

/*!
 * @brief           Checks that the convergence overload of iterate, called
 *                  after the iteration limit is used up, reports no transform
 *                  and returns the identity.
 */
TEST_F(Sim3SolverTest, IterateAfterTheLastRoundReturnsTheIdentity)
{
    /*!
     * The second key frame sees every point 1 m further along -x and turned
     * by 0.05 rad about y: p2 = R (p1 - s), so the transform from its camera
     * to the first one is p1 = R^T p2 + s, a 1 m shift along x. A rotation
     * of exactly zero has its own test (PureShiftGivesTheIdentityRotation).
     */
    const Eigen::Vector3f secondCameraShift_world(1.0F, 0.0F, 0.0F);
    const Eigen::Matrix3f secondCameraRotation_firstCameraToSecondCamera =
        Eigen::AngleAxisf(0.05F, Eigen::Vector3f::UnitY()).toRotationMatrix();
    for (int pointIndex = 0; pointIndex < CORRESPONDENCE_COUNT; pointIndex++)
    {
        const Eigen::Vector3f firstPosition_world =
            getPointPosition_world(pointIndex);
        addCorrespondence(pointIndex,
                          firstPosition_world,
                          secondCameraRotation_firstCameraToSecondCamera *
                              (firstPosition_world - secondCameraShift_world));
    }

    Sim3Solver solver(p_firstKeyFrame.get(),
                      p_secondKeyFrame.get(),
                      matchedMapPoints,
                      true,
                      std::vector<KeyFrame *>());
    /* One round in total; convergence needs more inliers than exist. */
    ASSERT_EQ(solver.setRansacParameters(0.99, CORRESPONDENCE_COUNT, 1),
              Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS);

    bool              areIterationsExhausted = false;
    std::vector<bool> inliersFlags;
    int               inlierCount  = -1;
    bool              hasConverged = true;
    Eigen::Matrix4f   lastRoundTransform{};
    ASSERT_EQ(solver.iterate(5,
                             areIterationsExhausted,
                             inliersFlags,
                             inlierCount,
                             hasConverged,
                             lastRoundTransform),
              Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS);
    ASSERT_TRUE(areIterationsExhausted);
    ASSERT_FALSE(hasConverged);
    /* Parentheses keep the comma in block<3, 1> out of the macro. */
    ASSERT_TRUE(
        (lastRoundTransform.block<3, 1>(0, 3).isApprox(secondCameraShift_world,
                                                       1.0e-3F)));

    Eigen::Matrix4f transform{};
    ASSERT_EQ(solver.iterate(5,
                             areIterationsExhausted,
                             inliersFlags,
                             inlierCount,
                             hasConverged,
                             transform),
              Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS);
    EXPECT_TRUE(areIterationsExhausted);
    EXPECT_FALSE(hasConverged);
    EXPECT_EQ(inlierCount, 0);
    EXPECT_EQ(inliersFlags, std::vector<bool>(CORRESPONDENCE_COUNT, false));
    EXPECT_TRUE(transform.isIdentity());
}

/*!
 * @brief           Checks that correspondences that differ by a pure 1 m shift
 *                  give the identity rotation and the 1 m translation instead
 *                  of aborting the process.
 */
TEST_F(Sim3SolverTest, PureShiftGivesTheIdentityRotation)
{
    /*!
     * The second key frame sees every point 1 m closer along the optical axis
     * z: p2 = p1 - s, so the transform from its camera to the first one is
     * p1 = p2 + s, no rotation and a 1 m shift along z. Along z both point
     * sets keep the same x and y and a constant depth, so their centred
     * coordinates are equal to the bit and the best rotation is exactly the
     * identity: the zero-norm case of computeSim3.
     */
    const Eigen::Vector3f secondCameraShift_world(0.0F, 0.0F, 1.0F);
    for (int pointIndex = 0; pointIndex < CORRESPONDENCE_COUNT; pointIndex++)
    {
        const Eigen::Vector3f firstPosition_world =
            getPointPosition_world(pointIndex);
        addCorrespondence(pointIndex,
                          firstPosition_world,
                          firstPosition_world - secondCameraShift_world);
    }

    Sim3Solver solver(p_firstKeyFrame.get(),
                      p_secondKeyFrame.get(),
                      matchedMapPoints,
                      true,
                      std::vector<KeyFrame *>());

    bool              areIterationsExhausted = true;
    std::vector<bool> inliersFlags;
    int               inlierCount  = 0;
    bool              hasConverged = false;
    Eigen::Matrix4f   transform{};
    ASSERT_EQ(solver.iterate(5,
                             areIterationsExhausted,
                             inliersFlags,
                             inlierCount,
                             hasConverged,
                             transform),
              Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS);
    ASSERT_TRUE(hasConverged);
    EXPECT_EQ(inlierCount, CORRESPONDENCE_COUNT);
    /* Parentheses keep the commas in block<3, 3> out of the macro. */
    EXPECT_TRUE((transform.block<3, 3>(0, 0).isIdentity()));
    EXPECT_TRUE((transform.block<3, 1>(0, 3).isApprox(secondCameraShift_world,
                                                      1.0e-6F)));
}

} // namespace
} // namespace core
} // namespace vs_graphs
