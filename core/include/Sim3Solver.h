/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

/*!
 * @file            Sim3Solver.h
 *
 * @brief           Declares Sim3Solver, which estimates the similarity
 *                  transform (rotation, translation and scale) between two key
 *                  frames with RANSAC, for loop closing and map merging.
 */

#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/core.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Sim3SolverStatus.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Estimates the similarity transform between two keyframes
 *                  from matched map points with RANSAC and Horn's closed-form
 *                  method.
 *
 *                  Used to verify loop and merge candidates. The transform maps
 *                  points from the camera frame of the second keyframe into
 *                  that of the first, with a scale that is fixed to 1 for
 *                  stereo and RGB-D cameras.
 */
class Sim3Solver
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Collects the usable 3D-3D correspondences between two
     *                  keyframes and sets the default RANSAC parameters.
     *
     *                  Matches whose map points are missing, bad, or not
     *                  observed by their keyframe are skipped.
     *
     * @param[in]       p_keyFrame1_inout
     *                  First keyframe; shall be non-null. Borrowed; its camera
     *                  model must outlive the solver.
     *
     * @param[in]       p_keyFrame2_inout
     *                  Second keyframe; shall be non-null. Borrowed; its camera
     *                  model must outlive the solver.
     *
     * @param[in]       matched12_in
     *                  Map point matched to each key point of the first
     *                  keyframe, by index; null entries mean no match.
     *
     * @param[in]       isScaleFixed_in
     *                  True to fix the scale to 1 (stereo, RGB-D); false to
     *                  estimate it (monocular).
     *
     * @param[in]       keyFrameMatchedMapPoints_in
     *                  Keyframe that observes each entry of matched12_in; empty
     *                  means every match belongs to the second keyframe.
     */
    Sim3Solver(KeyFrame                      *p_keyFrame1_inout,
               KeyFrame                      *p_keyFrame2_inout,
               const std::vector<MapPoint *> &matched12_in,
               const bool                     isScaleFixed_in = true,
               std::vector<KeyFrame *>        keyFrameMatchedMapPoints_in =
                   std::vector<KeyFrame *>()) :
        iterationCount(0),
        bestInlierCount(0),
        isScaleFixed(isScaleFixed_in),
        p_firstCamera(p_keyFrame1_inout->p_camera),
        p_secondCamera(p_keyFrame2_inout->p_camera)
    {
        bool areKeyFramesDifferent = false;
        if (keyFrameMatchedMapPoints_in.empty())
        {
            areKeyFramesDifferent = true;
            keyFrameMatchedMapPoints_in =
                std::vector<KeyFrame *>(matched12_in.size(), p_keyFrame2_inout);
        }

        p_keyFrame1 = p_keyFrame1_inout;
        p_keyFrame2 = p_keyFrame2_inout;

        std::vector<MapPoint *> keyFrameMapPoint1{};
        if (p_keyFrame1_inout->getMapPointMatches(keyFrameMapPoint1) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointMatches returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        firstMatchCount = matched12_in.size();

        mapPoints1.reserve(firstMatchCount);
        mapPoints2.reserve(firstMatchCount);
        mapPointMatches12 = matched12_in;
        indices1.reserve(firstMatchCount);
        points3Dc1.reserve(firstMatchCount);
        points3Dc2.reserve(firstMatchCount);

        Eigen::Matrix3f cameraRotation_worldToCamera1{};
        if (p_keyFrame1_inout->getRotation(cameraRotation_worldToCamera1) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRotation returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f cameraTranslation_worldToCamera1{};
        if (p_keyFrame1_inout->getTranslation(
                cameraTranslation_worldToCamera1) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTranslation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix3f cameraRotation_worldToCamera2{};
        if (p_keyFrame2_inout->getRotation(cameraRotation_worldToCamera2) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRotation returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f cameraTranslation_worldToCamera2{};
        if (p_keyFrame2_inout->getTranslation(
                cameraTranslation_worldToCamera2) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTranslation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        allIndices.reserve(firstMatchCount);

        size_t idx = 0;

        KeyFrame *pKFm = p_keyFrame2_inout; // Default variable
        for (int i1 = 0; i1 < firstMatchCount; i1++)
        {
            if (matched12_in[i1])
            {
                MapPoint *p_mapPoint1 = keyFrameMapPoint1[i1];
                MapPoint *p_mapPoint2 = matched12_in[i1];

                if (!p_mapPoint1)
                    continue;

                bool mapPoint1IsBad{};
                if (p_mapPoint1->isBad(mapPoint1IsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                bool mapPoint2IsBad{};
                if (!(mapPoint1IsBad) &&
                    p_mapPoint2->isBad(mapPoint2IsBad) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (mapPoint1IsBad || mapPoint2IsBad)
                    continue;

                if (areKeyFramesDifferent)
                    pKFm = keyFrameMatchedMapPoints_in[i1];

                std::tuple<int, int> mapPoint1IndexInKeyFrame{};
                if (p_mapPoint1->getIndexInKeyFrame(p_keyFrame1_inout,
                                                    mapPoint1IndexInKeyFrame) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getIndexInKeyFrame returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                int indexKeyFrame1 = std::get<0>(mapPoint1IndexInKeyFrame);
                std::tuple<int, int> mapPoint2IndexInKeyFrame{};
                if (p_mapPoint2->getIndexInKeyFrame(pKFm,
                                                    mapPoint2IndexInKeyFrame) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getIndexInKeyFrame returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                int indexKeyFrame2 = std::get<0>(mapPoint2IndexInKeyFrame);

                if (indexKeyFrame1 < 0 || indexKeyFrame2 < 0)
                    continue;

                const cv::KeyPoint &keyPoint1 =
                    p_keyFrame1_inout->keyPointsUndistorted[indexKeyFrame1];
                const cv::KeyPoint &keyPoint2 =
                    pKFm->keyPointsUndistorted[indexKeyFrame2];

                const float sigmaSquare1 =
                    p_keyFrame1_inout->levelSigmaSquared[keyPoint1.octave];
                const float sigmaSquare2 =
                    pKFm->levelSigmaSquared[keyPoint2.octave];

                maxError1.push_back(9.210F * sigmaSquare1);
                maxError2.push_back(9.210F * sigmaSquare2);

                mapPoints1.push_back(p_mapPoint1);
                mapPoints2.push_back(p_mapPoint2);
                indices1.push_back(i1);

                Eigen::Vector3f X3D1w{};
                if (p_mapPoint1->getWorldPos(X3D1w) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWorldPos returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                points3Dc1.push_back(cameraRotation_worldToCamera1 * X3D1w +
                                     cameraTranslation_worldToCamera1);

                Eigen::Vector3f X3D2w{};
                if (p_mapPoint2->getWorldPos(X3D2w) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWorldPos returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                points3Dc2.push_back(cameraRotation_worldToCamera2 * X3D2w +
                                     cameraTranslation_worldToCamera2);

                allIndices.push_back(idx);
                idx++;
            }
        }

        if (fromCameraToImage(points3Dc1, points1im1, p_firstCamera) !=
            Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: fromCameraToImage returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (fromCameraToImage(points3Dc2, points2im2, p_secondCamera) !=
            Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: fromCameraToImage returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (setRansacParameters() !=
            Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRansacParameters returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /*!
     * @brief           Sets the RANSAC parameters, adapts the iteration limit
     *                  to the number of correspondences and restarts the
     *                  iteration count.
     *
     * @param[in]       probability_in
     *                  Desired probability of drawing one outlier-free sample.
     *
     * @param[in]       minimumInliers_in
     *                  Minimum inlier count to accept a transform.
     *
     * @param[in]       maximumIterations_in
     *                  Upper bound on RANSAC iterations.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus
        setRansacParameters(double probability_in       = 0.99,
                            int    minimumInliers_in    = 6,
                            int    maximumIterations_in = 300);

    /*!
     * @brief           Runs RANSAC until it converges or the iteration limit is
     *                  reached.
     *
     * @param[out]      inliers12Flags_inout
     *                  One flag per entry of the constructor's match list, true
     *                  for inliers of the returned transform.
     *
     * @param[out]      inlierCount_inout
     *                  Number of inliers.
     *
     * @param[out]      transform_out
     *                  4x4 similarity transform from the second keyframe's
     *                  camera frame to the first's; identity when none was
     *                  found.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus find(std::vector<bool> &inliers12Flags_inout,
                                        int               &inlierCount_inout,
                                        Eigen::Matrix4f   &transform_out);

    /*!
     * @brief           Runs at most a given number of RANSAC rounds and stops
     *                  as soon as a transform has more inliers than the
     *                  minimum.
     *
     * @param[in]       iterationCount_in
     *                  Maximum number of rounds to run in this call.
     *
     * @param[out]      areIterationsExhausted_out
     *                  True when the total iteration limit is used up, or when
     *                  there are fewer correspondences than the minimum
     *                  inliers.
     *
     * @param[out]      inliersFlags_out
     *                  One flag per entry of the constructor's match list, true
     *                  for inliers; all false when no transform was accepted.
     *
     * @param[out]      inlierCount_out
     *                  Number of inliers of the accepted transform, 0 when
     *                  none.
     *
     * @param[out]      transform_out
     *                  4x4 similarity transform from the second keyframe's
     *                  camera frame to the first's; identity when none was
     *                  accepted.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus iterate(int   iterationCount_in,
                                           bool &areIterationsExhausted_out,
                                           std::vector<bool> &inliersFlags_out,
                                           int               &inlierCount_out,
                                           Eigen::Matrix4f   &transform_out);

    /*!
     * @brief           Like the overload without convergence flag, but also
     *                  reports whether a transform was accepted and otherwise
     *                  returns the best candidate seen.
     *
     * @param[in]       iterationCount_in
     *                  Maximum number of rounds to run in this call.
     *
     * @param[out]      areIterationsExhausted_out
     *                  True when the total iteration limit is used up, or when
     *                  there are fewer correspondences than the minimum
     *                  inliers.
     *
     * @param[out]      inliersFlags_out
     *                  One flag per entry of the constructor's match list, true
     *                  for inliers; all false when no transform was accepted.
     *
     * @param[out]      inlierCount_out
     *                  Number of inliers of the accepted transform, 0 when
     *                  none.
     *
     * @param[out]      hasConverged_out
     *                  True when a transform with more inliers than the minimum
     *                  was found.
     *
     * @param[out]      transform_out
     *                  4x4 similarity transform from the second keyframe's
     *                  camera frame to the first's: the accepted one, else the
     *                  best candidate of this call, else the identity when
     *                  there are too few correspondences.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus iterate(int   iterationCount_in,
                                           bool &areIterationsExhausted_out,
                                           std::vector<bool> &inliersFlags_out,
                                           int               &inlierCount_out,
                                           bool              &hasConverged_out,
                                           Eigen::Matrix4f   &transform_out);

    /*!
     * @brief           Returns the best similarity transform found so far.
     *
     * @param[out]      estimatedTransformation_out
     *                  4x4 transform from the second keyframe's camera frame to
     *                  the first's, scale included; uninitialised before the
     *                  first RANSAC round.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus getEstimatedTransformation(
        Eigen::Matrix4f &estimatedTransformation_out);

    /*!
     * @brief           Returns the rotation of the best transform found so far.
     *
     * @param[out]      estimatedRotation_out
     *                  Rotation from the second keyframe's camera frame to the
     *                  first's; uninitialised before the first RANSAC round.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus
        getEstimatedRotation(Eigen::Matrix3f &estimatedRotation_out);

    /*!
     * @brief           Returns the translation of the best transform found so
     *                  far.
     *
     * @param[out]      estimatedTranslation_out
     *                  Translation, in the first keyframe's camera frame, of
     *                  the transform from the second keyframe's camera frame;
     *                  uninitialised before the first RANSAC round.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus
        getEstimatedTranslation(Eigen::Vector3f &estimatedTranslation_out);

    /*!
     * @brief           Returns the scale of the best transform found so far.
     *
     * @param[out]      estimatedScale_out
     *                  Scale factor from the second keyframe's camera frame to
     *                  the first's; 1 when the scale is fixed; uninitialised
     *                  before the first RANSAC round.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus
        getEstimatedScale(float &estimatedScale_out) const;

  protected:
    /*!
     * @brief           Computes the centroid of three points and the points
     *                  relative to it.
     *
     * @param[in]       P_in
     *                  Three 3D points, one per column.
     *
     * @param[out]      Pr_inout
     *                  Receives the points minus the centroid, one per column;
     *                  every column is overwritten.
     *
     * @param[out]      C_out
     *                  Centroid of the points.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus computeCentroid(Eigen::Matrix3f &P_in,
                                                   Eigen::Matrix3f &Pr_inout,
                                                   Eigen::Vector3f &C_out);

    /*!
     * @brief           Computes the similarity transform between two sets of
     *                  three points with Horn's closed-form method.
     *
     *                  The result (rotation, translation, scale and the 4x4
     *                  transform and its inverse) is stored in the
     *                  current-estimate members; the scale is 1 when it is
     *                  fixed. The transform maps the points of P2 onto those of
     *                  P1.
     *
     * @param[in]       P1_inout
     *                  Three 3D points in the first keyframe's camera frame,
     *                  one per column.
     *
     * @param[in]       P2_inout
     *                  The same three points in the second keyframe's camera
     *                  frame, in the same order, one per column.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus computeSim3(Eigen::Matrix3f &P1_inout,
                                               Eigen::Matrix3f &P2_inout);

    /*!
     * @brief           Counts the correspondences that the current transform
     *                  maps within the error threshold in both images.
     *
     *                  Updates inlierFlags and inlierCount.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus checkInliers();

    /*!
     * @brief           Projects 3D points into an image after moving them with
     *                  a 4x4 transform.
     *
     * @param[in]       vP3Dw_in
     *                  3D points in the source frame of the transform.
     *
     * @param[out]      points2D_out
     *                  Image points in pixels, one per input point; cleared
     *                  first.
     *
     * @param[in]       cameraPose_worldToCamera_in
     *                  Transform applied to the points before projection; its
     *                  rotation block includes any scale.
     *
     * @param[in,out]   p_camera_inout
     *                  Camera model that projects the points; shall be
     *                  non-null.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus project(
        const std::vector<Eigen::Vector3f> &vP3Dw_in,
        std::vector<Eigen::Vector2f>       &points2D_out,
        Eigen::Matrix4f                     cameraPose_worldToCamera_in,
        camera_models::geometriccamera::GeometricCamera *p_camera_inout);
    /*!
     * @brief           Projects points given in a camera frame into the image
     *                  of that camera.
     *
     * @param[in]       vP3Dc_in
     *                  3D points in the camera frame.
     *
     * @param[out]      points2D_out
     *                  Image points in pixels, one per input point; cleared
     *                  first.
     *
     * @param[in,out]   p_camera_inout
     *                  Camera model that projects the points; shall be
     *                  non-null.
     *
     * @return          SIM3_SOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] Sim3SolverStatus fromCameraToImage(
        const std::vector<Eigen::Vector3f>              &vP3Dc_in,
        std::vector<Eigen::Vector2f>                    &points2D_out,
        camera_models::geometriccamera::GeometricCamera *p_camera_inout);

  protected:
    // KeyFrames and matches
    /*!
     * @brief           First keyframe. Borrowed.
     */
    KeyFrame *p_keyFrame1;

    /*!
     * @brief           Second keyframe. Borrowed.
     */
    KeyFrame *p_keyFrame2;

    /*!
     * @brief           Position of each usable correspondence's map point in
     *                  the first keyframe's camera frame.
     */
    std::vector<Eigen::Vector3f> points3Dc1;

    /*!
     * @brief           Position of each usable correspondence's map point in
     *                  the second keyframe's camera frame.
     */
    std::vector<Eigen::Vector3f> points3Dc2;

    /*!
     * @brief           Map point of the first keyframe in each usable
     *                  correspondence. Borrowed.
     */
    std::vector<MapPoint *> mapPoints1;

    /*!
     * @brief           Matched map point of the second side in each usable
     *                  correspondence. Borrowed.
     */
    std::vector<MapPoint *> mapPoints2;

    /*!
     * @brief           Copy of the constructor's match list: map point matched
     *                  to each key point of the first keyframe.
     */
    std::vector<MapPoint *> mapPointMatches12;

    /*!
     * @brief           Index in the match list of each usable correspondence.
     */
    std::vector<size_t> indices1;

    /*!
     * @brief           Largest squared reprojection error, in pixels squared,
     *                  that counts as an inlier in the first image, per
     *                  correspondence: 9.210 (chi-square, two degrees of
     *                  freedom, 99 %) times the key point's level sigma
     *                  squared.
     */
    std::vector<float> maxError1;

    /*!
     * @brief           Same as maxError1 for the second image.
     */
    std::vector<float> maxError2;

    /*!
     * @brief           Number of usable 3D-3D correspondences.
     */
    int correspondenceCount;

    /*!
     * @brief           Length of the constructor's match list, including null
     *                  entries.
     */
    int firstMatchCount;

    // Current Estimation
    /*!
     * @brief           Rotation of the current estimate, camera 2 to camera 1.
     */
    Eigen::Matrix3f mR12i;

    /*!
     * @brief           Translation of the current estimate, in the camera 1
     *                  frame.
     */
    Eigen::Vector3f mt12i;

    /*!
     * @brief           Scale of the current estimate; 1 when the scale is
     *                  fixed.
     */
    float ms12i;

    /*!
     * @brief           Current estimate as a 4x4 similarity transform, camera 2
     *                  to camera 1.
     */
    Eigen::Matrix4f mT12i;

    /*!
     * @brief           Inverse of mT12i, camera 1 to camera 2.
     */
    Eigen::Matrix4f mT21i;

    /*!
     * @brief           Inlier flag of each correspondence under the current
     *                  estimate.
     */
    std::vector<bool> inlierFlags;

    /*!
     * @brief           Number of inliers under the current estimate.
     */
    int inlierCount;

    // Current Ransac State
    /*!
     * @brief           RANSAC iterations run so far, across all iterate()
     *                  calls.
     */
    int iterationCount;

    /*!
     * @brief           Inlier flags of the best estimate found so far.
     */
    std::vector<bool> bestInlierFlags;

    /*!
     * @brief           Inlier count of the best estimate found so far.
     */
    int bestInlierCount;

    /*!
     * @brief           Best estimate found so far as a 4x4 similarity
     *                  transform, camera 2 to camera 1.
     */
    Eigen::Matrix4f mBestT12;

    /*!
     * @brief           Rotation of the best estimate found so far.
     */
    Eigen::Matrix3f bestRotation;

    /*!
     * @brief           Translation of the best estimate found so far.
     */
    Eigen::Vector3f bestTranslation;

    /*!
     * @brief           Scale of the best estimate found so far.
     */
    float bestScale;

    // Scale is fixed to 1 in the stereo/RGBD case
    /*!
     * @brief           True when the scale is fixed to 1 (stereo, RGB-D).
     */
    bool isScaleFixed;

    // Indices for random selection
    /*!
     * @brief           Correspondence indices 0 to correspondenceCount - 1, the
     *                  pool that RANSAC samples from.
     */
    std::vector<size_t> allIndices;

    // Projections
    /*!
     * @brief           Image position, in pixels, of each correspondence's
     *                  point in the first image.
     */
    std::vector<Eigen::Vector2f> points1im1;

    /*!
     * @brief           Image position, in pixels, of each correspondence's
     *                  point in the second image.
     */
    std::vector<Eigen::Vector2f> points2im2;

    // RANSAC probability
    /*!
     * @brief           Desired probability of drawing one outlier-free sample.
     */
    double ransacProb;

    // RANSAC min inliers
    /*!
     * @brief           Minimum inlier count to accept a transform.
     */
    int ransacMinInliers;

    // RANSAC max iterations
    /*!
     * @brief           Upper bound on RANSAC iterations, after adaptation.
     */
    int ransacMaxIterations;

    // Calibration
    // cv::Mat mK1;
    // cv::Mat mK2;

    /*!
     * @brief           Camera model of the first keyframe. Borrowed.
     */
    camera_models::geometriccamera::GeometricCamera *p_firstCamera;

    /*!
     * @brief           Camera model of the second keyframe. Borrowed.
     */
    camera_models::geometriccamera::GeometricCamera *p_secondCamera;
};

} // namespace core
} // namespace vs_graphs

#endif // SIM3SOLVER_H
