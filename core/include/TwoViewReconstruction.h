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
 * @file            TwoViewReconstruction.h
 *
 * @brief           Declares TwoViewReconstruction, which recovers the relative
 *                  camera pose and the first 3-D points from two views
 *                  (monocular initialisation).
 */

#ifndef TwoViewReconstruction_H
#define TwoViewReconstruction_H

#include "TwoViewReconstructionStatus.h"
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <unordered_set>

#include <sophus/se3.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Recovers the relative camera motion and a sparse 3D
 *               structure from two views of the same scene, to initialise the
 *               monocular map.
 *
 *               Frame 1 is the reference frame and frame 2 the current frame. A
 *               homography and a fundamental matrix are estimated in parallel
 *               with RANSAC and the better-scoring model is decomposed into a
 *               pose and triangulated points.
 */
class TwoViewReconstruction
{
    /*!
     * @brief        One match: key point index in frame 1 and key point index
     *               in frame 2.
     */
    typedef std::pair<int, int> Match;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief        Creates a reconstructor for a fixed camera calibration.
     *
     * @param[in]    calibrationMatrix_in
     *               Camera intrinsic matrix K of both views, in pixels.
     * @param[in]    sigma_in
     *               Standard deviation of the key point position noise, in
     *               pixels.
     * @param[in]    maxIterationCount_in
     *               Number of RANSAC iterations for each model.
     */
    TwoViewReconstruction(const Eigen::Matrix3f &calibrationMatrix_in,
                          float                  sigma_in             = 1.0,
                          int                    maxIterationCount_in = 200)
    {
        calibrationMatrix = calibrationMatrix_in;

        this->sigma   = sigma_in;
        sigmaSquared  = sigma_in * sigma_in;
        maxIterations = maxIterationCount_in;
    }

    /*!
     * @brief        Estimates a homography and a fundamental matrix in
     *               parallel, picks the better model and tries to recover the
     *               motion and the structure from it.
     *
     *               The homography is used when it explains more than half of
     *               the combined score. Reconstruction fails when the best
     *               motion hypothesis has too few triangulated points, too
     *               little parallax, or no clear winner.
     *
     * @param[in]    keys1_in
     *               Key points of frame 1 (reference), undistorted, in pixels.
     * @param[in]    keys2_in
     *               Key points of frame 2 (current), undistorted, in pixels.
     * @param[in]    matches12_in
     *               For each key point of frame 1, the index of its matching
     *               key point in frame 2, or a negative value when unmatched.
     * @param[in,out] T21_inout
     *               Receives the pose of frame 2 relative to frame 1, mapping
     *               points from the camera 1 frame into the camera 2 frame,
     *               with unit-length translation; unchanged on failure.
     * @param[in,out] vP3D_inout
     *               Receives the triangulated points in the camera 1 frame,
     *               one per key point of frame 1; unchanged on failure.
     * @param[in,out] triangulatedFlags_inout
     *               Receives one flag per key point of frame 1, true when its
     *               point was triangulated with enough parallax; unchanged on
     *               failure.
     * @param[out]   isReconstructed_out
     *               True when a motion and structure were recovered.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always, also when
     *               the reconstruction failed.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        reconstruct(const std::vector<cv::KeyPoint> &keys1_in,
                    const std::vector<cv::KeyPoint> &keys2_in,
                    const std::vector<int>          &matches12_in,
                    Sophus::SE3f                    &T21_inout,
                    std::vector<cv::Point3f>        &vP3D_inout,
                    std::vector<bool>               &triangulatedFlags_inout,
                    bool                            &isReconstructed_out);

  private:
    /*!
     * @brief        Finds the homography from frame 1 to frame 2 with the
     *               highest RANSAC score over the stored match sets.
     *
     * @param[out]   matchesInliersFlags_out
     *               One flag per match, true for inliers of the best
     *               homography.
     * @param[out]   score_inout
     *               Score of the best homography; 0 when none scored.
     * @param[out]   H21_out
     *               Best homography, mapping pixels of frame 1 to frame 2.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        findHomography(std::vector<bool> &matchesInliersFlags_out,
                       float             &score_inout,
                       Eigen::Matrix3f   &H21_out);
    /*!
     * @brief        Finds the fundamental matrix from frame 1 to frame 2 with
     *               the highest RANSAC score over the stored match sets.
     *
     * @param[in,out] inliersFlags_inout
     *               Its incoming size sets the number of matches; overwritten
     *               with the inlier flags of the best fundamental matrix.
     * @param[out]   score_inout
     *               Score of the best fundamental matrix; 0 when none
     *               scored.
     * @param[out]   F21_out
     *               Best fundamental matrix, relating pixels of frame 1 to
     *               epipolar lines in frame 2.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        findFundamental(std::vector<bool> &inliersFlags_inout,
                        float             &score_inout,
                        Eigen::Matrix3f   &F21_out);

    /*!
     * @brief        Computes the homography from frame 1 to frame 2 from
     *               point pairs with the direct linear transform.
     *
     * @param[in]    points1_in
     *               Points of frame 1, normalised.
     * @param[in]    points2_in
     *               Matching points of frame 2, normalised, in the same
     *               order.
     * @param[out]   h21_out
     *               Homography mapping the first points onto the second.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        computeH21(const std::vector<cv::Point2f> &points1_in,
                   const std::vector<cv::Point2f> &points2_in,
                   Eigen::Matrix3f                &h21_out);
    /*!
     * @brief        Computes the fundamental matrix from frame 1 to frame 2
     *               from point pairs with the eight-point algorithm.
     *
     *               The result is forced to rank 2.
     *
     * @param[in]    points1_in
     *               Points of frame 1, normalised.
     * @param[in]    points2_in
     *               Matching points of frame 2, normalised, in the same
     *               order.
     * @param[out]   f21_out
     *               Fundamental matrix relating the first points to epipolar
     *               lines of the second.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        computeF21(const std::vector<cv::Point2f> &points1_in,
                   const std::vector<cv::Point2f> &points2_in,
                   Eigen::Matrix3f                &f21_out);

    /*!
     * @brief        Scores a homography by the symmetric transfer error of
     *               every match and flags the matches that fit it.
     *
     * @param[in]    H21_in
     *               Homography from frame 1 to frame 2.
     * @param[in]    H12_in
     *               Inverse of H21_in.
     * @param[out]   matchesInliersFlags_inout
     *               Resized to the number of matches; every flag is
     *               overwritten, true for inliers.
     * @param[in]    sigma_in
     *               Standard deviation of the key point noise, in pixels.
     * @param[out]   score_out
     *               Sum over the inlier errors of how far each is below the
     *               chi-square threshold; higher is better.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        checkHomography(const Eigen::Matrix3f &H21_in,
                        const Eigen::Matrix3f &H12_in,
                        std::vector<bool>     &matchesInliersFlags_inout,
                        float                  sigma_in,
                        float                 &score_out);

    /*!
     * @brief        Scores a fundamental matrix by the point-to-epipolar-line
     *               distance of every match and flags the matches that fit
     *               it.
     *
     * @param[in]    F21_in
     *               Fundamental matrix from frame 1 to frame 2.
     * @param[out]   matchesInliersFlags_inout
     *               Resized to the number of matches; every flag is
     *               overwritten, true for inliers.
     * @param[in]    sigma_in
     *               Standard deviation of the key point noise, in pixels.
     * @param[out]   score_out
     *               Sum over the inlier errors of how far each is below the
     *               score threshold; higher is better.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        checkFundamental(const Eigen::Matrix3f &F21_in,
                         std::vector<bool>     &matchesInliersFlags_inout,
                         float                  sigma_in,
                         float                 &score_out);

    /*!
     * @brief        Recovers the motion and structure from a fundamental
     *               matrix by testing the four hypotheses of its essential
     *               matrix.
     *
     *               Succeeds only when one hypothesis clearly triangulates the
     *               most points (at least 90 percent of the inliers and the
     *               required minimum) with enough parallax.
     *
     * @param[in]    matchesInliersFlags_inout
     *               Inlier flag of each match; read only.
     * @param[in]    F21_in
     *               Fundamental matrix from frame 1 to frame 2.
     * @param[in]    K_in
     *               Camera intrinsic matrix, in pixels.
     * @param[out]   T21_out
     *               Pose of frame 2 relative to frame 1, mapping points from
     *               the camera 1 frame into the camera 2 frame; written only
     *               on success.
     * @param[out]   vP3D_out
     *               Triangulated points in the camera 1 frame, one per key
     *               point of frame 1; written only on success.
     * @param[out]   triangulatedFlags_out
     *               One flag per key point of frame 1, true when its point is
     *               valid and has enough parallax; written only on success.
     * @param[in]    minimumParallax_in
     *               Minimum parallax of the winning hypothesis, in degrees.
     * @param[in]    minimumTriangulated_in
     *               Minimum number of triangulated points.
     * @param[out]   isReconstructed_out
     *               True when a hypothesis was accepted.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        reconstructF(std::vector<bool>        &matchesInliersFlags_inout,
                     Eigen::Matrix3f          &F21_in,
                     Eigen::Matrix3f          &K_in,
                     Sophus::SE3f             &T21_out,
                     std::vector<cv::Point3f> &vP3D_out,
                     std::vector<bool>        &triangulatedFlags_out,
                     float                     minimumParallax_in,
                     int                       minimumTriangulated_in,
                     bool                     &isReconstructed_out);

    /*!
     * @brief        Recovers the motion and structure from a homography by
     *               testing the eight hypotheses of the plane decomposition.
     *
     *               Fails when the singular values of the homography are almost
     *               equal, when the best hypothesis is not clearly better than
     *               the second best, or when it has too few points or too
     *               little parallax.
     *
     * @param[in]    matchesInliersFlags_inout
     *               Inlier flag of each match; read only.
     * @param[in]    H21_in
     *               Homography from frame 1 to frame 2.
     * @param[in]    K_in
     *               Camera intrinsic matrix, in pixels.
     * @param[out]   T21_out
     *               Pose of frame 2 relative to frame 1, mapping points from
     *               the camera 1 frame into the camera 2 frame; written only
     *               on success.
     * @param[in,out] vP3D_inout
     *               Triangulated points in the camera 1 frame, one per key
     *               point of frame 1; written only on success.
     * @param[out]   triangulatedFlags_out
     *               One flag per key point of frame 1, true when its point is
     *               valid and has enough parallax; written only on success.
     * @param[in]    minimumParallax_in
     *               Minimum parallax of the winning hypothesis, in degrees.
     * @param[in]    minimumTriangulated_in
     *               Minimum number of triangulated points.
     * @param[out]   isReconstructed_out
     *               True when a hypothesis was accepted.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        reconstructH(std::vector<bool>        &matchesInliersFlags_inout,
                     Eigen::Matrix3f          &H21_in,
                     Eigen::Matrix3f          &K_in,
                     Sophus::SE3f             &T21_out,
                     std::vector<cv::Point3f> &vP3D_inout,
                     std::vector<bool>        &triangulatedFlags_out,
                     float                     minimumParallax_in,
                     int                       minimumTriangulated_in,
                     bool                     &isReconstructed_out);

    /*!
     * @brief        Shifts key points to zero mean and scales them to unit
     *               mean absolute deviation, for a well-conditioned
     *               estimation.
     *
     * @param[in]    keys_in
     *               Key points, in pixels.
     * @param[out]   normalizedPoints_inout
     *               Resized to one point per key point and overwritten with
     *               the normalised coordinates.
     * @param[out]   T_out
     *               Transform that maps pixel coordinates to normalised ones.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        normalize(const std::vector<cv::KeyPoint> &keys_in,
                  std::vector<cv::Point2f>        &normalizedPoints_inout,
                  Eigen::Matrix3f                 &T_out);

    /*!
     * @brief        Triangulates the inlier matches for one motion hypothesis
     *               and counts the points that are valid.
     *
     *               A point is good when it lies in front of both cameras
     *               (unless the parallax is tiny) and reprojects within the
     *               threshold in both images.
     *
     * @param[in]    R_in
     *               Rotation of the hypothesis, camera 1 to camera 2.
     * @param[in]    t_in
     *               Translation of the hypothesis, camera 1 to camera 2.
     * @param[in]    keys1_in
     *               Key points of frame 1, in pixels.
     * @param[in]    keys2_in
     *               Key points of frame 2, in pixels.
     * @param[in]    matches12_in
     *               Matches as key point index pairs, frame 1 to frame 2.
     * @param[in]    matchesInliersFlags_in
     *               Inlier flag of each match; only inliers are triangulated.
     * @param[in]    K_in
     *               Camera intrinsic matrix, in pixels.
     * @param[in,out] vP3D_inout
     *               Resized to one point per key point of frame 1; the points
     *               that pass the checks are written in the camera 1 frame.
     * @param[in]    threshold2_in
     *               Largest squared reprojection error, in pixels squared.
     * @param[out]   goodFlags_out
     *               One flag per key point of frame 1, true for good points
     *               with enough parallax.
     * @param[out]   parallax_out
     *               Parallax angle, in degrees, of the 50th point in order of
     *               decreasing parallax (the last one when fewer exist); 0
     *               when no point is good.
     * @param[out]   goodPointCount_out
     *               Number of good points.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        checkRT(const Eigen::Matrix3f           &R_in,
                const Eigen::Vector3f           &t_in,
                const std::vector<cv::KeyPoint> &keys1_in,
                const std::vector<cv::KeyPoint> &keys2_in,
                const std::vector<Match>        &matches12_in,
                std::vector<bool>               &matchesInliersFlags_in,
                const Eigen::Matrix3f           &K_in,
                std::vector<cv::Point3f>        &vP3D_inout,
                float                            threshold2_in,
                std::vector<bool>               &goodFlags_out,
                float                           &parallax_out,
                int                             &goodPointCount_out);

    /*!
     * @brief        Splits an essential matrix into its two possible rotations
     *               and its translation direction.
     *
     *               The four motion hypotheses are the two rotations combined
     *               with the translation and its negation.
     *
     * @param[in]    E_in
     *               Essential matrix from frame 1 to frame 2.
     * @param[out]   R1_out
     *               First rotation hypothesis.
     * @param[out]   R2_out
     *               Second rotation hypothesis.
     * @param[out]   t_out
     *               Translation direction as a unit vector, known up to sign.
     *
     * @return       TWO_VIEW_RECONSTRUCTION_STATUS_SUCCESS always.
     */
    [[nodiscard]] TwoViewReconstructionStatus
        decomposeE(const Eigen::Matrix3f &E_in,
                   Eigen::Matrix3f       &R1_out,
                   Eigen::Matrix3f       &R2_out,
                   Eigen::Vector3f       &t_out);

    // Keypoints from Reference Frame (Frame 1)
    /*!
     * @brief        Key points of frame 1 (reference), in pixels, copied by
     *               reconstruct().
     */
    std::vector<cv::KeyPoint> keys1;

    // Keypoints from Current Frame (Frame 2)
    /*!
     * @brief        Key points of frame 2 (current), in pixels, copied by
     *               reconstruct().
     */
    std::vector<cv::KeyPoint> keys2;

    // Current Matches from Reference to Current
    /*!
     * @brief        Matches of the current reconstruct() call as key point
     *               index pairs, frame 1 to frame 2.
     */
    std::vector<Match> matches12;

    /*!
     * @brief        True for each key point of frame 1 that has a match.
     *               Written by reconstruct() but never read.
     */
    std::vector<bool> matchedFlags1;

    // Calibration
    /*!
     * @brief        Camera intrinsic matrix K of both views, in pixels.
     */
    Eigen::Matrix3f calibrationMatrix;

    // Standard Deviation and Variance
    /*!
     * @brief        Standard deviation of the key point position noise, in
     *               pixels.
     */
    float sigma;

    /*!
     * @brief        Variance of the key point position noise, in pixels
     *               squared.
     */
    float sigmaSquared;

    // Ransac max iterations
    /*!
     * @brief        Number of RANSAC iterations for each model.
     */
    int maxIterations;

    // Ransac sets
    /*!
     * @brief        Random sets of eight match indices, one set per RANSAC
     *               iteration, drawn by reconstruct() and shared by the
     *               homography and fundamental searches.
     */
    std::vector<std::vector<size_t>> sets;
};

} // namespace core
} // namespace vs_graphs

#endif // TwoViewReconstruction_H
