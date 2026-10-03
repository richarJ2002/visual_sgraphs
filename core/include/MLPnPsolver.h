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
 * @file            MLPnPsolver.h
 *
 * @brief           Declares MLPnPsolver, which estimates a camera pose from 2-D
 *                  to 3-D matches with MLPnP inside RANSAC, for relocalisation.
 */

/******************************************************************************
 * Author:   Steffen Urban                                              *
 * Contact:  urbste@gmail.com                                          *
 * License:  Copyright (c) 2016 Steffen Urban, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

#ifndef VS_GRAPHS_CORE_MLPNPSOLVER_H
#define VS_GRAPHS_CORE_MLPNPSOLVER_H

#include "Frame.h"
#include "MLPnPsolverStatus.h"
#include "MapPoint.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
/*!
 * @brief           Estimates the camera pose of a frame from 2D-3D matches with
 *                  RANSAC and the MLPnP (maximum likelihood PnP) algorithm.
 *
 *                  The solver is built from one frame and the map points that
 *                  frame matched. Each call to iterate() runs more RANSAC
 *                  rounds; poses are transforms from the world frame to the
 *                  camera frame.
 */
class MLPnPsolver
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Collects the usable 2D-3D matches of a frame and sets
     *                  the default RANSAC parameters.
     *
     *                  Matches whose map point is null, bad, or has no
     *                  undistorted key point are skipped.
     *
     * @param[in]       frame_in
     *                  Frame whose key points are the 2D observations. Its
     *                  camera model is borrowed and must outlive the solver.
     *
     * @param[in]       mapPointMatches_in
     *                  Map point matched to each key point of the frame, by key
     *                  point index; null entries mean no match.
     */
    MLPnPsolver(const Frame                   &frame_in,
                const std::vector<MapPoint *> &mapPointMatches_in) :
        inlierCount(0),
        iterationCount(0),
        bestInlierCount(0),
        correspondenceCount(0),
        p_camera(frame_in.p_camera)
    {
        mapPointMatches = mapPointMatches_in;
        bearingVectors.reserve(frame_in.mapPoints.size());
        points2D.reserve(frame_in.mapPoints.size());
        sigmaSquared.reserve(frame_in.mapPoints.size());
        points3Dw.reserve(frame_in.mapPoints.size());
        keypointIndices.reserve(frame_in.mapPoints.size());
        allIndices.reserve(frame_in.mapPoints.size());

        int outputIndex = 0;
        for (size_t matchIndex = 0, matchCount = mapPointMatches.size();
             matchIndex < matchCount;
             matchIndex++)
        {
            MapPoint *p_mapPoint = mapPointMatches_in[matchIndex];

            if (p_mapPoint)
            {
                bool mapPointIsBad{};
                if (p_mapPoint->isBad(mapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!mapPointIsBad)
                {
                    if (matchIndex >= frame_in.keyPointsUndistorted.size())
                        continue;
                    const cv::KeyPoint &keyPoint =
                        frame_in.keyPointsUndistorted[matchIndex];

                    points2D.push_back(keyPoint.pt);
                    sigmaSquared.push_back(
                        frame_in.levelSigmaSquared[keyPoint.octave]);

                    // Bearing vector should be normalized
                    cv::Point3f bearingVectorCv =
                        p_camera->unproject(keyPoint.pt);
                    bearingVectorCv /= bearingVectorCv.z;
                    BearingVector bearingVector(bearingVectorCv.x,
                                                bearingVectorCv.y,
                                                bearingVectorCv.z);
                    bearingVectors.push_back(bearingVector);

                    // 3D coordinates
                    Eigen::Matrix<float, 3, 1> worldPositionEigen{};
                    if (p_mapPoint->getWorldPos(worldPositionEigen) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWorldPos returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    Point3 worldPosition(worldPositionEigen(0),
                                         worldPositionEigen(1),
                                         worldPositionEigen(2));
                    points3Dw.push_back(worldPosition);

                    keypointIndices.push_back(matchIndex);
                    allIndices.push_back(outputIndex);

                    outputIndex++;
                }
            }
        }

        if (setRansacParameters() !=
            MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setRansacParameters returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /*!
     * @brief           Sets the RANSAC parameters and adapts them to the number
     *                  of usable correspondences.
     *
     *                  The minimum inlier count is raised to at least epsilon
     *                  times the correspondence count and to at least the
     *                  minimum set size; the maximum iteration count is capped
     *                  by the count needed for the requested probability.
     *
     * @param[in]       probability_in
     *                  Desired probability of drawing one outlier-free sample.
     *
     * @param[in]       minimumInliers_in
     *                  Minimum inlier count to accept a pose.
     *
     * @param[in]       maximumIterations_in
     *                  Upper bound on RANSAC iterations.
     *
     * @param[in]       minimumSet_in
     *                  Number of correspondences drawn for each pose
     *                  hypothesis.
     *
     * @param[in]       epsilon_in
     *                  Expected fraction of inliers among all correspondences.
     *
     * @param[in]       threshold2_in
     *                  Squared-error threshold, in sigma-squared units, that
     *                  separates inliers from outliers.
     *
     * @return          MLPN_PSOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MLPnPsolverStatus
        setRansacParameters(double probability_in       = 0.99,
                            int    minimumInliers_in    = 8,
                            int    maximumIterations_in = 300,
                            int    minimumSet_in        = 6,
                            float  epsilon_in           = 0.4,
                            float  threshold2_in        = 5.991);

    // Find metod is necessary?

    /*!
     * @brief           Runs RANSAC rounds until a pose is found or the
     *                  iteration budget is spent.
     *
     * @param[in]       iterationCount_in
     *                  Minimum number of rounds to run in this call.
     *
     * @param[out]      areIterationsExhausted_out
     *                  True when the total iteration budget is used up, or when
     *                  there are fewer correspondences than the minimum
     *                  inliers.
     *
     * @param[out]      inliersFlags_out
     *                  One flag per entry of the constructor's match list, true
     *                  for inliers; empty when no pose was found.
     *
     * @param[out]      inlierCount_out
     *                  Number of inliers of the returned pose, 0 when none.
     *
     * @param[out]      Tout_out
     *                  Camera pose as a world-to-camera 4x4 transform; identity
     *                  when no pose was found.
     *
     * @param[out]      isSolved_out
     *                  True when a pose with enough inliers was found.
     *
     * @return          MLPN_PSOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MLPnPsolverStatus iterate(int   iterationCount_in,
                                            bool &areIterationsExhausted_out,
                                            std::vector<bool> &inliersFlags_out,
                                            int               &inlierCount_out,
                                            Eigen::Matrix4f   &Tout_out,
                                            bool              &isSolved_out);

    // Type definitions needed by the original code

    /*!
     * @brief           A 3-vector of unit length used to describe landmark
     *                  observations/bearings in camera frames (always expressed
     *                  in camera frames)
     */
    typedef Eigen::Vector3d BearingVector;

    /*!
     * @brief           An array of bearing-vectors
     */
    typedef std::vector<BearingVector, Eigen::aligned_allocator<BearingVector>>
        BearingVectors;

    /*!
     * @brief           A 2-matrix containing the 2D covariance information of a
     *                  bearing vector
     */
    typedef Eigen::Matrix2d Covariance2Matrix;

    /*!
     * @brief           A 3-matrix containing the 3D covariance information of a
     *                  bearing vector
     */
    typedef Eigen::Matrix3d Covariance3Matrix;

    /*!
     * @brief           An array of 3D covariance matrices
     */
    typedef std::vector<Covariance3Matrix,
                        Eigen::aligned_allocator<Covariance3Matrix>>
        Covariance3Matrices;

    /*!
     * @brief           A 3-vector describing a point in 3D-space
     */
    typedef Eigen::Vector3d Point3;

    /*!
     * @brief           An array of 3D-points
     */
    typedef std::vector<Point3, Eigen::aligned_allocator<Point3>> Points3;

    /*!
     * @brief           A homogeneous 3-vector describing a point in 3D-space
     */
    typedef Eigen::Vector4d Point4;

    /*!
     * @brief           An array of homogeneous 3D-points
     */
    typedef std::vector<Point4, Eigen::aligned_allocator<Point4>> Points4;

    /*!
     * @brief           A 3-vector containing the rodrigues parameters of a
     *                  rotation matrix
     */
    typedef Eigen::Vector3d RodriguesVector;

    /*!
     * @brief           A rotation matrix
     */
    typedef Eigen::Matrix3d RotationMatrix;

    /*!
     * @brief           A 3x4 transformation matrix containing rotation \f$
     *                  \mathbf{R} \f$ and
     *                   translation \f$ \mathbf{t} \f$ as follows: \f$ \left(
     *                   \begin{array}{cc} \mathbf{R} & \mathbf{t} \end{array}
     *                   \right)
     *                  \f$
     */
    typedef Eigen::Matrix<double, 3, 4> TransformationMatrix;

    /*!
     * @brief           A 3-vector describing a translation/camera position
     */
    typedef Eigen::Vector3d TranslationVector;

  private:
    /*!
     * @brief           Counts the correspondences that the current pose
     *                  estimate reprojects within their error threshold.
     *
     *                  Reads the current estimate (mRi, mti) and updates
     *                  inlierFlags and inlierCount.
     *
     * @return          MLPN_PSOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MLPnPsolverStatus checkInliers();

    /*!
     * @brief           Re-estimates the pose from the best inlier set and keeps
     *                  it when it still has enough inliers.
     *
     * @param[out]      isRefined_out
     *                  True when the refined pose has more inliers than the
     *                  minimum and was stored in mRefinedTcw.
     *
     * @return          MLPN_PSOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MLPnPsolverStatus refine(bool &isRefined_out);

    // Functions from de original MLPnP code

    /*!
     * @brief           Computes the camera pose from bearing vectors and the
     *                  matching 3D points.
     *
     *                  The linear MLPnP estimate is refined by Gauss-Newton. At
     *                  least six correspondences are required (asserted).
     *
     * @param[in]       f_in
     *                  Bearing vectors in the camera frame.
     *
     * @param[in]       p_in
     *                  3D points in the world frame, one per bearing vector.
     *
     * @param[in]       covMats_in
     *                  Bearing-vector covariances; used only when there is one
     *                  per selected correspondence.
     *
     * @param[in]       indices_in
     *                  Indices into f_in and p_in of the correspondences to
     *                  use.
     *
     * @param[out]      result_inout
     *                  Pose as a 3x4 world-to-camera transform, rotation then
     *                  translation; fully overwritten.
     *
     * @return          MLPN_PSOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MLPnPsolverStatus
        computePose(const BearingVectors      &f_in,
                    const Points3             &p_in,
                    const Covariance3Matrices &covMats_in,
                    const std::vector<int>    &indices_in,
                    TransformationMatrix      &result_inout);

    /*!
     * @brief           Refines a pose with at most five Gauss-Newton steps on
     *                  the MLPnP residuals.
     *
     *                  Stops early when a step is implausibly large, which
     *                  indicates a bad linear estimate, or when the residual
     *                  change falls below 1e-5.
     *
     * @param[in,out]   x_inout
     *                  Six-vector: Rodrigues rotation parameters, then
     *                  translation. Holds the initial guess and the result.
     *
     * @param[in]       points_in
     *                  3D points in the world frame.
     *
     * @param[in]       nullspaces_in
     *                  Two-column nullspace basis of each point's bearing
     *                  vector.
     *
     * @param[in]       Kll_in
     *                  Observation weight matrix, 2 rows per point.
     *
     * @param[in]       shouldUseCovariance_in
     *                  True to weight the normal equations by Kll_in.
     *
     * @return          MLPN_PSOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MLPnPsolverStatus
        mlpnp_gn(Eigen::VectorXd                    &x_inout,
                 const Points3                      &points_in,
                 const std::vector<Eigen::MatrixXd> &nullspaces_in,
                 const Eigen::SparseMatrix<double>   Kll_in,
                 bool                                shouldUseCovariance_in);

    /*!
     * @brief           Evaluates the MLPnP residuals and, optionally, their
     *                  Jacobian at a pose.
     *
     *                  Each point contributes two residuals: its transformed
     *                  unit direction projected on the two nullspace vectors of
     *                  its bearing vector.
     *
     * @param[in]       x_in
     *                  Six-vector: Rodrigues rotation parameters, then
     *                  translation.
     *
     * @param[in]       points_in
     *                  3D points in the world frame.
     *
     * @param[in]       nullspaces_in
     *                  Two-column nullspace basis of each point's bearing
     *                  vector.
     *
     * @param[out]      r_inout
     *                  Residuals, two per point; the caller sizes it and every
     *                  entry is overwritten.
     *
     * @param[out]      fjac_in
     *                  Jacobian, two rows by six columns per point; written
     *                  only when getJacs_in is true, and the caller sizes it.
     *
     * @param[in]       getJacs_in
     *                  True to also fill fjac_in.
     *
     * @return          MLPN_PSOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MLPnPsolverStatus mlpnp_residuals_and_jacs(
        const Eigen::VectorXd              &x_in,
        const Points3                      &points_in,
        const std::vector<Eigen::MatrixXd> &nullspaces_in,
        Eigen::VectorXd                    &r_inout,
        Eigen::MatrixXd                    &fjac_in,
        bool                                getJacs_in);

    /*!
     * @brief           Computes the 2x6 Jacobian of one point's two residuals
     *                  with respect to the rotation and translation.
     *
     * @param[in]       point_in
     *                  3D point in the world frame.
     *
     * @param[in]       nullspace_r
     *                  First nullspace vector of the point's bearing vector.
     *
     * @param[in]       nullspace_s_in
     *                  Second nullspace vector of the point's bearing vector.
     *
     * @param[in]       w_in
     *                  Rodrigues rotation parameters of the current pose.
     *
     * @param[in]       t_in
     *                  Translation of the current pose.
     *
     * @param[out]      jacs_in
     *                  2x6 Jacobian: columns 0-2 are the rotation parameters,
     *                  columns 3-5 the translation; the caller sizes it.
     *
     * @return          MLPN_PSOLVER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MLPnPsolverStatus
        mlpnpJacs(const Point3            &point_in,
                  const Eigen::Vector3d   &nullspace_r,
                  const Eigen::Vector3d   &nullspace_s_in,
                  const RodriguesVector   &w_in,
                  const TranslationVector &t_in,
                  Eigen::MatrixXd         &jacs_in);

    // Auxiliar methods

    /*!
     * @brief           Compute a rotation matrix from Rodrigues axis angle.
     *
     * @param[in]       omega_in
     *                  The Rodrigues-parameters of a rotation.
     *
     * @param[out]      rotation_out
     *                  The 3x3 rotation matrix.
     *
     * @return          MLPN_PSOLVER_STATUS_SUCCESS.
     */
    [[nodiscard]] MLPnPsolverStatus
        rodrigues2rot(const Eigen::Vector3d &omega_in,
                      Eigen::Matrix3d       &rotation_out);

    /*!
     * @brief           Compute the Rodrigues-parameters of a rotation matrix.
     *
     * @param[in]       R_in
     *                  The 3x3 rotation matrix.
     *
     * @param[out]      rodrigues_out
     *                  The Rodrigues-parameters.
     *
     * @return          MLPN_PSOLVER_STATUS_SUCCESS.
     */
    [[nodiscard]] MLPnPsolverStatus
        rot2rodrigues(const Eigen::Matrix3d &R_in,
                      Eigen::Vector3d       &rodrigues_out);

    //----------------------------------------------------
    // Fields of the solver
    //----------------------------------------------------
    /*!
     * @brief           Map point matched to each key point of the frame, by key
     *                  point index; null entries mean no match. Borrowed.
     */
    std::vector<MapPoint *> mapPointMatches;

    // 2D Points
    /*!
     * @brief           Undistorted key point positions of the usable
     *                  correspondences, in pixels.
     */
    std::vector<cv::Point2f> points2D;

    /*!
     * @brief           Bearing vector of each usable correspondence in the
     *                  camera frame, scaled so that z is 1.
     */
    BearingVectors bearingVectors;

    /*!
     * @brief           Squared measurement noise (sigma squared) of each usable
     *                  correspondence, taken from its key point's pyramid
     *                  level.
     */
    std::vector<float> sigmaSquared;

    // 3D Points
    /*!
     * @brief           World-frame position of the map point of each usable
     *                  correspondence, in metres.
     */
    Points3 points3Dw;

    // Index in Frame
    /*!
     * @brief           Key point index in the frame of each usable
     *                  correspondence.
     */
    std::vector<size_t> keypointIndices;

    // Current Estimation
    /*!
     * @brief           Rotation of the current pose estimate, world to camera.
     */
    double mRi[3][3];

    /*!
     * @brief           Translation of the current pose estimate, world to
     *                  camera.
     */
    double mti[3];

    /*!
     * @brief           Inlier flag of each correspondence under the current
     *                  pose estimate.
     */
    std::vector<bool> inlierFlags;

    /*!
     * @brief           Number of inliers under the current pose estimate.
     */
    int inlierCount;

    // Current Ransac State
    /*!
     * @brief           RANSAC iterations run so far, across all iterate()
     *                  calls.
     */
    int iterationCount;

    /*!
     * @brief           Inlier flags of the best pose found so far.
     */
    std::vector<bool> bestInlierFlags;

    /*!
     * @brief           Inlier count of the best pose found so far.
     */
    int bestInlierCount;

    /*!
     * @brief           Best pose found so far as a world-to-camera 4x4
     *                  transform.
     */
    Eigen::Matrix4f mBestTcw;

    // Refined
    /*!
     * @brief           Refined pose as a world-to-camera 4x4 transform.
     */
    Eigen::Matrix4f mRefinedTcw;

    /*!
     * @brief           Inlier flags of the refined pose.
     */
    std::vector<bool> refinedInlierFlags;

    /*!
     * @brief           Inlier count of the refined pose.
     */
    int refinedInlierCount;

    // Number of Correspondences
    /*!
     * @brief           Number of usable 2D-3D correspondences.
     */
    int correspondenceCount;

    // Indices for random selection [0 .. N-1]
    /*!
     * @brief           Correspondence indices 0 to correspondenceCount - 1, the
     *                  pool that RANSAC samples from.
     */
    std::vector<size_t> allIndices;

    // RANSAC probability
    /*!
     * @brief           Desired probability of drawing one outlier-free sample.
     */
    double ransacProb;

    // RANSAC min inliers
    /*!
     * @brief           Minimum inlier count to accept a pose, after adaptation
     *                  to the correspondence count.
     */
    int ransacMinInliers;

    // RANSAC max iterations
    /*!
     * @brief           Upper bound on RANSAC iterations, after adaptation.
     */
    int ransacMaxIterations;

    // RANSAC expected inliers/total ratio
    /*!
     * @brief           Expected fraction of inliers among all correspondences.
     */
    float ransacEpsilon;

    // RANSAC Minimun Set used at each iteration
    /*!
     * @brief           Number of correspondences drawn for each pose
     *                  hypothesis.
     */
    int ransacMinSet;

    // Max square error associated with scale level. Max error =
    // th*th*sigma(level)*sigma(level)
    /*!
     * @brief           Largest squared reprojection error, in pixels squared,
     *                  that still counts as an inlier, per correspondence.
     */
    std::vector<float> maxError;

    /*!
     * @brief           Camera model of the frame, used to project points.
     *                  Borrowed from the frame.
     */
    camera_models::geometriccamera::GeometricCamera *p_camera;
};

} // namespace core
} // namespace vs_graphs
#endif // VS_GRAPHS_CORE_MLPNPSOLVER_H
