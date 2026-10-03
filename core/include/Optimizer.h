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
 * @file            Optimizer.h
 *
 * @brief           Declares Optimizer, the bundle-adjustment and pose-graph
 *                  optimisations built on g2o.
 */

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Frame.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"
#include "OptimizerStatus.h"

#include <boost/bind.hpp>
#include <math.h>

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include <atomic>

namespace vs_graphs
{
namespace core
{
class LoopClosing;

/*!
 * @brief           Static collection of the g2o-based optimisations of the SLAM
 *                  system: bundle adjustment, pose refinement, pose-graph
 *                  optimisation, similarity estimation and inertial
 *                  initialisation.
 *
 *                  Poses of keyframes and frames are camera poses from the
 *                  world frame to the camera frame unless a function says
 *                  otherwise.
 */
class Optimizer
{
  public:
    [[nodiscard]] static OptimizerStatus bundleAdjustment(
        const std::vector<vs_graphs::core::KeyFrame *>         &keyFrames_in,
        const std::vector<vs_graphs::core::MapPoint *>         &mapPoints_in,
        const std::vector<vs_graphs::core::semantic::Marker *> &markers_in,
        const std::vector<vs_graphs::core::geometric::Plane *> &planes_in,
        const std::vector<vs_graphs::core::semantic::Room *>   &rooms_in,
        int                     iterationCount_in  = 5,
        bool                   *p_stopFlag_inout   = nullptr,
        const unsigned long     loopKeyFrameId_in  = 0,
        const bool              useRobustKernel_in = true,
        const std::atomic_bool *p_stopRequested_in = nullptr);

    /*!
     * @brief           Runs bundle adjustment over every keyframe, map point,
     *                  marker, plane and room of a map.
     *
     *                  Delegates to bundleAdjustment() with the map's full
     *                  contents; see it for how results are applied or
     *                  deferred.
     *
     * @param[in]       p_map_in
     *                  Map to optimise; shall be non-null, borrowed.
     *
     * @param[in]       iterationCount_in
     *                  Number of optimiser iterations.
     *
     * @param[in,out]   p_stopFlag_inout
     *                  Optional flag that, when set to true, makes the
     *                  optimiser stop early; may be null.
     *
     * @param[in]       loopKeyFrameId_in
     *                  Id of the loop keyframe that triggered this run; results
     *                  are applied directly when it equals the id of the map's
     *                  origin keyframe, otherwise stored for the caller to
     *                  apply.
     *
     * @param[in]       useRobustKernel_in
     *                  True to use a robust (Huber) cost on the reprojection
     *                  edges.
     *
     * @param[in]       p_stopRequested_in
     *                  Optional thread-safe cancellation request, read between
     *                  iterations and copied into p_stopFlag_inout; may be
     *                  null.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus globalBundleAdjustment(
        Map                    *p_map_in,
        int                     iterationCount_in  = 5,
        bool                   *p_stopFlag_inout   = nullptr,
        const unsigned long     loopKeyFrameId_in  = 0,
        const bool              useRobustKernel_in = true,
        const std::atomic_bool *p_stopRequested_in = nullptr);

    /*!
     * @brief           Runs visual-inertial bundle adjustment over every
     *                  keyframe and map point of a map.
     *
     *                  Updates keyframe poses, velocities, IMU biases and map
     *                  point positions. With loopKeyFrameId_in equal to 0 the
     *                  results are written straight into the map; otherwise
     *                  they are stored in the keyframes' and map points'
     *                  global-BA fields for the loop closing thread to apply.
     *                  Returns without optimising when p_stopFlag_inout is
     *                  already set.
     *
     * @param[in,out]   p_map_inout
     *                  Map to optimise; shall be non-null. Its change index is
     *                  increased.
     *
     * @param[in]       iterationCount_in
     *                  Number of optimiser iterations.
     *
     * @param[in]       fixLocalKeyFrames_in
     *                  True to hold the keyframes of the latest local window
     *                  fixed; nothing is optimised when fewer than three
     *                  keyframes remain free.
     *
     * @param[in]       loopKeyFrameId_in
     *                  Id of the loop keyframe that triggered this run, or 0 to
     *                  apply the results directly.
     *
     * @param[in,out]   p_stopFlag_inout
     *                  Optional flag that, when set to true, makes the
     *                  optimiser stop early; may be null.
     *
     * @param[in]       isImuInitialization_in
     *                  True during IMU initialisation: one shared pair of
     *                  biases is estimated with priors instead of one pair per
     *                  keyframe.
     *
     * @param[in]       gyroBiasPriorWeight_in
     *                  Information weight of the zero prior on the gyroscope
     *                  bias; used only when isImuInitialization_in is true.
     *
     * @param[in]       accelBiasPriorWeight_in
     *                  Information weight of the zero prior on the
     *                  accelerometer bias; used only when
     *                  isImuInitialization_in is true.
     *
     * @param[in]       p_singularValues_in
     *                  Not used.
     *
     * @param[in]       p_hessianComputed_in
     *                  Not used.
     *
     * @param[in]       p_stopRequested_in
     *                  Optional thread-safe cancellation request, copied into
     *                  p_stopFlag_inout between iterations; may be null.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always, also when nothing was
     *                  optimised.
     */
    [[nodiscard]] static OptimizerStatus
        fullInertialBA(Map                    *p_map_inout,
                       int                     iterationCount_in,
                       const bool              fixLocalKeyFrames_in   = false,
                       const unsigned long     loopKeyFrameId_in      = 0,
                       bool                   *p_stopFlag_inout       = nullptr,
                       bool                    isImuInitialization_in = false,
                       float                   gyroBiasPriorWeight_in = 1e2,
                       float                   accelBiasPriorWeight_in = 1e6,
                       Eigen::VectorXd        *p_singularValues_in  = nullptr,
                       bool                   *p_hessianComputed_in = nullptr,
                       const std::atomic_bool *p_stopRequested_in   = nullptr);

    /*!
     * @brief           Refines the keyframes covisible with a keyframe, the map
     *                  points they see and the semantic entities attached to
     *                  them.
     *
     *                  Covisible keyframes are the free variables; keyframes
     *                  that see the same points but are not covisible stay
     *                  fixed. Outlier observations are erased from the map.
     *                  Does nothing when no keyframe is fixed, when no edge was
     *                  created, or when the stop flag is already set.
     *
     * @param[in,out]   p_keyFrame_inout
     *                  Newest keyframe; the centre of the local window. Shall
     *                  be non-null.
     *
     * @param[in]       p_pbStopFlag_in
     *                  Optional flag that, when true, aborts the optimisation;
     *                  may be null.
     *
     * @param[in,out]   p_map_inout
     *                  Map holding the keyframe; shall be non-null.
     *
     * @param[out]      fixedKeyFrameCount_inout
     *                  Number of fixed keyframes in the problem; set to 0 on
     *                  entry, then overwritten.
     *
     * @param[out]      optKeyFrameCount_out
     *                  Number of optimised keyframes.
     *
     * @param[out]      mapPointCount_out
     *                  Number of map points in the problem.
     *
     * @param[out]      edgeCount_out
     *                  Number of reprojection edges.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always, also when nothing was
     *                  optimised.
     */
    [[nodiscard]] static OptimizerStatus
        localBundleAdjustment(KeyFrame *p_keyFrame_inout,
                              bool     *p_pbStopFlag_in,
                              Map      *p_map_inout,
                              int      &fixedKeyFrameCount_inout,
                              int      &optKeyFrameCount_out,
                              int      &mapPointCount_out,
                              int      &edgeCount_out);

    /*!
     * @brief           Local Bundle Adjustment for loop closure detection
     *
     * @param[in]       p_mainKeyFrame_in
     *                  Main KeyFrame
     *
     * @param[in]       adjustKeyFrames_in
     *                  Non-fixed KeyFrames to adjust
     *
     * @param[in]       fixedKeyFrames_in
     *                  Fixed KeyFrames to set
     *
     * @param[in]       p_pbStopFlag_in
     *                  Flag to forcely stop the optimization
     */
    [[nodiscard]] static OptimizerStatus loopClosureLocalBundleAdjustment(
        KeyFrame               *p_mainKeyFrame_in,
        std::vector<KeyFrame *> adjustKeyFrames_in,
        std::vector<KeyFrame *> fixedKeyFrames_in,
        bool                   *p_pbStopFlag_in);

    /*!
     * @brief           Refines the camera pose of a frame by minimising the
     *                  reprojection error of its matched map points, which stay
     *                  fixed.
     *
     *                  Marks the observations that remain outliers in the
     *                  frame's outlier flags and stores the optimised pose in
     *                  the frame. The pose is left unchanged when fewer than
     *                  three correspondences exist.
     *
     * @param[in,out]   p_frame_inout
     *                  Frame to refine; shall be non-null.
     *
     * @param[out]      inlierCount_out
     *                  Number of correspondences kept as inliers, 0 when there
     *                  were fewer than three.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus poseOptimization(Frame *p_frame_inout,
                                                          int &inlierCount_out);

    /*!
     * @brief           Refines a frame's pose, velocity and IMU biases from its
     *                  map point matches and the IMU preintegration since the
     *                  last keyframe, which stays fixed.
     *
     *                  Stores the result and a prior built by marginalising the
     *                  keyframe states in the frame, and marks outlier
     *                  observations.
     *
     * @param[in,out]   p_frame_inout
     *                  Frame to refine; shall be non-null and have a last
     *                  keyframe and an IMU preintegration.
     *
     * @param[out]      inlierCount_out
     *                  Number of correspondences kept as inliers.
     *
     * @param[in]       isRecentlyInitialized_in
     *                  True right after IMU initialisation: skips the recovery
     *                  of near-miss observations when inliers are few.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus poseInertialOptimizationLastKeyFrame(
        Frame *p_frame_inout,
        int   &inlierCount_out,
        bool   isRecentlyInitialized_in = false);

    /*!
     * @brief           Refines a frame's pose, velocity and IMU biases together
     *                  with those of the previous frame, using the IMU
     *                  preintegration between the two frames.
     *
     *                  Stores the result and a prior built by marginalising the
     *                  previous frame's states in the frame, releases the
     *                  previous frame's prior, and marks outlier observations.
     *
     * @param[in,out]   p_frame_inout
     *                  Frame to refine; shall be non-null and have a previous
     *                  frame and an IMU preintegration.
     *
     * @param[out]      inlierCount_out
     *                  Number of correspondences kept as inliers.
     *
     * @param[in]       isRecentlyInitialized_in
     *                  True right after IMU initialisation: skips the recovery
     *                  of near-miss observations when inliers are few.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus poseInertialOptimizationLastFrame(
        Frame *p_frame_inout,
        int   &inlierCount_out,
        bool   isRecentlyInitialized_in = false);

    /*!
     * @brief           Corrects all keyframe poses and map points of a map
     *                  after a loop closure by optimising the essential graph
     *                  (spanning tree, loop edges and strong covisibility
     *                  edges).
     *
     *                  With isScaleFixed_in true the problem is 6 degrees of
     *                  freedom per keyframe (stereo, RGB-D); otherwise it is 7
     *                  (monocular, with scale). Keyframe poses and map point
     *                  positions are written back to the map.
     *
     * @param[in,out]   p_map_inout
     *                  Map to correct; shall be non-null.
     *
     * @param[in]       p_loopKeyFrame_in
     *                  Keyframe the loop was detected against.
     *
     * @param[in]       p_currentKeyFrame_in
     *                  Keyframe that closed the loop.
     *
     * @param[in]       NonCorrectedSim3_in
     *                  Similarity poses, world to camera, of the keyframes
     *                  before the loop correction.
     *
     * @param[in]       CorrectedSim3_in
     *                  Similarity poses, world to camera, of the keyframes
     *                  corrected by the loop; they initialise the optimiser.
     *
     * @param[in]       loopConnections_in
     *                  New edges created by the loop: for each keyframe, the
     *                  keyframes it is now connected to.
     *
     * @param[in]       isScaleFixed_in
     *                  True to keep the scale of every keyframe fixed.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus optimizeEssentialGraph(
        Map                                              *p_map_inout,
        KeyFrame                                         *p_loopKeyFrame_in,
        KeyFrame                                         *p_currentKeyFrame_in,
        const LoopClosing::KeyFrameAndPose               &NonCorrectedSim3_in,
        const LoopClosing::KeyFrameAndPose               &CorrectedSim3_in,
        const std::map<KeyFrame *, std::set<KeyFrame *>> &loopConnections_in,
        const bool                                       &isScaleFixed_in);

    /*!
     * @brief           Optimize the Essential Graph when a loop closure is
     *                  detected
     *
     * @param[in]       p_currentKeyFrame_in
     *                  Current KeyFrame
     *
     * @param[in]       p_sourceMap_in
     *                  Source map whose semantic graph follows the optimized
     *                  keyframe deformation.
     *
     * @param[in]       fixedKeyFrames_in
     *                  Fixed KeyFrames
     *
     * @param[in]       fixedCorrectedKeyFrames_in
     *                  Corrected Fixed KeyFrames
     *
     * @param[in]       nonFixedKeyFrames_in
     *                  Non-Fixed KeyFrames
     *
     * @param[in]       nonCorrectedMapPoints_in
     *                  Non-Corrected MapPoints
     *
     * @param[in]       mergeTransform_mergeWorldToCurrentWorld_in
     *                  Baseline similarity transform used when a semantic
     *                  object has no valid reference keyframe.
     */
    [[nodiscard]] static OptimizerStatus optimizeEssentialGraph(
        vs_graphs::core::KeyFrame                *p_currentKeyFrame_in,
        vs_graphs::core::Map                     *p_sourceMap_in,
        std::vector<vs_graphs::core::KeyFrame *> &fixedKeyFrames_in,
        std::vector<vs_graphs::core::KeyFrame *> &fixedCorrectedKeyFrames_in,
        std::vector<vs_graphs::core::KeyFrame *> &nonFixedKeyFrames_in,
        std::vector<vs_graphs::core::MapPoint *> &nonCorrectedMapPoints_in,
        const g2o::Sim3 &mergeTransform_mergeWorldToCurrentWorld_in);

    /*!
     * @brief           Corrects keyframe poses and map points after a loop
     *                  closure in an inertial system by optimising the
     *                  essential graph with only four degrees of freedom per
     *                  keyframe (yaw and translation).
     *
     *                  The loop keyframe is held fixed. Keyframe poses and map
     *                  point positions are written back to the map.
     *
     * @param[in,out]   p_map_inout
     *                  Map to correct; shall be non-null.
     *
     * @param[in]       p_loopKeyFrame_in
     *                  Keyframe the loop was detected against.
     *
     * @param[in]       p_currentKeyFrame_in
     *                  Keyframe that closed the loop.
     *
     * @param[in]       NonCorrectedSim3_in
     *                  Similarity poses, world to camera, of the keyframes
     *                  before the loop correction.
     *
     * @param[in]       CorrectedSim3_in
     *                  Similarity poses, world to camera, of the keyframes
     *                  corrected by the loop; they initialise the optimiser.
     *
     * @param[in]       loopConnections_in
     *                  New edges created by the loop: for each keyframe, the
     *                  keyframes it is now connected to.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus optimizeEssentialGraph4DoF(
        Map                                              *p_map_inout,
        KeyFrame                                         *p_loopKeyFrame_in,
        KeyFrame                                         *p_currentKeyFrame_in,
        const LoopClosing::KeyFrameAndPose               &NonCorrectedSim3_in,
        const LoopClosing::KeyFrameAndPose               &CorrectedSim3_in,
        const std::map<KeyFrame *, std::set<KeyFrame *>> &loopConnections_in);

    /*!
     * @brief           Refines the similarity transform between two keyframes
     *                  by minimising the reprojection error of their matched
     *                  map points in both images.
     *
     *                  Matches that remain outliers are cleared in
     *                  matches1_inout. Nothing is refined, and 0 inliers are
     *                  reported, when fewer than ten matches survive the first
     *                  round.
     *
     * @param[in]       p_keyFrame1_in
     *                  First keyframe; shall be non-null.
     *
     * @param[in]       p_keyFrame2_in
     *                  Second keyframe; shall be non-null.
     *
     * @param[in,out]   matches1_inout
     *                  Map point of the second keyframe matched to each
     *                  key point of the first, by index; null entries mean no
     *                  match. Outlier matches are set to null.
     *
     * @param[in,out]   g2oS12_inout
     *                  Similarity transform that maps points from the camera
     *                  frame of the second keyframe into that of the first.
     *                  Initial guess on entry, refined value on exit.
     *
     * @param[in]       threshold2_in
     *                  Squared reprojection error, in pixels squared, above
     *                  which a match is an outlier; its square root is the
     *                  Huber threshold.
     *
     * @param[in]       isScaleFixed_in
     *                  True to keep the scale fixed (SE3, stereo or RGB-D);
     *                  false to estimate it (Sim3, monocular).
     *
     * @param[out]      acumHessian_out
     *                  7x7 matrix set to zero when the refinement runs; it is
     *                  never accumulated and is left untouched otherwise.
     *
     * @param[out]      inlierCount_out
     *                  Number of matches that remain inliers.
     *
     * @param[in]       shouldUseAllPoints_in
     *                  True to also use matches whose point is not an
     *                  observation of the second keyframe.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus
        optimizeSim3(KeyFrame                    *p_keyFrame1_in,
                     KeyFrame                    *p_keyFrame2_in,
                     std::vector<MapPoint *>     &matches1_inout,
                     g2o::Sim3                   &g2oS12_inout,
                     const float                  threshold2_in,
                     const bool                   isScaleFixed_in,
                     Eigen::Matrix<double, 7, 7> &acumHessian_out,
                     int                         &inlierCount_out,
                     const bool shouldUseAllPoints_in = false);

    // For inertial systems

    /*!
     * @brief           Refines a window of recent keyframes with visual and IMU
     *                  constraints: poses, velocities, IMU biases and the map
     *                  points they observe.
     *
     *                  Older keyframes that see the same points stay fixed.
     *                  Outlier observations are erased from the map. The result
     *                  is discarded, and nothing is written, when the error
     *                  rises to more than twice its start value (small window
     *                  only) or becomes NaN.
     *
     * @param[in,out]   p_keyFrame_inout
     *                  Newest keyframe; the end of the window. Shall be
     *                  non-null.
     *
     * @param[in]       p_pbStopFlag_in
     *                  Optional flag that, when true, aborts the optimisation;
     *                  may be null.
     *
     * @param[in,out]   p_map_inout
     *                  Map holding the keyframe; shall be non-null.
     *
     * @param[out]      fixedKeyFrameCount_out
     *                  Number of fixed keyframes.
     *
     * @param[out]      optKeyFrameCount_out
     *                  Number of optimised keyframes.
     *
     * @param[out]      mapPointCount_out
     *                  Number of map points in the problem.
     *
     * @param[out]      edgeCount_out
     *                  Number of reprojection edges.
     *
     * @param[in]       isLargeWindow_in
     *                  True for the larger window (up to 25 keyframes, 4
     *                  iterations) used after IMU initialisation; false for 10
     *                  keyframes and 10 iterations.
     *
     * @param[in]       isRecentlyInitialized_in
     *                  True right after IMU initialisation: every inertial edge
     *                  gets a Huber kernel, not only the window's last link
     *                  (which is always also down-weighted).
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always, also when nothing was
     *                  written.
     */
    [[nodiscard]] static OptimizerStatus
        localInertialBA(KeyFrame *p_keyFrame_inout,
                        bool     *p_pbStopFlag_in,
                        Map      *p_map_inout,
                        int      &fixedKeyFrameCount_out,
                        int      &optKeyFrameCount_out,
                        int      &mapPointCount_out,
                        int      &edgeCount_out,
                        bool      isLargeWindow_in         = false,
                        bool      isRecentlyInitialized_in = false);
    /*!
     * @brief           Refines the keyframes around a map merge with visual and
     *                  IMU constraints: a window of the current map's newest
     *                  keyframes and a window around the merge keyframe in the
     *                  other map.
     *
     *                  Updates poses, velocities, IMU biases and map point
     *                  positions, and records each corrected keyframe pose.
     *                  Does nothing when the stop flag is already set.
     *
     * @param[in,out]   p_currentKeyFrame_inout
     *                  Keyframe of the current map that closed the merge; shall
     *                  be non-null.
     *
     * @param[in,out]   p_mergeKeyFrame_inout
     *                  Keyframe of the other map it is merged with; shall be
     *                  non-null.
     *
     * @param[in]       p_pbStopFlag_in
     *                  Optional flag that, when true, aborts the optimisation;
     *                  may be null.
     *
     * @param[in,out]   p_map_inout
     *                  Map holding the current keyframe; shall be non-null.
     *
     * @param[in,out]   corrPoses_inout
     *                  Receives, for every optimised keyframe, its corrected
     *                  similarity pose from world to camera with scale 1;
     *                  existing entries for other keyframes are kept.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus
        mergeInertialBA(KeyFrame                     *p_currentKeyFrame_inout,
                        KeyFrame                     *p_mergeKeyFrame_inout,
                        bool                         *p_pbStopFlag_in,
                        Map                          *p_map_inout,
                        LoopClosing::KeyFrameAndPose &corrPoses_inout);

    /*!
     * @brief           Marginalises one block of an information matrix with the
     *                  Schur complement.
     *
     *                  The block's rows and columns of the result are filled
     *                  with zeros; the rest holds the Schur complement. The
     *                  block is inverted with its pseudo-inverse: singular
     *                  values below 1e-6 are treated as zero.
     *
     * @param[in]       H_in
     *                  Square information (Hessian) matrix.
     *
     * @param[in]       start_in
     *                  First row and column of the block to marginalise.
     *
     * @param[in]       end_in
     *                  Last row and column of the block to marginalise,
     *                  inclusive.
     *
     * @param[out]      marginalized_out
     *                  Matrix of the same size as H_in with the block removed.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus
        marginalize(const Eigen::MatrixXd &H_in,
                    const int             &start_in,
                    const int             &end_in,
                    Eigen::MatrixXd       &marginalized_out);

    // Inertial pose-graph
    /*!
     * @brief           Estimates gravity direction, scale, IMU biases and
     *                  keyframe velocities of a map from its keyframe poses,
     *                  which stay fixed.
     *
     *                  Writes the keyframe velocities, and the biases of
     *                  keyframes whose gyro bias changed by more than 0.01,
     *                  into the map's keyframes.
     *
     * @param[in]       p_map_in
     *                  Map whose keyframes are used; shall be non-null.
     *
     * @param[in,out]   Rwg_inout
     *                  Rotation that takes the gravity direction into the world
     *                  frame; initial guess on entry, estimate on exit.
     *
     * @param[in,out]   scale_inout
     *                  Metric scale of the map; initial guess on entry,
     *                  estimate on exit. Held fixed when isMono_in is false.
     *
     * @param[out]      bg_in
     *                  Estimated gyroscope bias.
     *
     * @param[out]      ba_in
     *                  Estimated accelerometer bias.
     *
     * @param[in]       isMono_in
     *                  True to estimate the scale, false to keep it fixed.
     *
     * @param[in]       covInertial_in
     *                  Not used.
     *
     * @param[in]       isFixedVelocity_in
     *                  True to hold velocities and biases fixed and estimate
     *                  only gravity and scale.
     *
     * @param[in]       shouldUseGaussNewton_in
     *                  Not used.
     *
     * @param[in]       priorG_in
     *                  Information weight of the zero prior on the gyroscope
     *                  bias; a non-zero value also makes the solver start
     *                  cautiously.
     *
     * @param[in]       priorA_in
     *                  Information weight of the zero prior on the
     *                  accelerometer bias.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus
        inertialOptimization(Map             *p_map_in,
                             Eigen::Matrix3d &Rwg_inout,
                             double          &scale_inout,
                             Eigen::Vector3d &bg_in,
                             Eigen::Vector3d &ba_in,
                             bool             isMono_in,
                             Eigen::MatrixXd &covInertial_in,
                             bool             isFixedVelocity_in      = false,
                             bool             shouldUseGaussNewton_in = false,
                             float            priorG_in               = 1e2,
                             float            priorA_in               = 1e6);
    /*!
     * @brief           Estimates the IMU biases and keyframe velocities of a
     *                  map from its keyframe poses, with gravity and scale held
     *                  at identity and 1.
     *
     *                  Writes the keyframe velocities, and the biases of
     *                  keyframes whose gyro bias changed by more than 0.01,
     *                  into the map's keyframes.
     *
     * @param[in]       p_map_in
     *                  Map whose keyframes are used; shall be non-null.
     *
     * @param[out]      gyroBias_out
     *                  Estimated gyroscope bias.
     *
     * @param[out]      accelBias_out
     *                  Estimated accelerometer bias.
     *
     * @param[in]       gyroBiasPriorWeight_in
     *                  Information weight of the zero prior on the gyroscope
     *                  bias.
     *
     * @param[in]       accelBiasPriorWeight_in
     *                  Information weight of the zero prior on the
     *                  accelerometer bias.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus
        inertialOptimization(Map             *p_map_in,
                             Eigen::Vector3d &gyroBias_out,
                             Eigen::Vector3d &accelBias_out,
                             float            gyroBiasPriorWeight_in  = 1e2,
                             float            accelBiasPriorWeight_in = 1e6);
    /*!
     * @brief           Refines only the gravity direction and the scale of a
     *                  map from its keyframe poses; velocities and biases stay
     *                  fixed at the keyframes' current values.
     *
     * @param[in]       p_map_in
     *                  Map whose keyframes are used; shall be non-null.
     *
     * @param[in,out]   Rwg_inout
     *                  Rotation that takes the gravity direction into the world
     *                  frame; initial guess on entry, estimate on exit.
     *
     * @param[in,out]   scale_inout
     *                  Metric scale of the map; initial guess on entry,
     *                  estimate on exit.
     *
     * @return          OPTIMIZER_STATUS_SUCCESS always.
     */
    [[nodiscard]] static OptimizerStatus
        inertialOptimization(Map             *p_map_in,
                             Eigen::Matrix3d &Rwg_inout,
                             double          &scale_inout);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace core
} // namespace vs_graphs

#endif // OPTIMIZER_H
