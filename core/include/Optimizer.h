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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Frame.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"

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

class Optimizer
{
  public:
    void static bundleAdjustment(
        const std::vector<vs_graphs::core::KeyFrame *>          &keyFrames_in,
        const std::vector<vs_graphs::core::MapPoint *>          &mapPoints_in,
        const std::vector<vs_graphs::core::semantic::Marker *>  &markers_in,
        const std::vector<vs_graphs::core::geometric::Plane *>  &planes_in,
        const std::vector<vs_graphs::core::semantic::Passage *> &doorways_in,
        const std::vector<vs_graphs::core::semantic::Room *>    &rooms_in,
        const std::vector<vs_graphs::core::semantic::Floor *>   &floors_in,
        int                     iterationCount_in  = 5,
        bool                   *p_stopFlag_inout   = nullptr,
        const unsigned long     loopKeyFrameId_in  = 0,
        const bool              useRobustKernel_in = true,
        double                  markerImpact_in    = 0.1,
        const std::atomic_bool *p_stopRequested_in = nullptr);

    void static globalBundleAdjustment(
        Map                    *p_map_in,
        int                     iterationCount_in  = 5,
        bool                   *p_stopFlag_inout   = nullptr,
        const unsigned long     loopKeyFrameId_in  = 0,
        const bool              useRobustKernel_in = true,
        double                  markerImpact_in    = 0.1,
        const std::atomic_bool *p_stopRequested_in = nullptr);

    void static fullInertialBA(
        Map                    *p_map_inout,
        int                     iterationCount_in,
        const bool              fixLocalKeyFrames_in    = false,
        const unsigned long     loopKeyFrameId_in       = 0,
        bool                   *p_stopFlag_inout        = nullptr,
        bool                    isImuInitialization_in  = false,
        float                   gyroBiasPriorWeight_in  = 1e2,
        float                   accelBiasPriorWeight_in = 1e6,
        Eigen::VectorXd        *p_singularValues_in     = nullptr,
        bool                   *p_hessianComputed_in    = nullptr,
        const std::atomic_bool *p_stopRequested_in      = nullptr);

    void static localBundleAdjustment(KeyFrame *p_keyFrame_inout,
                                      bool     *p_pbStopFlag_in,
                                      Map      *p_map_inout,
                                      int      &fixedKeyFrameCount_inout,
                                      int      &optKeyFrameCount_out,
                                      int      &mapPointCount_out,
                                      int      &edgeCount_out,
                                      double    markerImpact_in = 0.1);

    /*!
     * @brief Local Bundle Adjustment for loop closure detection
     *
     * @param p_mainKeyFrame_in Main KeyFrame
     * @param adjustKeyFrames_in Non-fixed KeyFrames to adjust
     * @param fixedKeyFrames_in Fixed KeyFrames to set
     * @param p_pbStopFlag_in Flag to forcely stop the optimization
     */
    void static loopClosureLocalBundleAdjustment(
        KeyFrame          *p_mainKeyFrame_in,
        vector<KeyFrame *> adjustKeyFrames_in,
        vector<KeyFrame *> fixedKeyFrames_in,
        bool              *p_pbStopFlag_in);

    int static poseOptimization(Frame *p_frame_inout);
    int static poseInertialOptimizationLastKeyFrame(
        Frame *p_frame_inout,
        bool   isRecentlyInitialized_in = false);
    int static poseInertialOptimizationLastFrame(
        Frame *p_frame_inout,
        bool   isRecentlyInitialized_in = false);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise
    // (mono)
    void static optimizeEssentialGraph(
        Map                                    *p_map_inout,
        KeyFrame                               *p_loopKeyFrame_in,
        KeyFrame                               *p_currentKeyFrame_in,
        const LoopClosing::KeyFrameAndPose     &NonCorrectedSim3_in,
        const LoopClosing::KeyFrameAndPose     &CorrectedSim3_in,
        const map<KeyFrame *, set<KeyFrame *>> &loopConnections_in,
        const bool                             &isScaleFixed_in);

    /*!
     * @brief Optimize the Essential Graph when a loop closure is detected
     *
     * @param p_currentKeyFrame_in Current KeyFrame
     * @param[in]     p_sourceMap_in Source map whose semantic graph follows
     *                the optimized keyframe deformation.
     * @param fixedKeyFrames_in Fixed KeyFrames
     * @param fixedCorrectedKeyFrames_in Corrected Fixed KeyFrames
     * @param nonFixedKeyFrames_in Non-Fixed KeyFrames
     * @param nonCorrectedMapPoints_in Non-Corrected MapPoints
     * @param[in] transform_mergeWorldToCurrentWorld_in Baseline similarity
     * transform used when a semantic object has no valid reference
     * keyframe.
     */
    void static optimizeEssentialGraph(
        vs_graphs::core::KeyFrame                *p_currentKeyFrame_in,
        vs_graphs::core::Map                     *p_sourceMap_in,
        std::vector<vs_graphs::core::KeyFrame *> &fixedKeyFrames_in,
        std::vector<vs_graphs::core::KeyFrame *> &fixedCorrectedKeyFrames_in,
        std::vector<vs_graphs::core::KeyFrame *> &nonFixedKeyFrames_in,
        std::vector<vs_graphs::core::MapPoint *> &nonCorrectedMapPoints_in,
        const g2o::Sim3 &transform_mergeWorldToCurrentWorld_in);

    // For inertial loopclosing
    void static optimizeEssentialGraph4DoF(
        Map                                    *p_map_inout,
        KeyFrame                               *p_loopKeyFrame_in,
        KeyFrame                               *p_currentKeyFrame_in,
        const LoopClosing::KeyFrameAndPose     &NonCorrectedSim3_in,
        const LoopClosing::KeyFrameAndPose     &CorrectedSim3_in,
        const map<KeyFrame *, set<KeyFrame *>> &loopConnections_in);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    // (NEW)
    static int optimizeSim3(KeyFrame                    *p_keyFrame1_in,
                            KeyFrame                    *p_keyFrame2_in,
                            std::vector<MapPoint *>     &matches1_inout,
                            g2o::Sim3                   &g2oS12_inout,
                            const float                  threshold2_in,
                            const bool                   isScaleFixed_in,
                            Eigen::Matrix<double, 7, 7> &acumHessian_out,
                            const bool shouldUseAllPoints_in = false);

    // For inertial systems

    void static localInertialBA(KeyFrame *p_keyFrame_inout,
                                bool     *p_pbStopFlag_in,
                                Map      *p_map_inout,
                                int      &fixedKeyFrameCount_in,
                                int      &optKeyFrameCount_in,
                                int      &mapPointCount_in,
                                int      &edgeCount_in,
                                bool      isLargeWindow_in         = false,
                                bool      isRecentlyInitialized_in = false);
    void static mergeInertialBA(KeyFrame *p_currentKeyFrame_inout,
                                KeyFrame *p_mergeKeyFrame_inout,
                                bool     *p_pbStopFlag_in,
                                Map      *p_map_inout,
                                LoopClosing::KeyFrameAndPose &corrPoses_inout);

    // Marginalize block element (start:end,start:end). Perform Schur
    // complement. Marginalized elements are filled with zeros.
    static Eigen::MatrixXd marginalize(const Eigen::MatrixXd &H_in,
                                       const int             &start_in,
                                       const int             &end_in);

    // Inertial pose-graph
    void static inertialOptimization(Map             *p_map_in,
                                     Eigen::Matrix3d &Rwg_inout,
                                     double          &scale_inout,
                                     Eigen::Vector3d &bg_in,
                                     Eigen::Vector3d &ba_in,
                                     bool             isMono_in,
                                     Eigen::MatrixXd &covInertial_in,
                                     bool  isFixedVelocity_in      = false,
                                     bool  shouldUseGaussNewton_in = false,
                                     float priorG_in               = 1e2,
                                     float priorA_in               = 1e6);
    void static inertialOptimization(Map             *p_map_in,
                                     Eigen::Vector3d &gyroBias_out,
                                     Eigen::Vector3d &accelBias_out,
                                     float gyroBiasPriorWeight_in  = 1e2,
                                     float accelBiasPriorWeight_in = 1e6);
    void static inertialOptimization(Map             *p_map_in,
                                     Eigen::Matrix3d &Rwg_inout,
                                     double          &scale_inout);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace core
} // namespace vs_graphs

#endif // OPTIMIZER_H
