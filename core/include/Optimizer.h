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
        const std::vector<vs_graphs::core::KeyFrame *>          &vpKF,
        const std::vector<vs_graphs::core::MapPoint *>          &vpMP,
        const std::vector<vs_graphs::core::semantic::Marker *>  &vpMarkers,
        const std::vector<vs_graphs::core::geometric::Plane *>  &vpPlanes,
        const std::vector<vs_graphs::core::semantic::Passage *> &vpDoorways,
        const std::vector<vs_graphs::core::semantic::Room *>    &vpRooms,
        const std::vector<vs_graphs::core::semantic::Floor *>   &vpFloors,
        int                     nIterations       = 5,
        bool                   *pbStopFlag        = nullptr,
        const unsigned long     nLoopKF           = 0,
        const bool              bRobust           = true,
        double                  markerImpact      = 0.1,
        const std::atomic_bool *pStopRequested_in = nullptr);

    void static globalBundleAdjustment(
        Map                    *pMap,
        int                     nIterations       = 5,
        bool                   *pbStopFlag        = nullptr,
        const unsigned long     nLoopKF           = 0,
        const bool              bRobust           = true,
        double                  markerImpact      = 0.1,
        const std::atomic_bool *pStopRequested_in = nullptr);

    void static fullInertialBA(
        Map                    *pMap,
        int                     its,
        const bool              bFixLocal         = false,
        const unsigned long     nLoopKF           = 0,
        bool                   *pbStopFlag        = nullptr,
        bool                    bInit             = false,
        float                   priorG            = 1e2,
        float                   priorA            = 1e6,
        Eigen::VectorXd        *vSingVal          = nullptr,
        bool                   *bHess             = nullptr,
        const std::atomic_bool *pStopRequested_in = nullptr);

    void static localBundleAdjustment(KeyFrame *pKF,
                                      bool     *pbStopFlag,
                                      Map      *pMap,
                                      int      &countFixedKF,
                                      int      &num_OptKF,
                                      int      &num_MPs,
                                      int      &num_edges,
                                      double    markerImpact = 0.1);

    /*!
     * @brief Local Bundle Adjustment for loop closure detection
     *
     * @param pMainKF Main KeyFrame
     * @param vpAdjustKF Non-fixed KeyFrames to adjust
     * @param vpFixedKF Fixed KeyFrames to set
     * @param pbStopFlag Flag to forcely stop the optimization
     */
    void static loopClosureLocalBundleAdjustment(KeyFrame          *pMainKF,
                                                 vector<KeyFrame *> vpAdjustKF,
                                                 vector<KeyFrame *> vpFixedKF,
                                                 bool              *pbStopFlag);

    int static poseOptimization(Frame *pFrame);
    int static poseInertialOptimizationLastKeyFrame(Frame *pFrame,
                                                    bool   bRecInit = false);
    int static poseInertialOptimizationLastFrame(Frame *pFrame,
                                                 bool   bRecInit = false);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise
    // (mono)
    void static optimizeEssentialGraph(
        Map                                    *pMap,
        KeyFrame                               *pLoopKF,
        KeyFrame                               *pCurKF,
        const LoopClosing::KeyFrameAndPose     &NonCorrectedSim3,
        const LoopClosing::KeyFrameAndPose     &CorrectedSim3,
        const map<KeyFrame *, set<KeyFrame *>> &LoopConnections,
        const bool                             &bFixScale);

    /*!
     * @brief Optimize the Essential Graph when a loop closure is detected
     *
     * @param pCurKF Current KeyFrame
     * @param[in,out] p_sourceMap_inout Source map whose semantic graph follows
     *                the optimized keyframe deformation.
     * @param vpFixedKFs Fixed KeyFrames
     * @param vpFixedCorrectedKFs Corrected Fixed KeyFrames
     * @param vpNonFixedKFs Non-Fixed KeyFrames
     * @param vpNonCorrectedMPs Non-Corrected MapPoints
     * @param[in] transform_mergeWorldToCurrentWorld_in Baseline similarity
     * transform used when a semantic object has no valid reference
     * keyframe.
     */
    void static optimizeEssentialGraph(
        vs_graphs::core::KeyFrame                *pCurKF,
        vs_graphs::core::Map                     *p_sourceMap_inout,
        std::vector<vs_graphs::core::KeyFrame *> &vpFixedKFs,
        std::vector<vs_graphs::core::KeyFrame *> &vpFixedCorrectedKFs,
        std::vector<vs_graphs::core::KeyFrame *> &vpNonFixedKFs,
        std::vector<vs_graphs::core::MapPoint *> &vpNonCorrectedMPs,
        const g2o::Sim3 &transform_mergeWorldToCurrentWorld_in);

    // For inertial loopclosing
    void static optimizeEssentialGraph4DoF(
        Map                                    *pMap,
        KeyFrame                               *pLoopKF,
        KeyFrame                               *pCurKF,
        const LoopClosing::KeyFrameAndPose     &NonCorrectedSim3,
        const LoopClosing::KeyFrameAndPose     &CorrectedSim3,
        const map<KeyFrame *, set<KeyFrame *>> &LoopConnections);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    // (NEW)
    static int optimizeSim3(KeyFrame                    *pKF1,
                            KeyFrame                    *pKF2,
                            std::vector<MapPoint *>     &vpMatches1,
                            g2o::Sim3                   &g2oS12,
                            const float                  th2,
                            const bool                   bFixScale,
                            Eigen::Matrix<double, 7, 7> &mAcumHessian,
                            const bool                   bAllPoints = false);

    // For inertial systems

    void static localInertialBA(KeyFrame *pKF,
                                bool     *pbStopFlag,
                                Map      *pMap,
                                int      &countFixedKF,
                                int      &num_OptKF,
                                int      &num_MPs,
                                int      &num_edges,
                                bool      bLarge   = false,
                                bool      bRecInit = false);
    void static mergeInertialBA(KeyFrame                     *pCurrKF,
                                KeyFrame                     *pMergeKF,
                                bool                         *pbStopFlag,
                                Map                          *pMap,
                                LoopClosing::KeyFrameAndPose &corrPoses);

    // Marginalize block element (start:end,start:end). Perform Schur
    // complement. Marginalized elements are filled with zeros.
    static Eigen::MatrixXd
        marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);

    // Inertial pose-graph
    void static inertialOptimization(Map             *pMap,
                                     Eigen::Matrix3d &Rwg,
                                     double          &scale,
                                     Eigen::Vector3d &bg,
                                     Eigen::Vector3d &ba,
                                     bool             bMono,
                                     Eigen::MatrixXd &covInertial,
                                     bool             bFixedVel = false,
                                     bool             bGauss    = false,
                                     float            priorG    = 1e2,
                                     float            priorA    = 1e6);
    void static inertialOptimization(Map             *pMap,
                                     Eigen::Vector3d &bg,
                                     Eigen::Vector3d &ba,
                                     float            priorG = 1e2,
                                     float            priorA = 1e6);
    void static inertialOptimization(Map             *pMap,
                                     Eigen::Matrix3d &Rwg,
                                     double          &scale);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace core
} // namespace vs_graphs

#endif // OPTIMIZER_H
