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
 * @file            poseInertialOptimizationLastFrame.cc
 *
 * @brief           Implements Optimizer::poseInertialOptimizationLastFrame(),
 *                  declared in optimizer/objects/OptimizerClass.h.
 */

#include "optimization/optimizer/objects/OptimizerClass.h"
#include "map_state/frame/frame.h"
#include "map_state/frame/objects/FrameStatusEnum.h"
#include "optimization/g2o_types/edge_acc_rw/edge_acc_rw.h"
#include "optimization/g2o_types/edge_acc_rw/objects/EdgeAccRWStatusEnum.h"
#include "optimization/g2o_types/edge_gyro_rw/edge_gyro_rw.h"
#include "optimization/g2o_types/edge_gyro_rw/objects/EdgeGyroRWStatusEnum.h"
#include "optimization/g2o_types/edge_inertial/objects/EdgeInertialStatusEnum.h"
#include "optimization/g2o_types/edge_mono_only_pose/objects/EdgeMonoOnlyPoseStatusEnum.h"
#include "optimization/g2o_types/edge_prior_pose_imu/objects/EdgePriorPoseImuStatusEnum.h"
#include "optimization/g2o_types/edge_stereo_only_pose/objects/EdgeStereoOnlyPoseStatusEnum.h"
#include "sensor_models/imu_types/bias/bias.h"
#include "execution/system/objects/VerboseStatusEnum.h"

#include "../private_functions/private_functions.h"

#include "optimization/g2o_types/edge_inertial/edge_inertial.h"
#include "optimization/g2o_types/edge_mono_only_pose/edge_mono_only_pose.h"
#include "optimization/g2o_types/edge_prior_pose_imu/edge_prior_pose_imu.h"
#include "optimization/g2o_types/edge_stereo_only_pose/edge_stereo_only_pose.h"
#include "optimization/g2o_types/vertex_acc_bias/vertex_acc_bias.h"
#include "optimization/g2o_types/vertex_gyro_bias/vertex_gyro_bias.h"
#include "optimization/g2o_types/vertex_pose/vertex_pose.h"
#include "optimization/g2o_types/vertex_velocity/vertex_velocity.h"
#include "execution/system/system.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace optimizer
{

OptimizerStatus
    Optimizer::poseInertialOptimizationLastFrame(frame::Frame *p_frame_inout,
                                                 int          &inlierCount_out,
                                                 bool isRecentlyInitialized_in)
{
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver;

    p_linearSolver =
        new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(p_linearSolver);

    g2o::OptimizationAlgorithmGaussNewton *p_solver =
        new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(p_solver);
    optimizer.setVerbose(false);

    int initialMonoCorrespondenceCount   = 0;
    int initialStereoCorrespondenceCount = 0;
    int initialCorrespondenceCount       = 0;

    /* Set Current frame::Frame vertex */
    g2o_types::vertex_pose::VertexPose *p_poseVertex = new g2o_types::vertex_pose::VertexPose(p_frame_inout);
    p_poseVertex->setId(0);
    p_poseVertex->setFixed(false);
    optimizer.addVertex(p_poseVertex);
    g2o_types::vertex_velocity::VertexVelocity *p_velocityVertex = new g2o_types::vertex_velocity::VertexVelocity(p_frame_inout);
    p_velocityVertex->setId(1);
    p_velocityVertex->setFixed(false);
    optimizer.addVertex(p_velocityVertex);
    g2o_types::vertex_gyro_bias::VertexGyroBias *p_gyroBiasVertex = new g2o_types::vertex_gyro_bias::VertexGyroBias(p_frame_inout);
    p_gyroBiasVertex->setId(2);
    p_gyroBiasVertex->setFixed(false);
    optimizer.addVertex(p_gyroBiasVertex);
    g2o_types::vertex_acc_bias::VertexAccBias *p_accelerometerBiasVertex = new g2o_types::vertex_acc_bias::VertexAccBias(p_frame_inout);
    p_accelerometerBiasVertex->setId(3);
    p_accelerometerBiasVertex->setFixed(false);
    optimizer.addVertex(p_accelerometerBiasVertex);

    /* Set map_point::MapPoint vertices */
    std::vector<g2o_types::edge_mono_only_pose::EdgeMonoOnlyPose *>   edgesMonos;
    std::vector<g2o_types::edge_stereo_only_pose::EdgeStereoOnlyPose *> edgesStereos;
    std::vector<size_t>               monoEdgeIndices;
    std::vector<size_t>               stereoEdgeIndices;
    if (addPoseOnlyObservationEdges(p_frame_inout,
                                    p_poseVertex,
                                    optimizer,
                                    edgesMonos,
                                    edgesStereos,
                                    monoEdgeIndices,
                                    stereoEdgeIndices,
                                    initialMonoCorrespondenceCount,
                                    initialStereoCorrespondenceCount) !=
        OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addPoseOnlyObservationEdges returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    initialCorrespondenceCount =
        initialMonoCorrespondenceCount + initialStereoCorrespondenceCount;

    /* Set Previous frame::Frame Vertex */
    frame::Frame *p_previousFrame = p_frame_inout->p_previousFrame;

    g2o_types::vertex_pose::VertexPose *VPk = new g2o_types::vertex_pose::VertexPose(p_previousFrame);
    VPk->setId(4);
    VPk->setFixed(false);
    optimizer.addVertex(VPk);
    g2o_types::vertex_velocity::VertexVelocity *VVk = new g2o_types::vertex_velocity::VertexVelocity(p_previousFrame);
    VVk->setId(5);
    VVk->setFixed(false);
    optimizer.addVertex(VVk);
    g2o_types::vertex_gyro_bias::VertexGyroBias *VGk = new g2o_types::vertex_gyro_bias::VertexGyroBias(p_previousFrame);
    VGk->setId(6);
    VGk->setFixed(false);
    optimizer.addVertex(VGk);
    g2o_types::vertex_acc_bias::VertexAccBias *VAk = new g2o_types::vertex_acc_bias::VertexAccBias(p_previousFrame);
    VAk->setId(7);
    VAk->setFixed(false);
    optimizer.addVertex(VAk);

    g2o_types::edge_inertial::EdgeInertial *ei =
        new g2o_types::edge_inertial::EdgeInertial(p_frame_inout->p_imuPreintegratedFrame.get());

    ei->setVertex(0, VPk);
    ei->setVertex(1, VVk);
    ei->setVertex(2, VGk);
    ei->setVertex(3, VAk);
    ei->setVertex(4, p_poseVertex);
    ei->setVertex(5, p_velocityVertex);
    optimizer.addEdge(ei);

    g2o_types::edge_gyro_rw::EdgeGyroRW *p_egr = new g2o_types::edge_gyro_rw::EdgeGyroRW();
    p_egr->setVertex(0, VGk);
    p_egr->setVertex(1, p_gyroBiasVertex);
    Eigen::Matrix3d informationG =
        p_frame_inout->p_imuPreintegrated->C.block<3, 3>(9, 9)
            .cast<double>()
            .inverse();
    p_egr->setInformation(informationG);
    optimizer.addEdge(p_egr);

    g2o_types::edge_acc_rw::EdgeAccRW *p_ear = new g2o_types::edge_acc_rw::EdgeAccRW();
    p_ear->setVertex(0, VAk);
    p_ear->setVertex(1, p_accelerometerBiasVertex);
    Eigen::Matrix3d informationA =
        p_frame_inout->p_imuPreintegrated->C.block<3, 3>(12, 12)
            .cast<double>()
            .inverse();
    p_ear->setInformation(informationA);
    optimizer.addEdge(p_ear);

    g2o_types::edge_prior_pose_imu::EdgePriorPoseImu *ep = nullptr;
    if (p_previousFrame->p_poseImuConstraint)
    {
        ep = new g2o_types::edge_prior_pose_imu::EdgePriorPoseImu(p_previousFrame->p_poseImuConstraint);

        ep->setVertex(0, VPk);
        ep->setVertex(1, VVk);
        ep->setVertex(2, VGk);
        ep->setVertex(3, VAk);
        g2o::RobustKernelHuber *p_rkp = new g2o::RobustKernelHuber;
        ep->setRobustKernel(p_rkp);
        p_rkp->setDelta(5);
        optimizer.addEdge(ep);
    }
    else
    {
        if (system::Verbose::printMess("pFp->p_poseImuConstraint does not "
                               "exist!!!\nPrevious frame::Frame " +
                                   std::to_string(p_previousFrame->id),
                               system::Verbose::VERBOSITY_NORMAL) !=
            system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    /*!
     * We perform 4 optimizations, after each optimization we classify
     * observation as inlier/outlier At the next optimization, outliers are not
     * included, but at the end they can be classified as inliers again.
     */
    const float chi2Mono[4]   = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
    const int   its[4]        = {10, 10, 10, 10};

    int badCount           = 0;
    int badMonoCount       = 0;
    int badStereoCount     = 0;
    int inliersMonoCount   = 0;
    int inliersStereoCount = 0;
    int inlierCount        = 0;
    for (size_t iterationIndex = 0; iterationIndex < 4; iterationIndex++)
    {
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[iterationIndex]);

        badCount                       = 0;
        badMonoCount                   = 0;
        badStereoCount                 = 0;
        inlierCount                    = 0;
        inliersMonoCount               = 0;
        inliersStereoCount             = 0;
        float closeChiSquaredThreshold = 1.5 * chi2Mono[iterationIndex];

        for (size_t keyPointIndex = 0, iend = edgesMonos.size();
             keyPointIndex < iend;
             keyPointIndex++)
        {
            g2o_types::edge_mono_only_pose::EdgeMonoOnlyPose *e = edgesMonos[keyPointIndex];

            const size_t featureIndex = monoEdgeIndices[keyPointIndex];
            bool         isClosePoint =
                p_frame_inout->mapPoints[featureIndex]->trackDepth < 10.f;

            if (p_frame_inout->outlierFlags[featureIndex])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if ((chi2 > chi2Mono[iterationIndex] && !isClosePoint) ||
                (isClosePoint && chi2 > closeChiSquaredThreshold) ||
                !e->isDepthPositive())
            {
                p_frame_inout->outlierFlags[featureIndex] = true;
                e->setLevel(1);
                badMonoCount++;
            }
            else
            {
                p_frame_inout->outlierFlags[featureIndex] = false;
                e->setLevel(0);
                inliersMonoCount++;
            }

            if (iterationIndex == 2)
                e->setRobustKernel(0);
        }

        for (size_t keyPointIndex = 0, iend = edgesStereos.size();
             keyPointIndex < iend;
             keyPointIndex++)
        {
            g2o_types::edge_stereo_only_pose::EdgeStereoOnlyPose *e = edgesStereos[keyPointIndex];

            const size_t featureIndex = stereoEdgeIndices[keyPointIndex];

            if (p_frame_inout->outlierFlags[featureIndex])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[iterationIndex])
            {
                p_frame_inout->outlierFlags[featureIndex] = true;
                e->setLevel(1);
                badStereoCount++;
            }
            else
            {
                p_frame_inout->outlierFlags[featureIndex] = false;
                e->setLevel(0);
                inliersStereoCount++;
            }

            if (iterationIndex == 2)
                e->setRobustKernel(0);
        }

        inlierCount = inliersMonoCount + inliersStereoCount;
        badCount    = badMonoCount + badStereoCount;

        if (optimizer.edges().size() < 10)
        {
            break;
        }
    }

    if ((inlierCount < 30) && !isRecentlyInitialized_in)
    {
        badCount                          = 0;
        const float         chi2MonoOut   = 18.f;
        const float         chi2StereoOut = 24.f;
        g2o_types::edge_mono_only_pose::EdgeMonoOnlyPose   *e1;
        g2o_types::edge_stereo_only_pose::EdgeStereoOnlyPose *e2;
        for (size_t keyPointIndex = 0, iend = monoEdgeIndices.size();
             keyPointIndex < iend;
             keyPointIndex++)
        {
            const size_t featureIndex = monoEdgeIndices[keyPointIndex];
            e1                        = edgesMonos[keyPointIndex];
            e1->computeError();
            if (e1->chi2() < chi2MonoOut)
                p_frame_inout->outlierFlags[featureIndex] = false;
            else
                badCount++;
        }
        for (size_t keyPointIndex = 0, iend = stereoEdgeIndices.size();
             keyPointIndex < iend;
             keyPointIndex++)
        {
            const size_t featureIndex = stereoEdgeIndices[keyPointIndex];
            e2                        = edgesStereos[keyPointIndex];
            e2->computeError();
            if (e2->chi2() < chi2StereoOut)
                p_frame_inout->outlierFlags[featureIndex] = false;
            else
                badCount++;
        }
    }

    inlierCount = inliersMonoCount + inliersStereoCount;

    /* Recover optimized pose, velocity and biases */
    if (p_frame_inout->setImuPoseVelocity(
            p_poseVertex->estimate().Rwb.cast<float>(),
            p_poseVertex->estimate().twb.cast<float>(),
            p_velocityVertex->estimate().cast<float>()) !=
        frame::FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setImuPoseVelocity returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Matrix<double, 6, 1> b;
    b << p_gyroBiasVertex->estimate(), p_accelerometerBiasVertex->estimate();
    p_frame_inout->imuBias = imu_types::bias::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

    /*!
     * Recover Hessian, marginalize previous frame states and generate new prior
     * for frame
     */
    Eigen::Matrix<double, 30, 30> H;
    H.setZero();

    Eigen::Matrix<double, 24, 24> eiHessian{};
    if (ei->getHessian(eiHessian) !=
        g2o_types::edge_inertial::EdgeInertialStatus::EDGE_INERTIAL_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHessian returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    H.block<24, 24>(0, 0) += eiHessian;

    Eigen::Matrix<double, 6, 6> Hgr{};
    if (p_egr->getHessian(Hgr) != g2o_types::edge_gyro_rw::EdgeGyroRWStatus::EDGE_GYRO_RWSTATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHessian returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
    H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
    H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
    H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

    Eigen::Matrix<double, 6, 6> Har{};
    if (p_ear->getHessian(Har) != g2o_types::edge_acc_rw::EdgeAccRWStatus::EDGE_ACC_RWSTATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHessian returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
    H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
    H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
    H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

    if (ep)
    {
        Eigen::Matrix<double, 15, 15> epHessian{};
        if (ep->getHessian(epHessian) !=
            g2o_types::edge_prior_pose_imu::EdgePriorPoseImuStatus::EDGE_PRIOR_POSE_IMU_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getHessian returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        H.block<15, 15>(0, 0) += epHessian;
    }

    int tot_in = 0, tot_out = 0;
    for (size_t keyPointIndex = 0, iend = edgesMonos.size();
         keyPointIndex < iend;
         keyPointIndex++)
    {
        g2o_types::edge_mono_only_pose::EdgeMonoOnlyPose *e = edgesMonos[keyPointIndex];

        const size_t featureIndex = monoEdgeIndices[keyPointIndex];

        if (!p_frame_inout->outlierFlags[featureIndex])
        {
            Eigen::Matrix<double, 6, 6> eHessian{};
            if (e->getHessian(eHessian) !=
                g2o_types::edge_mono_only_pose::EdgeMonoOnlyPoseStatus::EDGE_MONO_ONLY_POSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHessian returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            H.block<6, 6>(15, 15) += eHessian;
            tot_in++;
        }
        else
            tot_out++;
    }

    for (size_t keyPointIndex = 0, iend = edgesStereos.size();
         keyPointIndex < iend;
         keyPointIndex++)
    {
        g2o_types::edge_stereo_only_pose::EdgeStereoOnlyPose *e = edgesStereos[keyPointIndex];

        const size_t featureIndex = stereoEdgeIndices[keyPointIndex];

        if (!p_frame_inout->outlierFlags[featureIndex])
        {
            Eigen::Matrix<double, 6, 6> eHessian2{};
            if (e->getHessian(eHessian2) !=
                g2o_types::edge_stereo_only_pose::EdgeStereoOnlyPoseStatus::EDGE_STEREO_ONLY_POSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHessian returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            H.block<6, 6>(15, 15) += eHessian2;
            tot_in++;
        }
        else
            tot_out++;
    }

    Eigen::MatrixXd marginalized{};
    if (marginalize(H, 0, 14, marginalized) !=
        OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: marginalize returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    H = marginalized;

    p_frame_inout->p_poseImuConstraint =
        new g2o_types::edge_prior_pose_imu::ConstraintPoseImu(p_poseVertex->estimate().Rwb,
                              p_poseVertex->estimate().twb,
                              p_velocityVertex->estimate(),
                              p_gyroBiasVertex->estimate(),
                              p_accelerometerBiasVertex->estimate(),
                              H.block<15, 15>(15, 15));
    delete p_previousFrame->p_poseImuConstraint;
    p_previousFrame->p_poseImuConstraint = nullptr;

    inlierCount_out = initialCorrespondenceCount - badCount;
    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} /* namespace optimizer */
} /* namespace core */
} /* namespace vs_graphs */
