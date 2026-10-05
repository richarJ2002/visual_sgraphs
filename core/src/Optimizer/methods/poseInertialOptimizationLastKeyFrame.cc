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
 * @file            poseInertialOptimizationLastKeyFrame.cc
 *
 * @brief           Implements
 *                  Optimizer::poseInertialOptimizationLastKeyFrame(), declared
 *                  in Optimizer.h.
 */

#include "Optimizer.h"

#include "../private_functions/private_functions.h"

#include "G2oTypes.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

OptimizerStatus Optimizer::poseInertialOptimizationLastKeyFrame(
    Frame *p_frame_inout,
    int   &inlierCount_out,
    bool   isRecentlyInitialized_in)
{
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver;

    p_linearSolver =
        new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(p_linearSolver);

    g2o::OptimizationAlgorithmGaussNewton *p_solver =
        new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setVerbose(false);
    optimizer.setAlgorithm(p_solver);

    int initialMonoCorrespondenceCount   = 0;
    int initialStereoCorrespondenceCount = 0;
    int initialCorrespondenceCount       = 0;

    // Set Frame vertex
    VertexPose *p_poseVertex = new VertexPose(p_frame_inout);
    p_poseVertex->setId(0);
    p_poseVertex->setFixed(false);
    optimizer.addVertex(p_poseVertex);
    VertexVelocity *p_velocityVertex = new VertexVelocity(p_frame_inout);
    p_velocityVertex->setId(1);
    p_velocityVertex->setFixed(false);
    optimizer.addVertex(p_velocityVertex);
    VertexGyroBias *p_gyroBiasVertex = new VertexGyroBias(p_frame_inout);
    p_gyroBiasVertex->setId(2);
    p_gyroBiasVertex->setFixed(false);
    optimizer.addVertex(p_gyroBiasVertex);
    VertexAccBias *p_accelerometerBiasVertex = new VertexAccBias(p_frame_inout);
    p_accelerometerBiasVertex->setId(3);
    p_accelerometerBiasVertex->setFixed(false);
    optimizer.addVertex(p_accelerometerBiasVertex);

    // Set MapPoint vertices
    std::vector<EdgeMonoOnlyPose *>   edgesMonos;
    std::vector<EdgeStereoOnlyPose *> edgesStereos;
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

    KeyFrame   *p_keyFrame = p_frame_inout->p_lastKeyFrame;
    VertexPose *VPk        = new VertexPose(p_keyFrame);
    VPk->setId(4);
    VPk->setFixed(true);
    optimizer.addVertex(VPk);
    VertexVelocity *VVk = new VertexVelocity(p_keyFrame);
    VVk->setId(5);
    VVk->setFixed(true);
    optimizer.addVertex(VVk);
    VertexGyroBias *VGk = new VertexGyroBias(p_keyFrame);
    VGk->setId(6);
    VGk->setFixed(true);
    optimizer.addVertex(VGk);
    VertexAccBias *VAk = new VertexAccBias(p_keyFrame);
    VAk->setId(7);
    VAk->setFixed(true);
    optimizer.addVertex(VAk);

    EdgeInertial *ei = new EdgeInertial(p_frame_inout->p_imuPreintegrated);

    ei->setVertex(0, VPk);
    ei->setVertex(1, VVk);
    ei->setVertex(2, VGk);
    ei->setVertex(3, VAk);
    ei->setVertex(4, p_poseVertex);
    ei->setVertex(5, p_velocityVertex);
    optimizer.addEdge(ei);

    EdgeGyroRW *p_egr = new EdgeGyroRW();
    p_egr->setVertex(0, VGk);
    p_egr->setVertex(1, p_gyroBiasVertex);
    Eigen::Matrix3d informationG =
        p_frame_inout->p_imuPreintegrated->C.block<3, 3>(9, 9)
            .cast<double>()
            .inverse();
    p_egr->setInformation(informationG);
    optimizer.addEdge(p_egr);

    EdgeAccRW *p_ear = new EdgeAccRW();
    p_ear->setVertex(0, VAk);
    p_ear->setVertex(1, p_accelerometerBiasVertex);
    Eigen::Matrix3d informationA =
        p_frame_inout->p_imuPreintegrated->C.block<3, 3>(12, 12)
            .cast<double>()
            .inverse();
    p_ear->setInformation(informationA);
    optimizer.addEdge(p_ear);

    // We perform 4 optimizations, after each optimization we classify
    // observation as inlier/outlier At the next optimization, outliers are not
    // included, but at the end they can be classified as inliers again.
    float chi2Mono[4]   = {12, 7.5, 5.991, 5.991};
    float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};

    int its[4] = {10, 10, 10, 10};

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

        // For monocular observations
        for (size_t keyPointIndex = 0, iend = edgesMonos.size();
             keyPointIndex < iend;
             keyPointIndex++)
        {
            EdgeMonoOnlyPose *e = edgesMonos[keyPointIndex];

            const size_t featureIndex = monoEdgeIndices[keyPointIndex];

            if (p_frame_inout->outlierFlags[featureIndex])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();
            bool        isClosePoint =
                p_frame_inout->mapPoints[featureIndex]->trackDepth < 10.f;

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

        // For stereo observations
        for (size_t keyPointIndex = 0, iend = edgesStereos.size();
             keyPointIndex < iend;
             keyPointIndex++)
        {
            EdgeStereoOnlyPose *e = edgesStereos[keyPointIndex];

            const size_t featureIndex = stereoEdgeIndices[keyPointIndex];

            if (p_frame_inout->outlierFlags[featureIndex])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[iterationIndex])
            {
                p_frame_inout->outlierFlags[featureIndex] = true;
                e->setLevel(1); // not included in next optimization
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

    // If not too much tracks, recover not too bad points
    if ((inlierCount < 30) && !isRecentlyInitialized_in)
    {
        badCount                          = 0;
        const float         chi2MonoOut   = 18.f;
        const float         chi2StereoOut = 24.f;
        EdgeMonoOnlyPose   *e1;
        EdgeStereoOnlyPose *e2;
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

    // Recover optimized pose, velocity and biases
    if (p_frame_inout->setImuPoseVelocity(
            p_poseVertex->estimate().Rwb.cast<float>(),
            p_poseVertex->estimate().twb.cast<float>(),
            p_velocityVertex->estimate().cast<float>()) !=
        FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setImuPoseVelocity returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Vector6d b;
    b << p_gyroBiasVertex->estimate(), p_accelerometerBiasVertex->estimate();
    p_frame_inout->imuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

    // Recover Hessian, marginalize keyFframe states and generate new prior for
    // frame
    Eigen::Matrix<double, 15, 15> H;
    H.setZero();

    Eigen::Matrix<double, 9, 9> eiHessian2{};
    if (ei->getHessian2(eiHessian2) !=
        EdgeInertialStatus::EDGE_INERTIAL_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHessian2 returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    H.block<9, 9>(0, 0) += eiHessian2;
    Eigen::Matrix3d egrHessian2{};
    if (p_egr->getHessian2(egrHessian2) !=
        EdgeGyroRWStatus::EDGE_GYRO_RWSTATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHessian2 returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    H.block<3, 3>(9, 9) += egrHessian2;
    Eigen::Matrix3d earHessian2{};
    if (p_ear->getHessian2(earHessian2) !=
        EdgeAccRWStatus::EDGE_ACC_RWSTATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getHessian2 returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    H.block<3, 3>(12, 12) += earHessian2;

    int tot_in = 0, tot_out = 0;
    for (size_t keyPointIndex = 0, iend = edgesMonos.size();
         keyPointIndex < iend;
         keyPointIndex++)
    {
        EdgeMonoOnlyPose *e = edgesMonos[keyPointIndex];

        const size_t featureIndex = monoEdgeIndices[keyPointIndex];

        if (!p_frame_inout->outlierFlags[featureIndex])
        {
            Eigen::Matrix<double, 6, 6> eHessian{};
            if (e->getHessian(eHessian) !=
                EdgeMonoOnlyPoseStatus::EDGE_MONO_ONLY_POSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHessian returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            H.block<6, 6>(0, 0) += eHessian;
            tot_in++;
        }
        else
            tot_out++;
    }

    for (size_t keyPointIndex = 0, iend = edgesStereos.size();
         keyPointIndex < iend;
         keyPointIndex++)
    {
        EdgeStereoOnlyPose *e = edgesStereos[keyPointIndex];

        const size_t featureIndex = stereoEdgeIndices[keyPointIndex];

        if (!p_frame_inout->outlierFlags[featureIndex])
        {
            Eigen::Matrix<double, 6, 6> eHessian2{};
            if (e->getHessian(eHessian2) !=
                EdgeStereoOnlyPoseStatus::EDGE_STEREO_ONLY_POSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getHessian returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            H.block<6, 6>(0, 0) += eHessian2;
            tot_in++;
        }
        else
            tot_out++;
    }

    p_frame_inout->p_poseImuConstraint =
        new ConstraintPoseImu(p_poseVertex->estimate().Rwb,
                              p_poseVertex->estimate().twb,
                              p_velocityVertex->estimate(),
                              p_gyroBiasVertex->estimate(),
                              p_accelerometerBiasVertex->estimate(),
                              H);

    inlierCount_out = initialCorrespondenceCount - badCount;
    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
