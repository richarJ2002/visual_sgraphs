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

#include "Optimizer.h"

#include "../private_functions.h"

#include "G2oTypes.h"
#include "System.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

int Optimizer::poseInertialOptimizationLastFrame(Frame *p_frame_inout,
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

    // Set Current Frame vertex
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
    vector<EdgeMonoOnlyPose *>   edgesMonos;
    vector<EdgeStereoOnlyPose *> edgesStereos;
    vector<size_t>               monoEdgeIndices;
    vector<size_t>               stereoEdgeIndices;
    addPoseOnlyObservationEdges(p_frame_inout,
                                p_poseVertex,
                                optimizer,
                                edgesMonos,
                                edgesStereos,
                                monoEdgeIndices,
                                stereoEdgeIndices,
                                initialMonoCorrespondenceCount,
                                initialStereoCorrespondenceCount);

    initialCorrespondenceCount =
        initialMonoCorrespondenceCount + initialStereoCorrespondenceCount;

    // Set Previous Frame Vertex
    Frame *p_previousFrame = p_frame_inout->p_previousFrame;

    VertexPose *VPk = new VertexPose(p_previousFrame);
    VPk->setId(4);
    VPk->setFixed(false);
    optimizer.addVertex(VPk);
    VertexVelocity *VVk = new VertexVelocity(p_previousFrame);
    VVk->setId(5);
    VVk->setFixed(false);
    optimizer.addVertex(VVk);
    VertexGyroBias *VGk = new VertexGyroBias(p_previousFrame);
    VGk->setId(6);
    VGk->setFixed(false);
    optimizer.addVertex(VGk);
    VertexAccBias *VAk = new VertexAccBias(p_previousFrame);
    VAk->setId(7);
    VAk->setFixed(false);
    optimizer.addVertex(VAk);

    EdgeInertial *ei =
        new EdgeInertial(p_frame_inout->p_imuPreintegratedFrame.get());

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

    EdgePriorPoseImu *ep = nullptr;
    if (p_previousFrame->p_poseImuConstraint)
    {
        ep = new EdgePriorPoseImu(p_previousFrame->p_poseImuConstraint);

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
        Verbose::printMess(
            "pFp->p_poseImuConstraint does not exist!!!\nPrevious Frame " +
                to_string(p_previousFrame->id),
            Verbose::VERBOSITY_NORMAL);
    }

    // We perform 4 optimizations, after each optimization we classify
    // observation as inlier/outlier At the next optimization, outliers are not
    // included, but at the end they can be classified as inliers again.
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
            EdgeMonoOnlyPose *e = edgesMonos[keyPointIndex];

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

    inlierCount = inliersMonoCount + inliersStereoCount;

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

    // Recover Hessian, marginalize previous frame states and generate new prior
    // for frame
    Eigen::Matrix<double, 30, 30> H;
    H.setZero();

    H.block<24, 24>(0, 0) += ei->getHessian();

    Eigen::Matrix<double, 6, 6> Hgr = p_egr->getHessian();
    H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
    H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
    H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
    H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

    Eigen::Matrix<double, 6, 6> Har = p_ear->getHessian();
    H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
    H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
    H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
    H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

    if (ep)
    {
        H.block<15, 15>(0, 0) += ep->getHessian();
    }

    int tot_in = 0, tot_out = 0;
    for (size_t keyPointIndex = 0, iend = edgesMonos.size();
         keyPointIndex < iend;
         keyPointIndex++)
    {
        EdgeMonoOnlyPose *e = edgesMonos[keyPointIndex];

        const size_t featureIndex = monoEdgeIndices[keyPointIndex];

        if (!p_frame_inout->outlierFlags[featureIndex])
        {
            H.block<6, 6>(15, 15) += e->getHessian();
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
            H.block<6, 6>(15, 15) += e->getHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

    H = marginalize(H, 0, 14);

    p_frame_inout->p_poseImuConstraint =
        new ConstraintPoseImu(p_poseVertex->estimate().Rwb,
                              p_poseVertex->estimate().twb,
                              p_velocityVertex->estimate(),
                              p_gyroBiasVertex->estimate(),
                              p_accelerometerBiasVertex->estimate(),
                              H.block<15, 15>(15, 15));
    delete p_previousFrame->p_poseImuConstraint;
    p_previousFrame->p_poseImuConstraint = nullptr;

    return initialCorrespondenceCount - badCount;
}

} // namespace core
} // namespace vs_graphs
