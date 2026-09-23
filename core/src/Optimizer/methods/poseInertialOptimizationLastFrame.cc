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

#include "G2oTypes.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

int Optimizer::poseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit)
{
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver =
        new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton *solver =
        new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    int nInitialMonoCorrespondences   = 0;
    int nInitialStereoCorrespondences = 0;
    int nInitialCorrespondences       = 0;

    // Set Current Frame vertex
    VertexPose *VP = new VertexPose(pFrame);
    VP->setId(0);
    VP->setFixed(false);
    optimizer.addVertex(VP);
    VertexVelocity *VV = new VertexVelocity(pFrame);
    VV->setId(1);
    VV->setFixed(false);
    optimizer.addVertex(VV);
    VertexGyroBias *VG = new VertexGyroBias(pFrame);
    VG->setId(2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias *VA = new VertexAccBias(pFrame);
    VA->setId(3);
    VA->setFixed(false);
    optimizer.addVertex(VA);

    // Set MapPoint vertices
    const int  N      = pFrame->N;
    const int  Nleft  = pFrame->Nleft;
    const bool bRight = (Nleft != -1);

    vector<EdgeMonoOnlyPose *>   vpEdgesMono;
    vector<EdgeStereoOnlyPose *> vpEdgesStereo;
    vector<size_t>               vnIndexEdgeMono;
    vector<size_t>               vnIndexEdgeStereo;
    vpEdgesMono.reserve(N);
    vpEdgesStereo.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float thHuberMono   = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N; i++)
        {
            MapPoint *pMP = pFrame->mapPoints[i];
            if (pMP)
            {
                cv::KeyPoint kpUn;
                // Left monocular observation
                if ((!bRight && pFrame->uRight[i] < 0) || i < Nleft)
                {
                    if (i < Nleft) // pair left-right
                        kpUn = pFrame->keyPoints[i];
                    else
                        kpUn = pFrame->keyPointsUndistorted[i];

                    nInitialMonoCorrespondences++;
                    pFrame->outlierFlags[i] = false;

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose *e =
                        new EdgeMonoOnlyPose(pMP->getWorldPos(), 0);

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->p_camera->uncertainty2(obs);

                    const float invSigma2 =
                        pFrame->invLevelSigmaSquared[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                // Stereo observation
                else if (!bRight)
                {
                    nInitialStereoCorrespondences++;
                    pFrame->outlierFlags[i] = false;

                    kpUn = pFrame->keyPointsUndistorted[i];
                    const float                 kp_ur = pFrame->uRight[i];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereoOnlyPose *e =
                        new EdgeStereoOnlyPose(pMP->getWorldPos());

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 =
                        pFrame->p_camera->uncertainty2(obs.head(2));

                    const float &invSigma2 =
                        pFrame->invLevelSigmaSquared[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }

                // Right monocular observation
                if (bRight && i >= Nleft)
                {
                    nInitialMonoCorrespondences++;
                    pFrame->outlierFlags[i] = false;

                    kpUn = pFrame->keyPointsRight[i - Nleft];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose *e =
                        new EdgeMonoOnlyPose(pMP->getWorldPos(), 1);

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->p_camera->uncertainty2(obs);

                    const float invSigma2 =
                        pFrame->invLevelSigmaSquared[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
            }
        }
    }

    nInitialCorrespondences =
        nInitialMonoCorrespondences + nInitialStereoCorrespondences;

    // Set Previous Frame Vertex
    Frame *pFp = pFrame->p_previousFrame;

    VertexPose *VPk = new VertexPose(pFp);
    VPk->setId(4);
    VPk->setFixed(false);
    optimizer.addVertex(VPk);
    VertexVelocity *VVk = new VertexVelocity(pFp);
    VVk->setId(5);
    VVk->setFixed(false);
    optimizer.addVertex(VVk);
    VertexGyroBias *VGk = new VertexGyroBias(pFp);
    VGk->setId(6);
    VGk->setFixed(false);
    optimizer.addVertex(VGk);
    VertexAccBias *VAk = new VertexAccBias(pFp);
    VAk->setId(7);
    VAk->setFixed(false);
    optimizer.addVertex(VAk);

    EdgeInertial *ei = new EdgeInertial(pFrame->p_imuPreintegratedFrame.get());

    ei->setVertex(0, VPk);
    ei->setVertex(1, VVk);
    ei->setVertex(2, VGk);
    ei->setVertex(3, VAk);
    ei->setVertex(4, VP);
    ei->setVertex(5, VV);
    optimizer.addEdge(ei);

    EdgeGyroRW *egr = new EdgeGyroRW();
    egr->setVertex(0, VGk);
    egr->setVertex(1, VG);
    Eigen::Matrix3d InfoG = pFrame->p_imuPreintegrated->C.block<3, 3>(9, 9)
                                .cast<double>()
                                .inverse();
    egr->setInformation(InfoG);
    optimizer.addEdge(egr);

    EdgeAccRW *ear = new EdgeAccRW();
    ear->setVertex(0, VAk);
    ear->setVertex(1, VA);
    Eigen::Matrix3d InfoA = pFrame->p_imuPreintegrated->C.block<3, 3>(12, 12)
                                .cast<double>()
                                .inverse();
    ear->setInformation(InfoA);
    optimizer.addEdge(ear);

    EdgePriorPoseImu *ep = nullptr;
    if (pFp->p_poseImuConstraint)
    {
        ep = new EdgePriorPoseImu(pFp->p_poseImuConstraint);

        ep->setVertex(0, VPk);
        ep->setVertex(1, VVk);
        ep->setVertex(2, VGk);
        ep->setVertex(3, VAk);
        g2o::RobustKernelHuber *rkp = new g2o::RobustKernelHuber;
        ep->setRobustKernel(rkp);
        rkp->setDelta(5);
        optimizer.addEdge(ep);
    }
    else
    {
        Verbose::printMess(
            "pFp->p_poseImuConstraint does not exist!!!\nPrevious Frame " +
                to_string(pFp->mnId),
            Verbose::VERBOSITY_NORMAL);
    }

    // We perform 4 optimizations, after each optimization we classify
    // observation as inlier/outlier At the next optimization, outliers are not
    // included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]   = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
    const int   its[4]        = {10, 10, 10, 10};

    int nBad           = 0;
    int nBadMono       = 0;
    int nBadStereo     = 0;
    int nInliersMono   = 0;
    int nInliersStereo = 0;
    int nInliers       = 0;
    for (size_t it = 0; it < 4; it++)
    {
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad            = 0;
        nBadMono        = 0;
        nBadStereo      = 0;
        nInliers        = 0;
        nInliersMono    = 0;
        nInliersStereo  = 0;
        float chi2close = 1.5 * chi2Mono[it];

        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            EdgeMonoOnlyPose *e = vpEdgesMono[i];

            const size_t idx    = vnIndexEdgeMono[i];
            bool         bClose = pFrame->mapPoints[idx]->trackDepth < 10.f;

            if (pFrame->outlierFlags[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if ((chi2 > chi2Mono[it] && !bClose) ||
                (bClose && chi2 > chi2close) || !e->isDepthPositive())
            {
                pFrame->outlierFlags[idx] = true;
                e->setLevel(1);
                nBadMono++;
            }
            else
            {
                pFrame->outlierFlags[idx] = false;
                e->setLevel(0);
                nInliersMono++;
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            EdgeStereoOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (pFrame->outlierFlags[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[it])
            {
                pFrame->outlierFlags[idx] = true;
                e->setLevel(1);
                nBadStereo++;
            }
            else
            {
                pFrame->outlierFlags[idx] = false;
                e->setLevel(0);
                nInliersStereo++;
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        nInliers = nInliersMono + nInliersStereo;
        nBad     = nBadMono + nBadStereo;

        if (optimizer.edges().size() < 10)
        {
            break;
        }
    }

    if ((nInliers < 30) && !bRecInit)
    {
        nBad                              = 0;
        const float         chi2MonoOut   = 18.f;
        const float         chi2StereoOut = 24.f;
        EdgeMonoOnlyPose   *e1;
        EdgeStereoOnlyPose *e2;
        for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++)
        {
            const size_t idx = vnIndexEdgeMono[i];
            e1               = vpEdgesMono[i];
            e1->computeError();
            if (e1->chi2() < chi2MonoOut)
                pFrame->outlierFlags[idx] = false;
            else
                nBad++;
        }
        for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++)
        {
            const size_t idx = vnIndexEdgeStereo[i];
            e2               = vpEdgesStereo[i];
            e2->computeError();
            if (e2->chi2() < chi2StereoOut)
                pFrame->outlierFlags[idx] = false;
            else
                nBad++;
        }
    }

    nInliers = nInliersMono + nInliersStereo;

    // Recover optimized pose, velocity and biases
    pFrame->setImuPoseVelocity(VP->estimate().Rwb.cast<float>(),
                               VP->estimate().twb.cast<float>(),
                               VV->estimate().cast<float>());
    Vector6d b;
    b << VG->estimate(), VA->estimate();
    pFrame->imuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

    // Recover Hessian, marginalize previous frame states and generate new prior
    // for frame
    Eigen::Matrix<double, 30, 30> H;
    H.setZero();

    H.block<24, 24>(0, 0) += ei->getHessian();

    Eigen::Matrix<double, 6, 6> Hgr = egr->getHessian();
    H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
    H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
    H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
    H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

    Eigen::Matrix<double, 6, 6> Har = ear->getHessian();
    H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
    H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
    H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
    H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

    if (ep)
        H.block<15, 15>(0, 0) += ep->getHessian();

    int tot_in = 0, tot_out = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        EdgeMonoOnlyPose *e = vpEdgesMono[i];

        const size_t idx = vnIndexEdgeMono[i];

        if (!pFrame->outlierFlags[idx])
        {
            H.block<6, 6>(15, 15) += e->getHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
    {
        EdgeStereoOnlyPose *e = vpEdgesStereo[i];

        const size_t idx = vnIndexEdgeStereo[i];

        if (!pFrame->outlierFlags[idx])
        {
            H.block<6, 6>(15, 15) += e->getHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

    H = marginalize(H, 0, 14);

    pFrame->p_poseImuConstraint =
        new ConstraintPoseImu(VP->estimate().Rwb,
                              VP->estimate().twb,
                              VV->estimate(),
                              VG->estimate(),
                              VA->estimate(),
                              H.block<15, 15>(15, 15));
    delete pFp->p_poseImuConstraint;
    pFp->p_poseImuConstraint = nullptr;

    return nInitialCorrespondences - nBad;
}

} // namespace core
} // namespace vs_graphs
