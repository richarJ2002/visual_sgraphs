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

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{

void Optimizer::fullInertialBA(Map                              *pMap,
                               int                               its,
                               const bool                        bFixLocal,
                               const long unsigned int           nLoopId,
                               bool                             *pbStopFlag,
                               bool                              bInit,
                               float                             priorG,
                               float                             priorA,
                               [[maybe_unused]] Eigen::VectorXd *vSingVal,
                               [[maybe_unused]] bool            *bHess,
                               const std::atomic_bool *pStopRequested_in)
{
    long unsigned int        maxKFid = pMap->getMaxKeyFrameId();
    const vector<KeyFrame *> vpKFs   = pMap->getAllKeyFrames();
    const vector<MapPoint *> vpMPs   = pMap->getAllMapPoints();

    if (vpKFs.empty())
    {
        return;
    }

    AtomicOptimizerStopBridge stopBridge(pStopRequested_in, pbStopFlag);

    // Setup optimizer
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    if (pStopRequested_in != nullptr && pbStopFlag != nullptr)
    {
        optimizer.addPreIterationAction(&stopBridge);
        optimizer.addPostIterationAction(&stopBridge);
    }

    int nNonFixed = 0;

    // Set KeyFrame vertices
    KeyFrame *p_initializationKeyFrame = nullptr;
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;
        VertexPose *VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        p_initializationKeyFrame = pKFi;
        bool bFixed              = false;
        if (bFixLocal)
        {
            bFixed = (pKFi->baLocalKeyFrameId >= (maxKFid - 1)) ||
                     (pKFi->baFixedKeyFrameId >= (maxKFid - 1));
            if (!bFixed)
                nNonFixed++;
            VP->setFixed(bFixed);
        }
        if (i == 0) // Fix the first keyframe at origin
            VP->setFixed(true);
        optimizer.addVertex(VP);

        if (pKFi->isImu)
        {
            VertexVelocity *VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(bFixed);
            optimizer.addVertex(VV);
            if (!bInit)
            {
                VertexGyroBias *VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
                VG->setFixed(bFixed);
                optimizer.addVertex(VG);
                VertexAccBias *VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
                VA->setFixed(bFixed);
                optimizer.addVertex(VA);
            }
        }
    }

    if (bInit)
    {
        if (p_initializationKeyFrame == nullptr)
        {
            return;
        }

        VertexGyroBias *VG = new VertexGyroBias(p_initializationKeyFrame);
        VG->setId(4 * maxKFid + 2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        VertexAccBias *VA = new VertexAccBias(p_initializationKeyFrame);
        VA->setId(4 * maxKFid + 3);
        VA->setFixed(false);
        optimizer.addVertex(VA);
    }

    if (bFixLocal)
    {
        if (nNonFixed < 3)
            return;
    }

    // IMU links
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKFi = vpKFs[i];

        if (!pKFi->p_prevKF)
        {
            Verbose::printMess("NOT INERTIAL LINK TO PREVIOUS FRAME!",
                               Verbose::VERBOSITY_NORMAL);
            continue;
        }

        if (pKFi->p_prevKF && pKFi->mnId <= maxKFid)
        {
            if (pKFi->isBad() || pKFi->p_prevKF->mnId > maxKFid)
                continue;
            if (pKFi->isImu && pKFi->p_prevKF->isImu)
            {
                pKFi->p_imuPreintegrated->setNewBias(
                    pKFi->p_prevKF->getImuBias());
                g2o::HyperGraph::Vertex *VP1 =
                    optimizer.vertex(pKFi->p_prevKF->mnId);
                g2o::HyperGraph::Vertex *VV1 =
                    optimizer.vertex(maxKFid + 3 * (pKFi->p_prevKF->mnId) + 1);

                g2o::HyperGraph::Vertex *VG1;
                g2o::HyperGraph::Vertex *VA1;
                g2o::HyperGraph::Vertex *VG2;
                g2o::HyperGraph::Vertex *VA2;
                if (!bInit)
                {
                    VG1 = optimizer.vertex(maxKFid +
                                           3 * (pKFi->p_prevKF->mnId) + 2);
                    VA1 = optimizer.vertex(maxKFid +
                                           3 * (pKFi->p_prevKF->mnId) + 3);
                    VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
                    VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);
                }
                else
                {
                    VG1 = optimizer.vertex(4 * maxKFid + 2);
                    VA1 = optimizer.vertex(4 * maxKFid + 3);
                }

                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex *VV2 =
                    optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);

                if (!bInit)
                {
                    if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 ||
                        !VA2)
                    {
                        cout << "Error" << VP1 << ", " << VV1 << ", " << VG1
                             << ", " << VA1 << ", " << VP2 << ", " << VV2
                             << ", " << VG2 << ", " << VA2 << endl;
                        continue;
                    }
                }
                else
                {
                    if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2)
                    {
                        cout << "Error" << VP1 << ", " << VV1 << ", " << VG1
                             << ", " << VA1 << ", " << VP2 << ", " << VV2
                             << endl;
                        continue;
                    }
                }

                EdgeInertial *ei = new EdgeInertial(pKFi->p_imuPreintegrated);
                ei->setVertex(
                    0,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                ei->setVertex(
                    1,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                ei->setVertex(
                    2,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
                ei->setVertex(
                    3,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
                ei->setVertex(
                    4,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                ei->setVertex(
                    5,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));

                g2o::RobustKernelHuber *rki = new g2o::RobustKernelHuber;
                ei->setRobustKernel(rki);
                rki->setDelta(sqrt(16.92));

                optimizer.addEdge(ei);

                if (!bInit)
                {
                    EdgeGyroRW *egr = new EdgeGyroRW();
                    egr->setVertex(0, VG1);
                    egr->setVertex(1, VG2);
                    Eigen::Matrix3d InfoG =
                        pKFi->p_imuPreintegrated->C.block<3, 3>(9, 9)
                            .cast<double>()
                            .inverse();
                    egr->setInformation(InfoG);
                    egr->computeError();
                    optimizer.addEdge(egr);

                    EdgeAccRW *ear = new EdgeAccRW();
                    ear->setVertex(0, VA1);
                    ear->setVertex(1, VA2);
                    Eigen::Matrix3d InfoA =
                        pKFi->p_imuPreintegrated->C.block<3, 3>(12, 12)
                            .cast<double>()
                            .inverse();
                    ear->setInformation(InfoA);
                    ear->computeError();
                    optimizer.addEdge(ear);
                }
            }
            else
                cout << pKFi->mnId << " or " << pKFi->p_prevKF->mnId
                     << " no imu" << endl;
        }
    }

    if (bInit)
    {
        g2o::HyperGraph::Vertex *VG = optimizer.vertex(4 * maxKFid + 2);
        g2o::HyperGraph::Vertex *VA = optimizer.vertex(4 * maxKFid + 3);

        // Add prior to comon biases
        Eigen::Vector3f bprior;
        bprior.setZero();

        EdgePriorAcc *epa = new EdgePriorAcc(bprior);
        epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
        double infoPriorA = priorA; //
        epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);

        EdgePriorGyro *epg = new EdgePriorGyro(bprior);
        epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
        double infoPriorG = priorG; //
        epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);
    }

    const float thHuberMono   = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    const unsigned long iniMPid = maxKFid * 5;

    vector<bool> vbNotIncludedMP(vpMPs.size(), false);

    for (size_t i = 0; i < vpMPs.size(); i++)
    {
        MapPoint               *pMP    = vpMPs[i];
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->getWorldPos().cast<double>());
        unsigned long id = pMP->mnId + iniMPid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame *, tuple<int, int>> observations =
            pMP->getObservations();

        bool bAllFixed = true;

        // Set edges
        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnId > maxKFid)
                continue;

            if (!pKFi->isBad())
            {
                const int    leftIndex = get<0>(mit->second);
                cv::KeyPoint kpUn;

                if (leftIndex != -1 && pKFi->uRight[get<0>(mit->second)] <
                                           0) // Monocular observation
                {
                    kpUn = pKFi->keyPointsUndistorted[leftIndex];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono *e = new EdgeMono(0);

                    g2o::OptimizableGraph::Vertex *VP =
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(pKFi->mnId));
                    if (bAllFixed)
                        if (!VP->fixed())
                            bAllFixed = false;

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 =
                        pKFi->invLevelSigmaSquared[kpUn.octave];

                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                }
                else if (leftIndex != -1 &&
                         pKFi->uRight[leftIndex] >= 0) // stereo observation
                {
                    kpUn = pKFi->keyPointsUndistorted[leftIndex];
                    const float                 kp_ur = pKFi->uRight[leftIndex];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo *e = new EdgeStereo(0);

                    g2o::OptimizableGraph::Vertex *VP =
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(pKFi->mnId));
                    if (bAllFixed)
                        if (!VP->fixed())
                            bAllFixed = false;

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 =
                        pKFi->invLevelSigmaSquared[kpUn.octave];

                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                }

                if (pKFi->p_camera2)
                { // Monocular right observation
                    int rightIndex = get<1>(mit->second);

                    if (rightIndex != -1 && static_cast<size_t>(rightIndex) <
                                                pKFi->keyPointsRight.size())
                    {
                        rightIndex -= pKFi->Nleft;

                        Eigen::Matrix<double, 2, 1> obs;
                        kpUn = pKFi->keyPointsRight[rightIndex];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMono *e = new EdgeMono(1);

                        g2o::OptimizableGraph::Vertex *VP =
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(pKFi->mnId));
                        if (bAllFixed)
                            if (!VP->fixed())
                                bAllFixed = false;

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(id)));
                        e->setVertex(1, VP);
                        e->setMeasurement(obs);
                        const float invSigma2 =
                            pKFi->invLevelSigmaSquared[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                    }
                }
            }
        }

        if (bAllFixed)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i] = true;
        }
    }

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(its);
    optimizer.removePreIterationAction(&stopBridge);
    optimizer.removePostIterationAction(&stopBridge);

    // Recover optimized data
    // Keyframes
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;
        VertexPose *VP =
            static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
        if (nLoopId == 0)
        {
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                             VP->estimate().tcw[0].cast<float>());
            pKFi->setPose(Tcw);
        }
        else
        {
            pKFi->tcwGBA = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(),
                                        VP->estimate().tcw[0].cast<float>());
            pKFi->baGlobalKeyFrameId = nLoopId;
        }
        if (pKFi->isImu)
        {
            VertexVelocity *VV = static_cast<VertexVelocity *>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
            if (nLoopId == 0)
            {
                pKFi->setVelocity(VV->estimate().cast<float>());
            }
            else
            {
                pKFi->vwbGBA = VV->estimate().cast<float>();
            }

            VertexGyroBias *VG;
            VertexAccBias  *VA;
            if (!bInit)
            {
                VG = static_cast<VertexGyroBias *>(
                    optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
                VA = static_cast<VertexAccBias *>(
                    optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
            }
            else
            {
                VG = static_cast<VertexGyroBias *>(
                    optimizer.vertex(4 * maxKFid + 2));
                VA = static_cast<VertexAccBias *>(
                    optimizer.vertex(4 * maxKFid + 3));
            }

            Vector6d vb;
            vb << VG->estimate(), VA->estimate();
            IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
            if (nLoopId == 0)
            {
                pKFi->setNewBias(b);
            }
            else
            {
                pKFi->biasGBA = b;
            }
        }
    }

    // Points
    for (size_t i = 0; i < vpMPs.size(); i++)
    {
        if (vbNotIncludedMP[i])
            continue;

        MapPoint               *pMP    = vpMPs[i];
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(
            optimizer.vertex(pMP->mnId + iniMPid + 1));

        if (nLoopId == 0)
        {
            pMP->setWorldPos(vPoint->estimate().cast<float>());
            pMP->updateNormalAndDepth();
        }
        else
        {
            pMP->posGBA             = vPoint->estimate().cast<float>();
            pMP->baGlobalKeyFrameId = nLoopId;
        }
    }

    pMap->increaseChangeIndex();
}

} // namespace core
} // namespace vs_graphs
