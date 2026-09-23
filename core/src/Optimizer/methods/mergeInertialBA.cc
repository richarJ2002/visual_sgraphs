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

#include <mutex>

namespace vs_graphs
{
namespace core
{

void Optimizer::mergeInertialBA(KeyFrame                     *pCurrKF,
                                KeyFrame                     *pMergeKF,
                                bool                         *pbStopFlag,
                                Map                          *pMap,
                                LoopClosing::KeyFrameAndPose &corrPoses)
{
    const int           Nd      = 6;
    const unsigned long maxKFid = pCurrKF->mnId;

    vector<KeyFrame *> vpOptimizableKFs;
    vpOptimizableKFs.reserve(2 * Nd);

    // For cov KFS, inertial parameters are not optimized
    const int          maxCovKF = 30;
    vector<KeyFrame *> vpOptimizableCovKFs;
    vpOptimizableCovKFs.reserve(maxCovKF);

    // Add sliding window for current KF
    vpOptimizableKFs.push_back(pCurrKF);
    pCurrKF->baLocalKeyFrameId = pCurrKF->mnId;
    for (int i = 1; i < Nd; i++)
    {
        if (vpOptimizableKFs.back()->p_prevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->p_prevKF);
            vpOptimizableKFs.back()->baLocalKeyFrameId = pCurrKF->mnId;
        }
        else
            break;
    }

    list<KeyFrame *> lFixedKeyFrames;
    if (vpOptimizableKFs.back()->p_prevKF)
    {
        vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->p_prevKF);
        vpOptimizableKFs.back()->p_prevKF->baLocalKeyFrameId = pCurrKF->mnId;
    }
    else
    {
        vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Add temporal neighbours to merge KF (previous and next KFs)
    vpOptimizableKFs.push_back(pMergeKF);
    pMergeKF->baLocalKeyFrameId = pCurrKF->mnId;

    // Previous KFs
    for (int i = 1; i < (Nd / 2); i++)
    {
        if (vpOptimizableKFs.back()->p_prevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->p_prevKF);
            vpOptimizableKFs.back()->baLocalKeyFrameId = pCurrKF->mnId;
        }
        else
            break;
    }

    // We fix just once the old map
    if (vpOptimizableKFs.back()->p_prevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->p_prevKF);
        vpOptimizableKFs.back()->p_prevKF->baFixedKeyFrameId = pCurrKF->mnId;
    }
    else
    {
        vpOptimizableKFs.back()->baLocalKeyFrameId = 0;
        vpOptimizableKFs.back()->baFixedKeyFrameId = pCurrKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Next KFs
    if (pMergeKF->p_nextKF)
    {
        vpOptimizableKFs.push_back(pMergeKF->p_nextKF);
        vpOptimizableKFs.back()->baLocalKeyFrameId = pCurrKF->mnId;
    }

    while (vpOptimizableKFs.size() < (2 * Nd))
    {
        if (vpOptimizableKFs.back()->p_nextKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->p_nextKF);
            vpOptimizableKFs.back()->baLocalKeyFrameId = pCurrKF->mnId;
        }
        else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by optimizable keyframes
    list<MapPoint *>     localMapPointList;
    map<MapPoint *, int> localObservationCounts;
    for (int i = 0; i < N; i++)
    {
        vector<MapPoint *> vpMPs = vpOptimizableKFs[i]->getMapPointMatches();
        for (vector<MapPoint *>::iterator vit  = vpMPs.begin(),
                                          vend = vpMPs.end();
             vit != vend;
             vit++)
        {
            // Using mnBALocalForKF we avoid redundance here, one MP can not be
            // added several times to localMapPointList
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                {
                    if (pMP->baLocalKeyFrameId != pCurrKF->mnId)
                    {
                        localObservationCounts[pMP] = 1;
                        localMapPointList.push_back(pMP);
                        pMP->baLocalKeyFrameId = pCurrKF->mnId;
                    }
                    else
                    {
                        localObservationCounts[pMP]++;
                    }
                }
        }
    }

    std::vector<std::pair<MapPoint *, int>> pairs;
    pairs.reserve(localObservationCounts.size());
    for (auto itr = localObservationCounts.begin();
         itr != localObservationCounts.end();
         ++itr)
        pairs.push_back(*itr);
    sort(pairs.begin(), pairs.end(), sortByVal);

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not
    // Local Keyframes
    int i = 0;
    for (vector<pair<MapPoint *, int>>::iterator lit  = pairs.begin(),
                                                 lend = pairs.end();
         lit != lend;
         lit++, i++)
    {
        map<KeyFrame *, tuple<int, int>> observations =
            lit->first->getObservations();
        if (i >= maxCovKF)
            break;
        for (map<KeyFrame *, tuple<int, int>>::iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->baLocalKeyFrameId != pCurrKF->mnId &&
                pKFi->baFixedKeyFrameId !=
                    pCurrKF->mnId) // If optimizable or already included...
            {
                pKFi->baLocalKeyFrameId = pCurrKF->mnId;
                if (!pKFi->isBad())
                {
                    vpOptimizableCovKFs.push_back(pKFi);
                    break;
                }
            }
        }
    }

    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // Set Local KeyFrame vertices
    N = vpOptimizableKFs.size();
    for (int i = 0; i < N; i++)
    {
        KeyFrame *pKFi = vpOptimizableKFs[i];

        VertexPose *VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if (pKFi->isImu)
        {
            VertexVelocity *VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias *VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias *VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Local cov keyframes vertices
    int Ncov = vpOptimizableCovKFs.size();
    for (int i = 0; i < Ncov; i++)
    {
        KeyFrame *pKFi = vpOptimizableCovKFs[i];

        VertexPose *VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if (pKFi->isImu)
        {
            VertexVelocity *VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias *VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias *VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Fixed KeyFrame vertices
    for (list<KeyFrame *>::iterator lit  = lFixedKeyFrames.begin(),
                                    lend = lFixedKeyFrames.end();
         lit != lend;
         lit++)
    {
        KeyFrame   *pKFi = *lit;
        VertexPose *VP   = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        if (pKFi->isImu)
        {
            VertexVelocity *VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
            VV->setFixed(true);
            optimizer.addVertex(VV);
            VertexGyroBias *VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias *VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }
    }

    // Create intertial constraints
    vector<EdgeInertial *> vei(N, (EdgeInertial *)nullptr);
    vector<EdgeGyroRW *>   vegr(N, (EdgeGyroRW *)nullptr);
    vector<EdgeAccRW *>    vear(N, (EdgeAccRW *)nullptr);
    for (int i = 0; i < N; i++)
    {
        // cout << "inserting inertial edge " << i << endl;
        KeyFrame *pKFi = vpOptimizableKFs[i];

        if (!pKFi->p_prevKF)
        {
            Verbose::printMess("NOT INERTIAL LINK TO PREVIOUS FRAME!!!!",
                               Verbose::VERBOSITY_NORMAL);
            continue;
        }
        if (pKFi->isImu && pKFi->p_prevKF->isImu && pKFi->p_imuPreintegrated)
        {
            pKFi->p_imuPreintegrated->setNewBias(pKFi->p_prevKF->getImuBias());
            g2o::HyperGraph::Vertex *VP1 =
                optimizer.vertex(pKFi->p_prevKF->mnId);
            g2o::HyperGraph::Vertex *VV1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->p_prevKF->mnId) + 1);
            g2o::HyperGraph::Vertex *VG1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->p_prevKF->mnId) + 2);
            g2o::HyperGraph::Vertex *VA1 =
                optimizer.vertex(maxKFid + 3 * (pKFi->p_prevKF->mnId) + 3);
            g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex *VV2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
            g2o::HyperGraph::Vertex *VG2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
            g2o::HyperGraph::Vertex *VA2 =
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

            if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
            {
                cerr << "Error " << VP1 << ", " << VV1 << ", " << VG1 << ", "
                     << VA1 << ", " << VP2 << ", " << VV2 << ", " << VG2 << ", "
                     << VA2 << endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->p_imuPreintegrated);

            vei[i]->setVertex(
                0,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            vei[i]->setVertex(
                1,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            vei[i]->setVertex(
                2,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG1));
            vei[i]->setVertex(
                3,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA1));
            vei[i]->setVertex(
                4,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            vei[i]->setVertex(
                5,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));

            // TODO Uncomment
            g2o::RobustKernelHuber *rki = new g2o::RobustKernelHuber;
            vei[i]->setRobustKernel(rki);
            rki->setDelta(sqrt(16.92));
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0, VG1);
            vegr[i]->setVertex(1, VG2);
            Eigen::Matrix3d InfoG =
                pKFi->p_imuPreintegrated->C.block<3, 3>(9, 9)
                    .cast<double>()
                    .inverse();
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0, VA1);
            vear[i]->setVertex(1, VA2);
            Eigen::Matrix3d InfoA =
                pKFi->p_imuPreintegrated->C.block<3, 3>(12, 12)
                    .cast<double>()
                    .inverse();
            vear[i]->setInformation(InfoA);
            optimizer.addEdge(vear[i]);
        }
        else
            Verbose::printMess("ERROR building inertial edge",
                               Verbose::VERBOSITY_NORMAL);
    }

    Verbose::printMess("end inserting inertial edges",
                       Verbose::VERBOSITY_NORMAL);

    // Set MapPoint vertices
    const int nExpectedSize =
        (N + Ncov + lFixedKeyFrames.size()) * localMapPointList.size();

    // Mono
    vector<EdgeMono *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    // Stereo
    vector<EdgeStereo *> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono   = sqrt(5.991);
    const float chi2Mono2     = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2   = 7.815;

    const unsigned long iniMPid = maxKFid * 5;

    for (list<MapPoint *>::iterator lit  = localMapPointList.begin(),
                                    lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        MapPoint *pMP = *lit;
        if (!pMP)
            continue;

        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->getWorldPos().cast<double>());

        unsigned long id = pMP->mnId + iniMPid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame *, tuple<int, int>> observations =
            pMP->getObservations();

        // Create visual constraints
        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (!pKFi)
                continue;

            if ((pKFi->baLocalKeyFrameId != pCurrKF->mnId) &&
                (pKFi->baFixedKeyFrameId != pCurrKF->mnId))
                continue;

            if (pKFi->mnId > maxKFid)
            {
                continue;
            }

            if (optimizer.vertex(id) == nullptr ||
                optimizer.vertex(pKFi->mnId) == nullptr)
                continue;

            if (!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn =
                    pKFi->keyPointsUndistorted[get<0>(mit->second)];

                if (pKFi->uRight[get<0>(mit->second)] <
                    0) // Monocular observation
                {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono *e = new EdgeMono();
                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 =
                        pKFi->invLevelSigmaSquared[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);
                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // stereo observation
                {
                    const float kp_ur = pKFi->uRight[get<0>(mit->second)];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo *e = new EdgeStereo();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 =
                        pKFi->invLevelSigmaSquared[kpUn.octave];
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(8);

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        EdgeMono *e   = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > chi2Mono2)
        {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // Stereo
    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
    {
        EdgeStereo *e   = vpEdgesStereo[i];
        MapPoint   *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > chi2Stereo2)
        {
            KeyFrame *pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // Get Map Mutex and erase outliers
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);
    if (!vToErase.empty())
    {
        for (size_t i = 0; i < vToErase.size(); i++)
        {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->eraseMapPointMatch(pMPi);
            pMPi->eraseObservation(pKFi);
        }
    }

    // Recover optimized data
    // Keyframes
    for (int i = 0; i < N; i++)
    {
        KeyFrame *pKFi = vpOptimizableKFs[i];

        VertexPose *VP =
            static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                         VP->estimate().tcw[0].cast<float>());
        pKFi->setPose(Tcw);

        Sophus::SE3d Tiw = pKFi->getPose().cast<double>();
        g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
        corrPoses[pKFi] = g2oSiw;

        if (pKFi->isImu)
        {
            VertexVelocity *VV = static_cast<VertexVelocity *>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
            pKFi->setVelocity(VV->estimate().cast<float>());
            VertexGyroBias *VG = static_cast<VertexGyroBias *>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
            VertexAccBias *VA = static_cast<VertexAccBias *>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->setNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
        }
    }

    for (int i = 0; i < Ncov; i++)
    {
        KeyFrame *pKFi = vpOptimizableCovKFs[i];

        VertexPose *VP =
            static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                         VP->estimate().tcw[0].cast<float>());
        pKFi->setPose(Tcw);

        Sophus::SE3d Tiw = pKFi->getPose().cast<double>();
        g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
        corrPoses[pKFi] = g2oSiw;

        if (pKFi->isImu)
        {
            VertexVelocity *VV = static_cast<VertexVelocity *>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
            pKFi->setVelocity(VV->estimate().cast<float>());
            VertexGyroBias *VG = static_cast<VertexGyroBias *>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
            VertexAccBias *VA = static_cast<VertexAccBias *>(
                optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->setNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
        }
    }

    // Points
    for (list<MapPoint *>::iterator lit  = localMapPointList.begin(),
                                    lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        MapPoint               *pMP    = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(
            optimizer.vertex(pMP->mnId + iniMPid + 1));
        pMP->setWorldPos(vPoint->estimate().cast<float>());
        pMP->updateNormalAndDepth();
    }

    pMap->increaseChangeIndex();
}

} // namespace core
} // namespace vs_graphs
