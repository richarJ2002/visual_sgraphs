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

void Optimizer::localInertialBA(KeyFrame *pKF,
                                bool     *pbStopFlag,
                                Map      *pMap,
                                int      &countFixedKF,
                                int      &num_OptKF,
                                int      &num_MPs,
                                int      &num_edges,
                                bool      bLarge,
                                bool      bRecInit)
{
    Map *pCurrentMap = pKF->getMap();

    int maxOpt = 10;
    int opt_it = 10;
    if (bLarge)
    {
        maxOpt = 25;
        opt_it = 4;
    }
    const int Nd = std::min((int)pCurrentMap->getKeyFrameCount() - 2, maxOpt);
    const unsigned long maxKFid = pKF->mnId;

    vector<KeyFrame *>       vpOptimizableKFs;
    const vector<KeyFrame *> vpNeighsKFs = pKF->getVectorCovisibleKeyFrames();
    list<KeyFrame *>         lpOptVisKFs;

    vpOptimizableKFs.reserve(Nd);
    vpOptimizableKFs.push_back(pKF);
    pKF->baLocalKeyFrameId = pKF->mnId;
    for (int i = 1; i < Nd; i++)
    {
        if (vpOptimizableKFs.back()->p_prevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->p_prevKF);
            vpOptimizableKFs.back()->baLocalKeyFrameId = pKF->mnId;
        }
        else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by temporal optimizable keyframes
    list<MapPoint *> localMapPointList;
    for (int i = 0; i < N; i++)
    {
        vector<MapPoint *> vpMPs = vpOptimizableKFs[i]->getMapPointMatches();
        for (vector<MapPoint *>::iterator vit  = vpMPs.begin(),
                                          vend = vpMPs.end();
             vit != vend;
             vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->baLocalKeyFrameId != pKF->mnId)
                    {
                        localMapPointList.push_back(pMP);
                        pMP->baLocalKeyFrameId = pKF->mnId;
                    }
        }
    }

    // Fixed Keyframe: First frame previous KF to optimization window)
    list<KeyFrame *> lFixedKeyFrames;
    if (vpOptimizableKFs.back()->p_prevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->p_prevKF);
        vpOptimizableKFs.back()->p_prevKF->baFixedKeyFrameId = pKF->mnId;
    }
    else
    {
        vpOptimizableKFs.back()->baLocalKeyFrameId = 0;
        vpOptimizableKFs.back()->baFixedKeyFrameId = pKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Optimizable visual KFs
    const int maxCovKF = 0;
    for (int i = 0, iend = vpNeighsKFs.size(); i < iend; i++)
    {
        if (lpOptVisKFs.size() >= maxCovKF)
            break;

        KeyFrame *pKFi = vpNeighsKFs[i];
        if (pKFi->baLocalKeyFrameId == pKF->mnId ||
            pKFi->baFixedKeyFrameId == pKF->mnId)
            continue;
        pKFi->baLocalKeyFrameId = pKF->mnId;
        if (!pKFi->isBad() && pKFi->getMap() == pCurrentMap)
        {
            lpOptVisKFs.push_back(pKFi);

            vector<MapPoint *> vpMPs = pKFi->getMapPointMatches();
            for (vector<MapPoint *>::iterator vit  = vpMPs.begin(),
                                              vend = vpMPs.end();
                 vit != vend;
                 vit++)
            {
                MapPoint *pMP = *vit;
                if (pMP)
                    if (!pMP->isBad())
                        if (pMP->baLocalKeyFrameId != pKF->mnId)
                        {
                            localMapPointList.push_back(pMP);
                            pMP->baLocalKeyFrameId = pKF->mnId;
                        }
            }
        }
    }

    // Fixed KFs which are not covisible optimizable
    const int maxFixKF = 200;

    for (list<MapPoint *>::iterator lit  = localMapPointList.begin(),
                                    lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        map<KeyFrame *, tuple<int, int>> observations =
            (*lit)->getObservations();
        for (map<KeyFrame *, tuple<int, int>>::iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            KeyFrame *pKFi = mit->first;

            if (pKFi->baLocalKeyFrameId != pKF->mnId &&
                pKFi->baFixedKeyFrameId != pKF->mnId)
            {
                pKFi->baFixedKeyFrameId = pKF->mnId;
                if (!pKFi->isBad())
                {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        if (lFixedKeyFrames.size() >= maxFixKF)
            break;
    }

    // Setup optimizer
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    if (bLarge)
    {
        g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(
            1e-2); // to avoid iterating for finding optimal lambda
        optimizer.setAlgorithm(solver);
    }
    else
    {
        g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e0);
        optimizer.setAlgorithm(solver);
    }

    // Set Local temporal KeyFrame vertices
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

    // Set Local visual KeyFrame vertices
    for (list<KeyFrame *>::iterator it    = lpOptVisKFs.begin(),
                                    itEnd = lpOptVisKFs.end();
         it != itEnd;
         it++)
    {
        KeyFrame   *pKFi = *it;
        VertexPose *VP   = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);
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

        if (pKFi->isImu) // This should be done only for keyframe just before
                         // temporal window
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
        KeyFrame *pKFi = vpOptimizableKFs[i];

        if (!pKFi->p_prevKF)
        {
            cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
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

            if (i == N - 1 || bRecInit)
            {
                // All inertial residuals are included without robust cost
                // function, but not that one linking the last optimizable
                // keyframe inside of the local window and the first fixed
                // keyframe out. The information matrix for this measurement is
                // also downweighted. This is done to avoid accumulating error
                // due to fixing variables.
                g2o::RobustKernelHuber *rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                if (i == N - 1)
                    vei[i]->setInformation(vei[i]->information() * 1e-2);
                rki->setDelta(sqrt(16.92));
            }
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
            cout << "ERROR building inertial edge" << endl;
    }

    // Set MapPoint vertices
    const int nExpectedSize =
        (N + lFixedKeyFrames.size()) * localMapPointList.size();

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

    map<int, int> visibleEdgeCounts;
    for (int i = 0; i < N; i++)
    {
        KeyFrame *pKFi                = vpOptimizableKFs[i];
        visibleEdgeCounts[pKFi->mnId] = 0;
    }
    for (list<KeyFrame *>::iterator lit  = lFixedKeyFrames.begin(),
                                    lend = lFixedKeyFrames.end();
         lit != lend;
         lit++)
    {
        visibleEdgeCounts[(*lit)->mnId] = 0;
    }

    for (list<MapPoint *>::iterator lit  = localMapPointList.begin(),
                                    lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        MapPoint               *pMP    = *lit;
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

            if (pKFi->baLocalKeyFrameId != pKF->mnId &&
                pKFi->baFixedKeyFrameId != pKF->mnId)
                continue;

            if (!pKFi->isBad() && pKFi->getMap() == pCurrentMap)
            {
                const int leftIndex = get<0>(mit->second);

                cv::KeyPoint kpUn;

                // Monocular left observation
                if (leftIndex != -1 && pKFi->uRight[leftIndex] < 0)
                {
                    visibleEdgeCounts[pKFi->mnId]++;

                    kpUn = pKFi->keyPointsUndistorted[leftIndex];
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono *e = new EdgeMono(0);

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pKFi->p_camera->uncertainty2(obs);

                    const float &invSigma2 =
                        pKFi->invLevelSigmaSquared[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                // Stereo-observation
                else if (leftIndex != -1) // Stereo observation
                {
                    kpUn = pKFi->keyPointsUndistorted[leftIndex];
                    visibleEdgeCounts[pKFi->mnId]++;

                    const float                 kp_ur = pKFi->uRight[leftIndex];
                    Eigen::Matrix<double, 3, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo *e = new EdgeStereo(0);

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 =
                        pKFi->p_camera->uncertainty2(obs.head(2));

                    const float &invSigma2 =
                        pKFi->invLevelSigmaSquared[kpUn.octave] / unc2;
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }

                // Monocular right observation
                if (pKFi->p_camera2)
                {
                    int rightIndex = get<1>(mit->second);

                    if (rightIndex != -1 &&
                        rightIndex < (int)pKFi->keyPointsRight.size())
                    {
                        rightIndex -= pKFi->Nleft;
                        visibleEdgeCounts[pKFi->mnId]++;

                        Eigen::Matrix<double, 2, 1> obs;
                        cv::KeyPoint kp = pKFi->keyPointsRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        EdgeMono *e = new EdgeMono(1);

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(id)));
                        e->setVertex(
                            1,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->p_camera->uncertainty2(obs);

                        const float &invSigma2 =
                            pKFi->invLevelSigmaSquared[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                }
            }
        }
    }

    // cout << "Total map points: " << localMapPointList.size() << endl;
    for (map<int, int>::iterator mit  = visibleEdgeCounts.begin(),
                                 mend = visibleEdgeCounts.end();
         mit != mend;
         mit++)
    {
        assert(mit->second >= 3);
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    float err = optimizer.activeRobustChi2();
    optimizer.optimize(opt_it); // Originally to 2
    float err_end = optimizer.activeRobustChi2();
    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        EdgeMono *e      = vpEdgesMono[i];
        MapPoint *pMP    = vpMapPointEdgeMono[i];
        bool      bClose = pMP->trackDepth < 10.f;

        if (pMP->isBad())
            continue;

        if ((e->chi2() > chi2Mono2 && !bClose) ||
            (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive())
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

    // TODO: Some convergence problems have been detected here
    if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge) // bGN)
    {
        cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
        return;
    }

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

    for (list<KeyFrame *>::iterator lit  = lFixedKeyFrames.begin(),
                                    lend = lFixedKeyFrames.end();
         lit != lend;
         lit++)
        (*lit)->baFixedKeyFrameId = 0;

    // Recover optimized data
    // Local temporal Keyframes
    N = vpOptimizableKFs.size();
    for (int i = 0; i < N; i++)
    {
        KeyFrame *pKFi = vpOptimizableKFs[i];

        VertexPose *VP =
            static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                         VP->estimate().tcw[0].cast<float>());
        pKFi->setPose(Tcw);
        pKFi->baLocalKeyFrameId = 0;

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

    // Local visual KeyFrame
    for (list<KeyFrame *>::iterator it    = lpOptVisKFs.begin(),
                                    itEnd = lpOptVisKFs.end();
         it != itEnd;
         it++)
    {
        KeyFrame   *pKFi = *it;
        VertexPose *VP =
            static_cast<VertexPose *>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(),
                         VP->estimate().tcw[0].cast<float>());
        pKFi->setPose(Tcw);
        pKFi->baLocalKeyFrameId = 0;
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
