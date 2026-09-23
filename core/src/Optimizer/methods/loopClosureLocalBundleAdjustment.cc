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

#include "OptimizableTypes.h"
#include "OptimizerEdgeLookup.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void Optimizer::loopClosureLocalBundleAdjustment(KeyFrame          *pMainKF,
                                                 vector<KeyFrame *> vpAdjustKF,
                                                 vector<KeyFrame *> vpFixedKF,
                                                 bool              *pbStopFlag)
{
    // Variables
    vector<MapPoint *>   vpMPs;
    set<KeyFrame *>      spKeyFrameBA;
    long unsigned int    maxKFid = 0;
    g2o::SparseOptimizer optimizer;

    // Define a linear solver to solve the linear system arising while
    // optimization
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
    linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // Force stop flag
    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    // Get the current map
    Map *pCurrentMap = pMainKF->getMap();

    // Set fixed KeyFrame vertices
    int numInsertedPoints = 0;
    for (KeyFrame *pKFi : vpFixedKF)
    {
        // Skip the KeyFrame if it is bad or is not in the current map
        if (pKFi->isBad() || pKFi->getMap() != pCurrentMap)
        {
            Verbose::printMess("[Error in LoopClosureLocalBundleAdjustment] "
                               "KeyFrame is bad or is not in the current map!",
                               Verbose::VERBOSITY_NORMAL);
            continue;
        }

        // Set the local BA id for the KeyFrame
        pKFi->baLocalMergeId = pMainKF->mnId;

        // Create a new vertex for the KeyFrame
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float>    Tcw  = pKFi->getPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;

        // Get the map points observed by the KeyFrame
        set<MapPoint *> spViewMPs = pKFi->getMapPoints();
        for (MapPoint *pMPi : spViewMPs)
            if (pMPi)
                if (!pMPi->isBad() && pMPi->getMap() == pCurrentMap)
                    if (pMPi->baLocalMergeId != pMainKF->mnId)
                    {
                        // Add the map point to the list of optimizable map
                        // points
                        vpMPs.push_back(pMPi);
                        pMPi->baLocalMergeId = pMainKF->mnId;
                        numInsertedPoints++;
                    }

        spKeyFrameBA.insert(pKFi);
    }

    // Set non-fixed KeyFrame vertices
    set<KeyFrame *> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
    numInsertedPoints = 0;
    for (KeyFrame *pKFi : vpAdjustKF)
    {
        if (pKFi->isBad() || pKFi->getMap() != pCurrentMap)
            continue;

        pKFi->baLocalMergeId = pMainKF->mnId;

        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float>    Tcw  = pKFi->getPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;

        set<MapPoint *> spViewMPs = pKFi->getMapPoints();
        for (MapPoint *pMPi : spViewMPs)
            if (pMPi)
                if (!pMPi->isBad() && pMPi->getMap() == pCurrentMap)
                    if (pMPi->baLocalMergeId != pMainKF->mnId)
                    {
                        vpMPs.push_back(pMPi);
                        pMPi->baLocalMergeId = pMainKF->mnId;
                        numInsertedPoints++;
                    }

        spKeyFrameBA.insert(pKFi);
    }

    const int nExpectedSize =
        (vpAdjustKF.size() + vpFixedKF.size()) * vpMPs.size();

    vector<vs_graphs::core::EdgeSE3ProjectXYZ *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    map<KeyFrame *, int> obsKeyFrames;
    map<KeyFrame *, int> obsFinalKeyFrames;
    map<MapPoint *, int> obsMapPoints;
    for (unsigned int i = 0; i < vpMPs.size(); ++i)
    {
        MapPoint *pMPi = vpMPs[i];
        if (pMPi->isBad())
            continue;

        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMPi->getWorldPos().cast<double>());
        const int id = pMPi->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame *, tuple<int, int>> observations =
            pMPi->getObservations();
        int nEdges = 0;
        // SET EDGES
        for (map<KeyFrame *, tuple<int, int>>::const_iterator mit =
                 observations.begin();
             mit != observations.end();
             mit++)
        {
            KeyFrame *pKF = mit->first;
            if (pKF->isBad() || pKF->mnId > maxKFid ||
                pKF->baLocalMergeId != pMainKF->mnId ||
                !pKF->getMapPoint(get<0>(mit->second)))
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn =
                pKF->keyPointsUndistorted[get<0>(mit->second)];

            if (pKF->uRight[get<0>(mit->second)] < 0) // Monocular
            {
                obsMapPoints[pMPi]++;
                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                vs_graphs::core::EdgeSE3ProjectXYZ *e =
                    new vs_graphs::core::EdgeSE3ProjectXYZ();

                e->setVertex(0,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(id)));
                e->setVertex(1,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->invLevelSigmaSquared[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                e->pCamera = pKF->p_camera;

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMPi);

                obsKeyFrames[pKF]++;
            }
            else // RGBD or Stereo
            {
                obsMapPoints[pMPi] += 2;
                Eigen::Matrix<double, 3, 1> obs;
                const float kp_ur = pKF->uRight[get<0>(mit->second)];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ *e =
                    new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(id)));
                e->setVertex(1,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->invLevelSigmaSquared[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber3D);

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMPi);

                obsKeyFrames[pKF]++;
            }
        }
    }

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    map<unsigned long int, int> mWrongObsKF;
    if (bDoMore)
    {
        // Check inlier observations
        int badMonoMP = 0, badStereoMP = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            vs_graphs::core::EdgeSE3ProjectXYZ *e   = vpEdgesMono[i];
            MapPoint                           *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
                badMonoMP++;
            }
            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e   = vpEdgesStereo[i];
            MapPoint                     *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
                badStereoMP++;
            }

            e->setRobustKernel(0);
        }
        Verbose::printMess("[BA]: First optimization(Huber), there are " +
                               to_string(badMonoMP) + " monocular and " +
                               to_string(badStereoMP) + " stereo bad edges",
                           Verbose::VERBOSITY_DEBUG);

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
    set<MapPoint *> spErasedMPs;
    set<KeyFrame *> spErasedKFs;

    // Check inlier observations
    int badMonoMP = 0, badStereoMP = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
    {
        vs_graphs::core::EdgeSE3ProjectXYZ *e   = vpEdgesMono[i];
        MapPoint                           *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
            mWrongObsKF[pKFi->mnId]++;
            badMonoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ *e   = vpEdgesStereo[i];
        MapPoint                     *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive())
        {
            KeyFrame *pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
            mWrongObsKF[pKFi->mnId]++;
            badStereoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

    Verbose::printMess("[BA]: Second optimization, there are " +
                           to_string(badMonoMP) + " monocular and " +
                           to_string(badStereoMP) + " sterero bad edges",
                       Verbose::VERBOSITY_DEBUG);

    // Get Map Mutex
    unique_lock<mutex> lock(pMainKF->getMap()->mMutexMapUpdate);

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
    for (unsigned int i = 0; i < vpMPs.size(); ++i)
    {
        MapPoint *pMPi = vpMPs[i];
        if (pMPi->isBad())
            continue;

        const map<KeyFrame *, tuple<int, int>> observations =
            pMPi->getObservations();
        for (map<KeyFrame *, tuple<int, int>>::const_iterator mit =
                 observations.begin();
             mit != observations.end();
             mit++)
        {
            KeyFrame *pKF = mit->first;
            if (pKF->isBad() || pKF->mnId > maxKFid ||
                pKF->baLocalKeyFrameId != pMainKF->mnId ||
                !pKF->getMapPoint(get<0>(mit->second)))
                continue;

            if (pKF->uRight[get<0>(mit->second)] < 0) // Monocular
            {
                obsFinalKeyFrames[pKF]++;
            }
            else // RGBD or Stereo
            {
                obsFinalKeyFrames[pKF]++;
            }
        }
    }

    // Recover optimized data
    // Keyframes
    for (KeyFrame *pKFi : vpAdjustKF)
    {
        if (pKFi->isBad())
            continue;

        g2o::VertexSE3Expmap *vSE3 =
            static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(),
                         SE3quat.translation().cast<float>());

        int                numMonoBadPoints = 0, numMonoOptPoints = 0;
        int                numStereoBadPoints = 0, numStereoOptPoints = 0;
        vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;
        vector<MapPoint *> vpMonoMPsBad, vpStereoMPsBad;

        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            vs_graphs::core::EdgeSE3ProjectXYZ *e   = vpEdgesMono[i];
            MapPoint                           *pMP = vpMapPointEdgeMono[i];
            KeyFrame *pKFedge = edgeSourceKeyFrame(vpEdgeKFMono, i);

            if (pKFedge == nullptr || pKFi != pKFedge)
            {
                continue;
            }

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                numMonoBadPoints++;
                vpMonoMPsBad.push_back(pMP);
            }
            else
            {
                numMonoOptPoints++;
                vpMonoMPsOpt.push_back(pMP);
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e   = vpEdgesStereo[i];
            MapPoint                     *pMP = vpMapPointEdgeStereo[i];
            KeyFrame *pKFedge = edgeSourceKeyFrame(vpEdgeKFStereo, i);

            if (pKFedge == nullptr || pKFi != pKFedge)
            {
                continue;
            }

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                numStereoBadPoints++;
                vpStereoMPsBad.push_back(pMP);
            }
            else
            {
                numStereoOptPoints++;
                vpStereoMPsOpt.push_back(pMP);
            }
        }

        pKFi->setPose(Tiw);
    }

    // Points
    for (MapPoint *pMPi : vpMPs)
    {
        if (pMPi->isBad())
            continue;

        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(
            optimizer.vertex(pMPi->mnId + maxKFid + 1));
        pMPi->setWorldPos(vPoint->estimate().cast<float>());
        pMPi->updateNormalAndDepth();
    }
}

} // namespace core
} // namespace vs_graphs
