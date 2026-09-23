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

#include <mutex>

namespace vs_graphs
{
namespace core
{

void Optimizer::optimizeEssentialGraph(
    Map                                    *pMap,
    KeyFrame                               *pLoopKF,
    KeyFrame                               *pCurKF,
    const LoopClosing::KeyFrameAndPose     &NonCorrectedSim3,
    const LoopClosing::KeyFrameAndPose     &CorrectedSim3,
    const map<KeyFrame *, set<KeyFrame *>> &LoopConnections,
    const bool                             &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame *> vpKFs = pMap->getAllKeyFrames();
    const vector<MapPoint *> vpMPs = pMap->getAllMapPoints();

    const unsigned int nMaxKFid = pMap->getMaxKeyFrameId();

    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
        nMaxKFid + 1);
    vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

    vector<Eigen::Vector3d> vZvectors(nMaxKFid + 1); // For debugging
    Eigen::Vector3d         z_vec;
    z_vec << 0.0, 0.0, 1.0;

    const int minFeat = 100;

    // Set KeyFrame vertices
    for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
    {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it =
            CorrectedSim3.find(pKF);

        if (it != CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Sophus::SE3d Tcw = pKF->getPose().cast<double>();
            g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if (pKF->mnId == pMap->getInitKeyFrameId())
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);
        vZvectors[nIDi] = vScw[nIDi].rotation() * z_vec; // For debugging

        vpVertices[nIDi] = VSim3;
    }

    set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

    const Eigen::Matrix<double, 7, 7> matLambda =
        Eigen::Matrix<double, 7, 7>::Identity();

    // Set Loop edges
    int count_loop = 0;
    for (map<KeyFrame *, set<KeyFrame *>>::const_iterator
             mit  = LoopConnections.begin(),
             mend = LoopConnections.end();
         mit != mend;
         mit++)
    {
        KeyFrame               *pKF           = mit->first;
        const long unsigned int nIDi          = pKF->mnId;
        const set<KeyFrame *>  &spConnections = mit->second;
        const g2o::Sim3         Siw           = vScw[nIDi];
        const g2o::Sim3         Swi           = Siw.inverse();

        for (set<KeyFrame *>::const_iterator sit  = spConnections.begin(),
                                             send = spConnections.end();
             sit != send;
             sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
                pKF->getWeight(*sit) < minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3 *e = new g2o::EdgeSim3();
            e->setVertex(1,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(nIDj)));
            e->setVertex(0,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);
            count_loop++;
            sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
        }
    }

    // Set normal edges
    for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
    {
        KeyFrame *pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti =
            NonCorrectedSim3.find(pKF);

        if (iti != NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame *pParentKF = pKF->getParent();

        // Spanning tree edge
        if (pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj =
                NonCorrectedSim3.find(pParentKF);

            if (itj != NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3 *e = new g2o::EdgeSim3();
            e->setVertex(1,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(nIDj)));
            e->setVertex(0,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);
            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<KeyFrame *> sLoopEdges = pKF->getLoopEdges();
        for (set<KeyFrame *>::const_iterator sit  = sLoopEdges.begin(),
                                             send = sLoopEdges.end();
             sit != send;
             sit++)
        {
            KeyFrame *pLKF = *sit;
            if (pLKF->mnId < pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl =
                    NonCorrectedSim3.find(pLKF);

                if (itl != NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3      Sli = Slw * Swi;
                g2o::EdgeSim3 *el  = new g2o::EdgeSim3();
                el->setVertex(1,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  optimizer.vertex(pLKF->mnId)));
                el->setVertex(0,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame *> vpConnectedKFs =
            pKF->getCovisiblesByWeight(minFeat);
        for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin();
             vit != vpConnectedKFs.end();
             vit++)
        {
            KeyFrame *pKFn = *vit;
            if (pKFn && pKFn != pParentKF &&
                !pKF->hasChild(pKFn) /*&& !sLoopEdges.count(pKFn)*/)
            {
                if (!pKFn->isBad() && pKFn->mnId < pKF->mnId)
                {
                    if (sInsertedEdges.count(
                            make_pair(min(pKF->mnId, pKFn->mnId),
                                      max(pKF->mnId, pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn =
                        NonCorrectedSim3.find(pKFn);

                    if (itn != NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                    en->setVertex(1,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }

        // Inertial edges if inertial
        if (pKF->isImu && pKF->p_prevKF)
        {
            g2o::Sim3                                    Spw;
            LoopClosing::KeyFrameAndPose::const_iterator itp =
                NonCorrectedSim3.find(pKF->p_prevKF);
            if (itp != NonCorrectedSim3.end())
                Spw = itp->second;
            else
                Spw = vScw[pKF->p_prevKF->mnId];

            g2o::Sim3      Spi = Spw * Swi;
            g2o::EdgeSim3 *ep  = new g2o::EdgeSim3();
            ep->setVertex(1,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex(pKF->p_prevKF->mnId)));
            ep->setVertex(0,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex(nIDi)));
            ep->setMeasurement(Spi);
            ep->information() = matLambda;
            optimizer.addEdge(ep);
        }
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);
    optimizer.computeActiveErrors();
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap *VSim3 =
            static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw = VSim3->estimate();
        vCorrectedSwc[nIDi]    = CorrectedSiw.inverse();
        double s               = CorrectedSiw.scale();
        s                      = (s == 0.0) ? 1.0 : s;

        Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(),
                         CorrectedSiw.translation().cast<float>() / s);
        pKFi->setPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and
    // transform back with optimized pose
    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
    {
        MapPoint *pMP = vpMPs[i];

        if (pMP->isBad())
            continue;

        int nIDr;
        if (pMP->correctedByKeyFrameId == pCurKF->mnId)
        {
            nIDr = pMP->correctedReferenceKeyFrameId;
        }
        else
        {
            KeyFrame *pRefKF = pMP->getReferenceKeyFrame();
            nIDr             = pRefKF->mnId;
        }

        g2o::Sim3 Srw          = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Matrix<double, 3, 1> eigP3Dw = pMP->getWorldPos().cast<double>();
        Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
            correctedSwr.map(Srw.map(eigP3Dw));
        pMP->setWorldPos(eigCorrectedP3Dw.cast<float>());

        pMP->updateNormalAndDepth();
    }

    pMap->increaseChangeIndex();
}

} // namespace core
} // namespace vs_graphs
