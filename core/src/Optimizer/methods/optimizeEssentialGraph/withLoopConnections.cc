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
    Map                                    *p_map_inout,
    KeyFrame                               *p_loopKeyFrame_in,
    KeyFrame                               *p_currentKeyFrame_in,
    const LoopClosing::KeyFrameAndPose     &NonCorrectedSim3_in,
    const LoopClosing::KeyFrameAndPose     &CorrectedSim3_in,
    const map<KeyFrame *, set<KeyFrame *>> &loopConnections_in,
    const bool                             &isScaleFixed_in)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType *p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(p_linearSolver);
    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    p_solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(p_solver);

    const vector<KeyFrame *> keyFrames = p_map_inout->getAllKeyFrames();
    const vector<MapPoint *> mapPoints = p_map_inout->getAllMapPoints();

    const unsigned int maximumKeyFrameIdCount = p_map_inout->getMaxKeyFrameId();

    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(
        maximumKeyFrameIdCount + 1);
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
        maximumKeyFrameIdCount + 1);
    vector<g2o::VertexSim3Expmap *> vertices(maximumKeyFrameIdCount + 1);

    vector<Eigen::Vector3d> zvectors(maximumKeyFrameIdCount +
                                     1); // For debugging
    Eigen::Vector3d         z_vec;
    z_vec << 0.0, 0.0, 1.0;

    const int minimumFeature = 100;

    // Set KeyFrame vertices
    for (size_t keyFrameIndex = 0, iend = keyFrames.size();
         keyFrameIndex < iend;
         keyFrameIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];
        if (p_keyFrame->isBad())
            continue;
        g2o::VertexSim3Expmap *p_sim3Vertex = new g2o::VertexSim3Expmap();

        const int idCount = p_keyFrame->id;

        LoopClosing::KeyFrameAndPose::const_iterator correctedPoseIt =
            CorrectedSim3_in.find(p_keyFrame);

        if (correctedPoseIt != CorrectedSim3_in.end())
        {
            vScw[idCount] = correctedPoseIt->second;
            p_sim3Vertex->setEstimate(correctedPoseIt->second);
        }
        else
        {
            Sophus::SE3d Tcw = p_keyFrame->getPose().cast<double>();
            g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);
            vScw[idCount] = Siw;
            p_sim3Vertex->setEstimate(Siw);
        }

        if (p_keyFrame->id == p_map_inout->getInitKeyFrameId())
            p_sim3Vertex->setFixed(true);

        p_sim3Vertex->setId(idCount);
        p_sim3Vertex->setMarginalized(false);
        p_sim3Vertex->_fix_scale = isScaleFixed_in;

        optimizer.addVertex(p_sim3Vertex);
        zvectors[idCount] = vScw[idCount].rotation() * z_vec; // For debugging

        vertices[idCount] = p_sim3Vertex;
    }

    set<pair<long unsigned int, long unsigned int>> insertedEdges;

    const Eigen::Matrix<double, 7, 7> matrixLambda =
        Eigen::Matrix<double, 7, 7>::Identity();

    // Set Loop edges
    int loopCount = 0;
    for (map<KeyFrame *, set<KeyFrame *>>::const_iterator
             mit  = loopConnections_in.begin(),
             mend = loopConnections_in.end();
         mit != mend;
         mit++)
    {
        KeyFrame               *p_keyFrame  = mit->first;
        const long unsigned int idCount     = p_keyFrame->id;
        const set<KeyFrame *>  &connections = mit->second;
        const g2o::Sim3         Siw         = vScw[idCount];
        const g2o::Sim3         Swi         = Siw.inverse();

        for (set<KeyFrame *>::const_iterator sit  = connections.begin(),
                                             send = connections.end();
             sit != send;
             sit++)
        {
            const long unsigned int loopKeyFrameId = (*sit)->id;
            if ((idCount != p_currentKeyFrame_in->id ||
                 loopKeyFrameId != p_loopKeyFrame_in->id) &&
                p_keyFrame->getWeight(*sit) < minimumFeature)
                continue;

            const g2o::Sim3 Sjw = vScw[loopKeyFrameId];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3 *e = new g2o::EdgeSim3();
            e->setVertex(1,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(loopKeyFrameId)));
            e->setVertex(0,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(idCount)));
            e->setMeasurement(Sji);

            e->information() = matrixLambda;

            optimizer.addEdge(e);
            loopCount++;
            insertedEdges.insert(make_pair(min(idCount, loopKeyFrameId),
                                           max(idCount, loopKeyFrameId)));
        }
    }

    // Set normal edges
    for (size_t keyFrameIndex = 0, iend = keyFrames.size();
         keyFrameIndex < iend;
         keyFrameIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];

        const int idCount = p_keyFrame->id;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti =
            NonCorrectedSim3_in.find(p_keyFrame);

        if (iti != NonCorrectedSim3_in.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[idCount].inverse();

        KeyFrame *p_parentKeyFrame = p_keyFrame->getParent();

        // Spanning tree edge
        if (p_parentKeyFrame)
        {
            int loopKeyFrameId = p_parentKeyFrame->id;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj =
                NonCorrectedSim3_in.find(p_parentKeyFrame);

            if (itj != NonCorrectedSim3_in.end())
                Sjw = itj->second;
            else
                Sjw = vScw[loopKeyFrameId];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3 *e = new g2o::EdgeSim3();
            e->setVertex(1,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(loopKeyFrameId)));
            e->setVertex(0,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(idCount)));
            e->setMeasurement(Sji);
            e->information() = matrixLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<KeyFrame *> loopEdges = p_keyFrame->getLoopEdges();
        for (set<KeyFrame *>::const_iterator sit  = loopEdges.begin(),
                                             send = loopEdges.end();
             sit != send;
             sit++)
        {
            KeyFrame *p_loopKeyFrame = *sit;
            if (p_loopKeyFrame->id < p_keyFrame->id)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl =
                    NonCorrectedSim3_in.find(p_loopKeyFrame);

                if (itl != NonCorrectedSim3_in.end())
                    Slw = itl->second;
                else
                    Slw = vScw[p_loopKeyFrame->id];

                g2o::Sim3      Sli = Slw * Swi;
                g2o::EdgeSim3 *el  = new g2o::EdgeSim3();
                el->setVertex(1,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  optimizer.vertex(p_loopKeyFrame->id)));
                el->setVertex(0,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  optimizer.vertex(idCount)));
                el->setMeasurement(Sli);
                el->information() = matrixLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame *> connectedKeyFrames =
            p_keyFrame->getCovisiblesByWeight(minimumFeature);
        for (vector<KeyFrame *>::const_iterator vit =
                 connectedKeyFrames.begin();
             vit != connectedKeyFrames.end();
             vit++)
        {
            KeyFrame *pKFn = *vit;
            if (pKFn && pKFn != p_parentKeyFrame &&
                !p_keyFrame->hasChild(pKFn) /*&& !sLoopEdges.count(pKFn)*/)
            {
                if (!pKFn->isBad() && pKFn->id < p_keyFrame->id)
                {
                    if (insertedEdges.count(
                            make_pair(min(p_keyFrame->id, pKFn->id),
                                      max(p_keyFrame->id, pKFn->id))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn =
                        NonCorrectedSim3_in.find(pKFn);

                    if (itn != NonCorrectedSim3_in.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->id];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                    en->setVertex(1,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(pKFn->id)));
                    en->setVertex(0,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(idCount)));
                    en->setMeasurement(Sni);
                    en->information() = matrixLambda;
                    optimizer.addEdge(en);
                }
            }
        }

        // Inertial edges if inertial
        if (p_keyFrame->isImu && p_keyFrame->p_prevKF)
        {
            g2o::Sim3                                    Spw;
            LoopClosing::KeyFrameAndPose::const_iterator itp =
                NonCorrectedSim3_in.find(p_keyFrame->p_prevKF);
            if (itp != NonCorrectedSim3_in.end())
                Spw = itp->second;
            else
                Spw = vScw[p_keyFrame->p_prevKF->id];

            g2o::Sim3      Spi = Spw * Swi;
            g2o::EdgeSim3 *ep  = new g2o::EdgeSim3();
            ep->setVertex(1,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex(p_keyFrame->p_prevKF->id)));
            ep->setVertex(0,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              optimizer.vertex(idCount)));
            ep->setMeasurement(Spi);
            ep->information() = matrixLambda;
            optimizer.addEdge(ep);
        }
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);
    optimizer.computeActiveErrors();
    unique_lock<mutex> lock(p_map_inout->mapUpdateMutex);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
         keyFrameIndex++)
    {
        KeyFrame *p_mapKeyFrame = keyFrames[keyFrameIndex];

        const int idCount = p_mapKeyFrame->id;

        g2o::VertexSim3Expmap *p_sim3Vertex =
            static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(idCount));
        g2o::Sim3 CorrectedSiw = p_sim3Vertex->estimate();
        vCorrectedSwc[idCount] = CorrectedSiw.inverse();
        double s               = CorrectedSiw.scale();
        s                      = (s == 0.0) ? 1.0 : s;

        Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(),
                         CorrectedSiw.translation().cast<float>() / s);
        p_mapKeyFrame->setPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and
    // transform back with optimized pose
    for (size_t keyFrameIndex = 0, iend = mapPoints.size();
         keyFrameIndex < iend;
         keyFrameIndex++)
    {
        MapPoint *p_mapPoint = mapPoints[keyFrameIndex];

        if (p_mapPoint->isBad())
            continue;

        int nIDr;
        if (p_mapPoint->correctedByKeyFrameId == p_currentKeyFrame_in->id)
        {
            nIDr = p_mapPoint->correctedReferenceKeyFrameId;
        }
        else
        {
            KeyFrame *p_referenceKeyFrame = p_mapPoint->getReferenceKeyFrame();
            nIDr                          = p_referenceKeyFrame->id;
        }

        g2o::Sim3 Srw          = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Matrix<double, 3, 1> eigP3Dw =
            p_mapPoint->getWorldPos().cast<double>();
        Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
            correctedSwr.map(Srw.map(eigP3Dw));
        p_mapPoint->setWorldPos(eigCorrectedP3Dw.cast<float>());

        p_mapPoint->updateNormalAndDepth();
    }

    p_map_inout->increaseChangeIndex();
}

} // namespace core
} // namespace vs_graphs
