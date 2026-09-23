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
#include "Utils/Utils/objects/Utils.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void Optimizer::optimizeEssentialGraph(
    vs_graphs::core::KeyFrame                *pCurKF,
    vs_graphs::core::Map                     *p_sourceMap_inout,
    std::vector<vs_graphs::core::KeyFrame *> &vpFixedKFs,
    std::vector<vs_graphs::core::KeyFrame *> &vpFixedCorrectedKFs,
    std::vector<vs_graphs::core::KeyFrame *> &vpNonFixedKFs,
    std::vector<vs_graphs::core::MapPoint *> &vpNonCorrectedMPs,
    const g2o::Sim3 &transform_mergeWorldToCurrentWorld_in)
{
    // Variables
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    // Linear solver and block solver
    g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    // Get map
    Map *pMap = pCurKF->getMap();

    /*
     * Keyframe identifiers are Atlas-global. A source map can therefore
     * contain a larger identifier than the surviving map, especially after a
     * previous merge or loading a serialized Atlas. Size every ID-indexed
     * table from all optimizer inputs rather than from only the current map.
     */
    unsigned long maxKeyFrameId = pMap->getMaxKeyFrameId();

    const auto includeMaximumKeyFrameId =
        [&maxKeyFrameId](const std::vector<KeyFrame *> &keyFrames_in)
    {
        for (const KeyFrame *p_keyFrame : keyFrames_in)
        {
            if (p_keyFrame != nullptr)
            {
                maxKeyFrameId = std::max(maxKeyFrameId, p_keyFrame->mnId);
            }
        }
    };

    includeMaximumKeyFrameId(vpFixedKFs);
    includeMaximumKeyFrameId(vpFixedCorrectedKFs);
    includeMaximumKeyFrameId(vpNonFixedKFs);

    const std::size_t poseTableSize =
        static_cast<std::size_t>(maxKeyFrameId) + 1U;

    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(poseTableSize);
    vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
        poseTableSize);

    vector<bool> vpGoodPose(poseTableSize);
    vector<bool> vpBadPose(poseTableSize);

    const int minFeat = 100;

    // Loop over fixed KeyFrames
    for (KeyFrame *pKFi : vpFixedKFs)
    {
        if (pKFi == nullptr || pKFi->isBad())
            continue;

        g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKFi->mnId;

        Sophus::SE3d Tcw = pKFi->getPose().cast<double>();
        g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

        vCorrectedSwc[nIDi] = Siw.inverse();
        VSim3->setEstimate(Siw);

        VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = true;

        optimizer.addVertex(VSim3);

        vpGoodPose[nIDi] = true;
        vpBadPose[nIDi]  = false;
    }

    // Loop over fixed corrected KeyFrames
    set<unsigned long> sIdKF;
    for (KeyFrame *pKFi : vpFixedCorrectedKFs)
    {
        if (pKFi == nullptr || pKFi->isBad())
            continue;

        g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKFi->mnId;

        Sophus::SE3d Tcw = pKFi->getPose().cast<double>();
        g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

        vCorrectedSwc[nIDi] = Siw.inverse();
        VSim3->setEstimate(Siw);

        Sophus::SE3d Tcw_bef = pKFi->tcwBefMerge.cast<double>();
        vScw[nIDi] =
            g2o::Sim3(Tcw_bef.unit_quaternion(), Tcw_bef.translation(), 1.0);

        VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = true;

        optimizer.addVertex(VSim3);

        sIdKF.insert(nIDi);

        vpGoodPose[nIDi] = true;
        vpBadPose[nIDi]  = true;
    }

    // Loop over non-fixed KeyFrames
    for (KeyFrame *pKFi : vpNonFixedKFs)
    {
        if (pKFi == nullptr || pKFi->isBad())
            continue;

        const int nIDi = pKFi->mnId;

        if (sIdKF.count(
                nIDi)) // It has already added in the corrected merge KFs
            continue;

        g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

        Sophus::SE3d Tcw = pKFi->getPose().cast<double>();
        g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

        vScw[nIDi] = Siw;
        VSim3->setEstimate(Siw);

        VSim3->setFixed(false);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = true;

        optimizer.addVertex(VSim3);

        sIdKF.insert(nIDi);

        vpGoodPose[nIDi] = false;
        vpBadPose[nIDi]  = true;
    }

    vector<KeyFrame *> vpKFs;
    set<KeyFrame *>    spKFs;
    vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() +
                  vpNonFixedKFs.size());

    const auto appendOptimizedKeyFrames =
        [&optimizer, &vpKFs, &spKFs](const vector<KeyFrame *> &keyFrames_in)
    {
        for (KeyFrame *p_keyFrame : keyFrames_in)
        {
            if (p_keyFrame == nullptr || p_keyFrame->isBad() ||
                optimizer.vertex(p_keyFrame->mnId) == nullptr ||
                !spKFs.insert(p_keyFrame).second)
            {
                continue;
            }

            vpKFs.push_back(p_keyFrame);
        }
    };

    appendOptimizedKeyFrames(vpFixedKFs);
    appendOptimizedKeyFrames(vpFixedCorrectedKFs);
    appendOptimizedKeyFrames(vpNonFixedKFs);

    const Eigen::Matrix<double, 7, 7> matLambda =
        Eigen::Matrix<double, 7, 7>::Identity();

    for (KeyFrame *pKFi : vpKFs)
    {
        int       num_connections = 0;
        const int nIDi            = pKFi->mnId;

        g2o::Sim3 correctedSwi;
        g2o::Sim3 Swi;

        if (vpGoodPose[nIDi])
            correctedSwi = vCorrectedSwc[nIDi];
        if (vpBadPose[nIDi])
            Swi = vScw[nIDi].inverse();

        KeyFrame *pParentKFi = pKFi->getParent();

        // Spanning tree edge
        if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end())
        {
            int nIDj = pParentKFi->mnId;

            g2o::Sim3 Sji;
            bool      bHasRelation = false;

            if (vpGoodPose[nIDi] && vpGoodPose[nIDj])
            {
                Sji          = vCorrectedSwc[nIDj].inverse() * correctedSwi;
                bHasRelation = true;
            }
            else if (vpBadPose[nIDi] && vpBadPose[nIDj])
            {
                Sji          = vScw[nIDj] * Swi;
                bHasRelation = true;
            }

            if (bHasRelation)
            {
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
                num_connections++;
            }
        }

        // Loop edges
        const set<KeyFrame *> sLoopEdges = pKFi->getLoopEdges();
        for (set<KeyFrame *>::const_iterator sit  = sLoopEdges.begin(),
                                             send = sLoopEdges.end();
             sit != send;
             sit++)
        {
            KeyFrame *pLKF = *sit;
            if (spKFs.find(pLKF) != spKFs.end() && pLKF->mnId < pKFi->mnId)
            {
                g2o::Sim3 Sli;
                bool      bHasRelation = false;

                if (vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId])
                {
                    Sli = vCorrectedSwc[pLKF->mnId].inverse() * correctedSwi;
                    bHasRelation = true;
                }
                else if (vpBadPose[nIDi] && vpBadPose[pLKF->mnId])
                {
                    Sli          = vScw[pLKF->mnId] * Swi;
                    bHasRelation = true;
                }

                if (bHasRelation)
                {
                    g2o::EdgeSim3 *el = new g2o::EdgeSim3();
                    el->setVertex(1,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(pLKF->mnId)));
                    el->setVertex(0,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(nIDi)));
                    el->setMeasurement(Sli);
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                    num_connections++;
                }
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame *> vpConnectedKFs =
            pKFi->getCovisiblesByWeight(minFeat);
        for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin();
             vit != vpConnectedKFs.end();
             vit++)
        {
            KeyFrame *pKFn = *vit;
            if (pKFn && pKFn != pParentKFi && !pKFi->hasChild(pKFn) &&
                !sLoopEdges.count(pKFn) && spKFs.find(pKFn) != spKFs.end())
            {
                if (!pKFn->isBad() && pKFn->mnId < pKFi->mnId)
                {

                    g2o::Sim3 Sni;
                    bool      bHasRelation = false;

                    if (vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId])
                    {
                        Sni =
                            vCorrectedSwc[pKFn->mnId].inverse() * correctedSwi;
                        bHasRelation = true;
                    }
                    else if (vpBadPose[nIDi] && vpBadPose[pKFn->mnId])
                    {
                        Sni          = vScw[pKFn->mnId] * Swi;
                        bHasRelation = true;
                    }

                    if (bHasRelation)
                    {
                        g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                        en->setVertex(
                            1,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(pKFn->mnId)));
                        en->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(nIDi)));
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                        num_connections++;
                    }
                }
            }
        }

        if (num_connections == 0)
            Verbose::printMess("Opt_Essential: KF " + to_string(pKFi->mnId) +
                                   " has 0 connections",
                               Verbose::VERBOSITY_DEBUG);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // Inform the user
    std::cout << "\n[Optimizer]" << std::endl;

    // Loop over non-fixed KeyFrames to correct them
    // [hint] Sim3 [sR t | 0 1] --> SE3 [R t/s| 0 1]
    std::cout << "- Correcting the poses of KeyFrames ..." << std::endl;
    for (KeyFrame *pKFi : vpNonFixedKFs)
    {
        if (pKFi == nullptr || pKFi->isBad())
            continue;

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap *VSim3 =
            static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));

        if (VSim3 == nullptr)
        {
            continue;
        }

        g2o::Sim3 CorrectedSiw = VSim3->estimate();
        vCorrectedSwc[nIDi]    = CorrectedSiw.inverse();
        double       s         = CorrectedSiw.scale();
        Sophus::SE3d Tiw(CorrectedSiw.rotation(),
                         CorrectedSiw.translation() / s);

        pKFi->tcwBefMerge = pKFi->getPose();
        pKFi->twcBefMerge = pKFi->getPoseInverse();
        pKFi->setPose(Tiw.cast<float>());
    }

    // Transform to "non-optimized" reference keyframe pose and transform back
    // with optimized pose
    std::cout << "- Correcting the poses of 3D mapped points ..." << std::endl;
    for (MapPoint *pMPi : vpNonCorrectedMPs)
    {
        if (pMPi == nullptr || pMPi->isBad())
        {
            continue;
        }

        bool mapPointWasTransformed = false;

        const auto canCorrectFromReference =
            [&vpBadPose](KeyFrame *p_keyFrame_in)
        {
            return p_keyFrame_in != nullptr && !p_keyFrame_in->isBad() &&
                   p_keyFrame_in->mnId < vpBadPose.size() &&
                   vpBadPose[p_keyFrame_in->mnId];
        };

        KeyFrame *pRefKF = pMPi->getReferenceKeyFrame();

        if (!canCorrectFromReference(pRefKF))
        {
            pRefKF = nullptr;

            const auto observations = pMPi->getObservations();
            for (const auto &[p_observingKeyFrame, featureIndexes] :
                 observations)
            {
                (void)featureIndexes;

                if (canCorrectFromReference(p_observingKeyFrame))
                {
                    pRefKF = p_observingKeyFrame;
                    break;
                }
            }
        }

        if (pRefKF == nullptr)
        {
            Verbose::printMess("MP " + to_string(pMPi->mnId) +
                                   " without a valid reference KF",
                               Verbose::VERBOSITY_DEBUG);
        }
        else
        {
            Sophus::SE3f TNonCorrectedwr = pRefKF->twcBefMerge;
            Sophus::SE3f Twr             = pRefKF->getPoseInverse();

            Eigen::Vector3f eigCorrectedP3Dw =
                Twr * TNonCorrectedwr.inverse() * pMPi->getWorldPos();
            pMPi->setWorldPos(eigCorrectedP3Dw);

            pMPi->updateNormalAndDepth();
            mapPointWasTransformed = true;
        }

        /*
         * Orphan points and points tied only to fixed current-side keyframes
         * still belong to the merge-world frame. Apply the baseline map
         * transform before their ownership changes.
         */
        if (!mapPointWasTransformed)
        {
            const Eigen::Vector3f position_mergeWorld_m = pMPi->getWorldPos();
            const Eigen::Vector3f normal_mergeWorld     = pMPi->getNormal();

            pMPi->setWorldPos(transform_mergeWorldToCurrentWorld_in
                                  .map(position_mergeWorld_m.cast<double>())
                                  .cast<float>());

            pMPi->setNormalVector(
                transform_mergeWorldToCurrentWorld_in.rotation().cast<float>() *
                normal_mergeWorld);

            pMPi->updateNormalAndDepth();
        }
    }

    /*
     * Apply one shared deformation to the complete source semantic graph.
     * Keeping this in the same primitive used by welding BA and GBA prevents
     * planes from receiving a local correction while their rooms, passages,
     * floors, and topology receive only the coarse map transform.
     */
    utils::utils::Utils::KeyFramePoseMap keyFramePosesBefore_WorldToCamera;
    utils::utils::Utils::KeyFramePoseMap keyFramePosesAfter_WorldToCamera;

    const auto appendSemanticDeformationNodes =
        [&keyFramePosesBefore_WorldToCamera, &keyFramePosesAfter_WorldToCamera](
            const std::vector<KeyFrame *> &keyFrames_in)
    {
        for (KeyFrame *p_keyFrame : keyFrames_in)
        {
            if (p_keyFrame == nullptr || p_keyFrame->isBad())
            {
                continue;
            }

            const Sophus::SE3d poseBefore_WorldToCamera =
                p_keyFrame->tcwBefMerge.cast<double>();
            const Sophus::SE3d poseAfter_WorldToCamera =
                p_keyFrame->getPose().cast<double>();

            keyFramePosesBefore_WorldToCamera.insert_or_assign(
                p_keyFrame,
                g2o::Sim3(poseBefore_WorldToCamera.unit_quaternion(),
                          poseBefore_WorldToCamera.translation(),
                          1.0));
            keyFramePosesAfter_WorldToCamera.insert_or_assign(
                p_keyFrame,
                g2o::Sim3(poseAfter_WorldToCamera.unit_quaternion(),
                          poseAfter_WorldToCamera.translation(),
                          1.0));
        }
    };

    appendSemanticDeformationNodes(vpFixedCorrectedKFs);
    appendSemanticDeformationNodes(vpNonFixedKFs);

    if (p_sourceMap_inout != nullptr)
    {
        utils::utils::Utils::propagateSemanticPoseCorrections(
            p_sourceMap_inout,
            keyFramePosesBefore_WorldToCamera,
            keyFramePosesAfter_WorldToCamera,
            transform_mergeWorldToCurrentWorld_in);
    }

    std::cout << "- Corrections finished!" << std::endl;
}

} // namespace core
} // namespace vs_graphs
