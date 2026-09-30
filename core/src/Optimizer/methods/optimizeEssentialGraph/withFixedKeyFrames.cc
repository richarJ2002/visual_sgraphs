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
#include "System.h"
#include "Utils/Utils/objects/Utils.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

OptimizerStatus Optimizer::optimizeEssentialGraph(
    vs_graphs::core::KeyFrame                *p_currentKeyFrame_in,
    vs_graphs::core::Map                     *p_sourceMap_in,
    std::vector<vs_graphs::core::KeyFrame *> &fixedKeyFrames_in,
    std::vector<vs_graphs::core::KeyFrame *> &fixedCorrectedKeyFrames_in,
    std::vector<vs_graphs::core::KeyFrame *> &nonFixedKeyFrames_in,
    std::vector<vs_graphs::core::MapPoint *> &nonCorrectedMapPoints_in,
    const g2o::Sim3 &transform_mergeWorldToCurrentWorld_in)
{
    // Variables
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    // Linear solver and block solver
    g2o::BlockSolver_7_3::LinearSolverType *p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(p_linearSolver);
    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    p_solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(p_solver);

    // Get map
    Map *p_map = nullptr;
    if (p_currentKeyFrame_in->getMap(p_map) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /*
     * Keyframe identifiers are Atlas-global. A source map can therefore
     * contain a larger identifier than the surviving map, especially after a
     * previous merge or loading a serialized Atlas. Size every ID-indexed
     * table from all optimizer inputs rather than from only the current map.
     */
    unsigned long maximumKeyFrameId{};
    if (p_map->getMaxKeyFrameId(maximumKeyFrameId) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMaxKeyFrameId returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    const auto includeMaximumKeyFrameId =
        [&maximumKeyFrameId](const std::vector<KeyFrame *> &keyFrames_in)
    {
        for (const KeyFrame *p_keyFrame : keyFrames_in)
        {
            if (p_keyFrame != nullptr)
            {
                maximumKeyFrameId = std::max(maximumKeyFrameId, p_keyFrame->id);
            }
        }
    };

    includeMaximumKeyFrameId(fixedKeyFrames_in);
    includeMaximumKeyFrameId(fixedCorrectedKeyFrames_in);
    includeMaximumKeyFrameId(nonFixedKeyFrames_in);

    const std::size_t poseTableSize =
        static_cast<std::size_t>(maximumKeyFrameId) + 1U;

    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(
        poseTableSize);
    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
        poseTableSize);

    std::vector<bool> goodPoses(poseTableSize);
    std::vector<bool> badPoses(poseTableSize);

    const int minimumFeature = 100;

    // Loop over fixed KeyFrames
    for (KeyFrame *p_fixedKeyFrame : fixedKeyFrames_in)
    {
        bool fixedKeyFrameIsBad{};
        if (!(p_fixedKeyFrame == nullptr) &&
            p_fixedKeyFrame->isBad(fixedKeyFrameIsBad) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_fixedKeyFrame == nullptr || fixedKeyFrameIsBad)
            continue;

        g2o::VertexSim3Expmap *p_sim3Vertex = new g2o::VertexSim3Expmap();

        const int idCount = p_fixedKeyFrame->id;

        Sophus::SE3f fixedKeyFramePose{};
        if (p_fixedKeyFrame->getPose(fixedKeyFramePose) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d Tcw = fixedKeyFramePose.cast<double>();
        g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

        vCorrectedSwc[idCount] = Siw.inverse();
        p_sim3Vertex->setEstimate(Siw);

        p_sim3Vertex->setFixed(true);

        p_sim3Vertex->setId(idCount);
        p_sim3Vertex->setMarginalized(false);
        p_sim3Vertex->_fix_scale = true;

        optimizer.addVertex(p_sim3Vertex);

        goodPoses[idCount] = true;
        badPoses[idCount]  = false;
    }

    // Loop over fixed corrected KeyFrames
    std::set<unsigned long> idKeyFrames;
    for (KeyFrame *p_fixedKeyFrame : fixedCorrectedKeyFrames_in)
    {
        bool fixedKeyFrameIsBad2{};
        if (!(p_fixedKeyFrame == nullptr) &&
            p_fixedKeyFrame->isBad(fixedKeyFrameIsBad2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_fixedKeyFrame == nullptr || fixedKeyFrameIsBad2)
            continue;

        g2o::VertexSim3Expmap *p_sim3Vertex = new g2o::VertexSim3Expmap();

        const int idCount = p_fixedKeyFrame->id;

        Sophus::SE3f fixedKeyFramePose2{};
        if (p_fixedKeyFrame->getPose(fixedKeyFramePose2) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d Tcw = fixedKeyFramePose2.cast<double>();
        g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

        vCorrectedSwc[idCount] = Siw.inverse();
        p_sim3Vertex->setEstimate(Siw);

        Sophus::SE3d Tcw_bef = p_fixedKeyFrame->tcwBefMerge.cast<double>();
        vScw[idCount] =
            g2o::Sim3(Tcw_bef.unit_quaternion(), Tcw_bef.translation(), 1.0);

        p_sim3Vertex->setFixed(true);

        p_sim3Vertex->setId(idCount);
        p_sim3Vertex->setMarginalized(false);
        p_sim3Vertex->_fix_scale = true;

        optimizer.addVertex(p_sim3Vertex);

        idKeyFrames.insert(idCount);

        goodPoses[idCount] = true;
        badPoses[idCount]  = true;
    }

    // Loop over non-fixed KeyFrames
    for (KeyFrame *p_fixedKeyFrame : nonFixedKeyFrames_in)
    {
        bool fixedKeyFrameIsBad3{};
        if (!(p_fixedKeyFrame == nullptr) &&
            p_fixedKeyFrame->isBad(fixedKeyFrameIsBad3) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_fixedKeyFrame == nullptr || fixedKeyFrameIsBad3)
            continue;

        const int idCount = p_fixedKeyFrame->id;

        if (idKeyFrames.count(
                idCount)) // It has already added in the corrected merge KFs
            continue;

        g2o::VertexSim3Expmap *p_sim3Vertex = new g2o::VertexSim3Expmap();

        Sophus::SE3f fixedKeyFramePose3{};
        if (p_fixedKeyFrame->getPose(fixedKeyFramePose3) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d Tcw = fixedKeyFramePose3.cast<double>();
        g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

        vScw[idCount] = Siw;
        p_sim3Vertex->setEstimate(Siw);

        p_sim3Vertex->setFixed(false);

        p_sim3Vertex->setId(idCount);
        p_sim3Vertex->setMarginalized(false);
        p_sim3Vertex->_fix_scale = true;

        optimizer.addVertex(p_sim3Vertex);

        idKeyFrames.insert(idCount);

        goodPoses[idCount] = false;
        badPoses[idCount]  = true;
    }

    std::vector<KeyFrame *> mapKeyFrames;
    std::set<KeyFrame *>    keyFrames;
    mapKeyFrames.reserve(fixedKeyFrames_in.size() +
                         fixedCorrectedKeyFrames_in.size() +
                         nonFixedKeyFrames_in.size());

    const auto appendOptimizedKeyFrames =
        [&optimizer, &mapKeyFrames, &keyFrames](
            const std::vector<KeyFrame *> &keyFrames_in)
    {
        for (KeyFrame *p_keyFrame : keyFrames_in)
        {
            bool keyFrameIsBad{};
            if (!(p_keyFrame == nullptr) &&
                p_keyFrame->isBad(keyFrameIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame == nullptr || keyFrameIsBad ||
                optimizer.vertex(p_keyFrame->id) == nullptr ||
                !keyFrames.insert(p_keyFrame).second)
            {
                continue;
            }

            mapKeyFrames.push_back(p_keyFrame);
        }
    };

    appendOptimizedKeyFrames(fixedKeyFrames_in);
    appendOptimizedKeyFrames(fixedCorrectedKeyFrames_in);
    appendOptimizedKeyFrames(nonFixedKeyFrames_in);

    const Eigen::Matrix<double, 7, 7> matrixLambda =
        Eigen::Matrix<double, 7, 7>::Identity();

    for (KeyFrame *p_fixedKeyFrame : mapKeyFrames)
    {
        int       connectionCount = 0;
        const int idCount         = p_fixedKeyFrame->id;

        g2o::Sim3 correctedSwi;
        g2o::Sim3 Swi;

        if (goodPoses[idCount])
            correctedSwi = vCorrectedSwc[idCount];
        if (badPoses[idCount])
            Swi = vScw[idCount].inverse();

        KeyFrame *p_parentKeyFrame = nullptr;
        if (p_fixedKeyFrame->getParent(p_parentKeyFrame) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParent returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        // Spanning tree edge
        if (p_parentKeyFrame &&
            keyFrames.find(p_parentKeyFrame) != keyFrames.end())
        {
            int parentKeyFrameId = p_parentKeyFrame->id;

            g2o::Sim3 Sji;
            bool      hasRelation = false;

            if (goodPoses[idCount] && goodPoses[parentKeyFrameId])
            {
                Sji = vCorrectedSwc[parentKeyFrameId].inverse() * correctedSwi;
                hasRelation = true;
            }
            else if (badPoses[idCount] && badPoses[parentKeyFrameId])
            {
                Sji         = vScw[parentKeyFrameId] * Swi;
                hasRelation = true;
            }

            if (hasRelation)
            {
                g2o::EdgeSim3 *e = new g2o::EdgeSim3();
                e->setVertex(1,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(parentKeyFrameId)));
                e->setVertex(0,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(idCount)));
                e->setMeasurement(Sji);

                e->information() = matrixLambda;
                optimizer.addEdge(e);
                connectionCount++;
            }
        }

        // Loop edges
        std::set<KeyFrame *> loopEdges{};
        if (p_fixedKeyFrame->getLoopEdges(loopEdges) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLoopEdges returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (std::set<KeyFrame *>::const_iterator sit  = loopEdges.begin(),
                                                  send = loopEdges.end();
             sit != send;
             sit++)
        {
            KeyFrame *p_loopKeyFrame = *sit;
            if (keyFrames.find(p_loopKeyFrame) != keyFrames.end() &&
                p_loopKeyFrame->id < p_fixedKeyFrame->id)
            {
                g2o::Sim3 Sli;
                bool      hasRelation = false;

                if (goodPoses[idCount] && goodPoses[p_loopKeyFrame->id])
                {
                    Sli = vCorrectedSwc[p_loopKeyFrame->id].inverse() *
                          correctedSwi;
                    hasRelation = true;
                }
                else if (badPoses[idCount] && badPoses[p_loopKeyFrame->id])
                {
                    Sli         = vScw[p_loopKeyFrame->id] * Swi;
                    hasRelation = true;
                }

                if (hasRelation)
                {
                    g2o::EdgeSim3 *el = new g2o::EdgeSim3();
                    el->setVertex(1,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(p_loopKeyFrame->id)));
                    el->setVertex(0,
                                  dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                      optimizer.vertex(idCount)));
                    el->setMeasurement(Sli);
                    el->information() = matrixLambda;
                    optimizer.addEdge(el);
                    connectionCount++;
                }
            }
        }

        // Covisibility graph edges
        std::vector<KeyFrame *> connectedKeyFrames{};
        if (p_fixedKeyFrame->getCovisiblesByWeight(minimumFeature,
                                                   connectedKeyFrames) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCovisiblesByWeight returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (std::vector<KeyFrame *>::const_iterator vit =
                 connectedKeyFrames.begin();
             vit != connectedKeyFrames.end();
             vit++)
        {
            KeyFrame *pKFn = *vit;
            bool      fixedKeyFrameHasChild{};
            if ((pKFn && pKFn != p_parentKeyFrame) &&
                p_fixedKeyFrame->hasChild(pKFn, fixedKeyFrameHasChild) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasChild returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (pKFn && pKFn != p_parentKeyFrame && !fixedKeyFrameHasChild &&
                !loopEdges.count(pKFn) &&
                keyFrames.find(pKFn) != keyFrames.end())
            {
                bool pKFnIsBad{};
                if (pKFn->isBad(pKFnIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!pKFnIsBad && pKFn->id < p_fixedKeyFrame->id)
                {

                    g2o::Sim3 Sni;
                    bool      hasRelation = false;

                    if (goodPoses[idCount] && goodPoses[pKFn->id])
                    {
                        Sni = vCorrectedSwc[pKFn->id].inverse() * correctedSwi;
                        hasRelation = true;
                    }
                    else if (badPoses[idCount] && badPoses[pKFn->id])
                    {
                        Sni         = vScw[pKFn->id] * Swi;
                        hasRelation = true;
                    }

                    if (hasRelation)
                    {
                        g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                        en->setVertex(
                            1,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(pKFn->id)));
                        en->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(idCount)));
                        en->setMeasurement(Sni);
                        en->information() = matrixLambda;
                        optimizer.addEdge(en);
                        connectionCount++;
                    }
                }
            }
        }

        if (connectionCount == 0)
        {
            if (Verbose::printMess("Opt_Essential: KF " +
                                       std::to_string(p_fixedKeyFrame->id) +
                                       " has 0 connections",
                                   Verbose::VERBOSITY_DEBUG) !=
                VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    std::unique_lock<std::mutex> lock(p_map->mapUpdateMutex);

    // Inform the user
    std::cout << "\n[Optimizer]" << std::endl;

    // Loop over non-fixed KeyFrames to correct them
    // [hint] Sim3 [sR t | 0 1] --> SE3 [R t/s| 0 1]
    std::cout << "- Correcting the poses of KeyFrames ..." << std::endl;
    for (KeyFrame *p_fixedKeyFrame : nonFixedKeyFrames_in)
    {
        bool fixedKeyFrameIsBad4{};
        if (!(p_fixedKeyFrame == nullptr) &&
            p_fixedKeyFrame->isBad(fixedKeyFrameIsBad4) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_fixedKeyFrame == nullptr || fixedKeyFrameIsBad4)
            continue;

        const int idCount = p_fixedKeyFrame->id;

        g2o::VertexSim3Expmap *p_sim3Vertex =
            static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(idCount));

        if (p_sim3Vertex == nullptr)
        {
            continue;
        }

        g2o::Sim3 CorrectedSiw = p_sim3Vertex->estimate();
        vCorrectedSwc[idCount] = CorrectedSiw.inverse();
        double       s         = CorrectedSiw.scale();
        Sophus::SE3d Tiw(CorrectedSiw.rotation(),
                         CorrectedSiw.translation() / s);

        Sophus::SE3f fixedKeyFramePose4{};
        if (p_fixedKeyFrame->getPose(fixedKeyFramePose4) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_fixedKeyFrame->tcwBefMerge = fixedKeyFramePose4;
        Sophus::SE3f fixedKeyFramePoseInverse{};
        if (p_fixedKeyFrame->getPoseInverse(fixedKeyFramePoseInverse) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        p_fixedKeyFrame->twcBefMerge = fixedKeyFramePoseInverse;
        if (p_fixedKeyFrame->setPose(Tiw.cast<float>()) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    // Transform to "non-optimized" reference keyframe pose and transform back
    // with optimized pose
    std::cout << "- Correcting the poses of 3D mapped points ..." << std::endl;
    for (MapPoint *p_mapPoint : nonCorrectedMapPoints_in)
    {
        bool mapPointIsBad{};
        if (!(p_mapPoint == nullptr) &&
            p_mapPoint->isBad(mapPointIsBad) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mapPoint == nullptr || mapPointIsBad)
        {
            continue;
        }

        bool mapPointWasTransformed = false;

        const auto canCorrectFromReference =
            [&badPoses](KeyFrame *p_keyFrame_in)
        {
            bool keyFrameIsBad{};
            if ((p_keyFrame_in != nullptr) &&
                p_keyFrame_in->isBad(keyFrameIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            return p_keyFrame_in != nullptr && !keyFrameIsBad &&
                   p_keyFrame_in->id < badPoses.size() &&
                   badPoses[p_keyFrame_in->id];
        };

        KeyFrame *p_referenceKeyFrame = nullptr;
        if (p_mapPoint->getReferenceKeyFrame(p_referenceKeyFrame) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getReferenceKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (!canCorrectFromReference(p_referenceKeyFrame))
        {
            p_referenceKeyFrame = nullptr;

            std::map<KeyFrame *, std::tuple<int, int>> observations{};
            if (p_mapPoint->getObservations(observations) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getObservations returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (const auto &[p_observingKeyFrame, featureIndexes] :
                 observations)
            {
                (void)featureIndexes;

                if (canCorrectFromReference(p_observingKeyFrame))
                {
                    p_referenceKeyFrame = p_observingKeyFrame;
                    break;
                }
            }
        }

        if (p_referenceKeyFrame == nullptr)
        {
            if (Verbose::printMess("MP " + std::to_string(p_mapPoint->id) +
                                       " without a valid reference KF",
                                   Verbose::VERBOSITY_DEBUG) !=
                VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {
            Sophus::SE3f TNonCorrectedwr = p_referenceKeyFrame->twcBefMerge;
            Sophus::SE3f Twr{};
            if (p_referenceKeyFrame->getPoseInverse(Twr) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            Eigen::Vector3f mapPointWorldPos{};
            if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3f eigCorrectedP3Dw =
                Twr * TNonCorrectedwr.inverse() * mapPointWorldPos;
            if (p_mapPoint->setWorldPos(eigCorrectedP3Dw) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->updateNormalAndDepth() !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateNormalAndDepth returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            mapPointWasTransformed = true;
        }

        /*
         * Orphan points and points tied only to fixed current-side keyframes
         * still belong to the merge-world frame. Apply the baseline map
         * transform before their ownership changes.
         */
        if (!mapPointWasTransformed)
        {
            Eigen::Vector3f position_mergeWorld_m{};
            if (p_mapPoint->getWorldPos(position_mergeWorld_m) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3f normal_mergeWorld{};
            if (p_mapPoint->getNormal(normal_mergeWorld) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getNormal returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->setWorldPos(
                    transform_mergeWorldToCurrentWorld_in
                        .map(position_mergeWorld_m.cast<double>())
                        .cast<float>()) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->setNormalVector(
                    transform_mergeWorldToCurrentWorld_in.rotation()
                        .cast<float>() *
                    normal_mergeWorld) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNormalVector returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->updateNormalAndDepth() !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: updateNormalAndDepth returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
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
            bool keyFrameIsBad{};
            if (!(p_keyFrame == nullptr) &&
                p_keyFrame->isBad(keyFrameIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame == nullptr || keyFrameIsBad)
            {
                continue;
            }

            const Sophus::SE3d poseBefore_WorldToCamera =
                p_keyFrame->tcwBefMerge.cast<double>();
            Sophus::SE3f keyFramePose{};
            if (p_keyFrame->getPose(keyFramePose) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            const Sophus::SE3d poseAfter_WorldToCamera =
                keyFramePose.cast<double>();

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

    appendSemanticDeformationNodes(fixedCorrectedKeyFrames_in);
    appendSemanticDeformationNodes(nonFixedKeyFrames_in);

    if (p_sourceMap_in != nullptr)
    {
        if (utils::utils::Utils::propagateSemanticPoseCorrections(
                p_sourceMap_in,
                keyFramePosesBefore_WorldToCamera,
                keyFramePosesAfter_WorldToCamera,
                transform_mergeWorldToCurrentWorld_in) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: propagateSemanticPoseCorrections returned a failure "
                "status although it cannot fail; continuing as before.",
                __func__);
        }
    }

    std::cout << "- Corrections finished!" << std::endl;

    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
