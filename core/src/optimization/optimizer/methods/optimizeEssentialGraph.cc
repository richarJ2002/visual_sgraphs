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

/*!
 * @file            optimizeEssentialGraph.cc
 *
 * @brief           Implements all three Optimizer::inertialOptimization()
 *                  overloads.
 */


#include "mapping/geometry_helpers/utils.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"
#include "execution/system/objects/VerboseStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "optimization/optimizer/objects/OptimizerClass.h"
#include "execution/system/system.h"
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace optimizer
{

OptimizerStatus Optimizer::optimizeEssentialGraph(
    vs_graphs::core::keyframe::KeyFrame                *p_currentKeyFrame_in,
    vs_graphs::core::map::Map                          *p_sourceMap_in,
    std::vector<vs_graphs::core::keyframe::KeyFrame *> &fixedKeyFrames_in,
    std::vector<vs_graphs::core::keyframe::KeyFrame *>
        &fixedCorrectedKeyFrames_in,
    std::vector<vs_graphs::core::keyframe::KeyFrame *> &nonFixedKeyFrames_in,
    std::vector<vs_graphs::core::map_point::MapPoint *>
                    &nonCorrectedMapPoints_in,
    const g2o::Sim3 &mergeTransform_mergeWorldToCurrentWorld_in)
{
    /* Variables */
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    /* Linear solver and block solver */
    g2o::BlockSolver_7_3::LinearSolverType *p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(p_linearSolver);
    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    p_solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(p_solver);

    /* Get map */
    map::Map *p_map = nullptr;
    if (p_currentKeyFrame_in->getMap(p_map) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /*
     * Keyframe identifiers are atlas::Atlas-global. A source map can therefore
     * contain a larger identifier than the surviving map, especially after a
     * previous merge or loading a serialized atlas::Atlas. Size every
     * ID-indexed table from all optimizer inputs rather than from only the
     * current map.
     */
    unsigned long maximumKeyFrameId{};
    if (p_map->getMaxKeyFrameId(maximumKeyFrameId) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMaxKeyFrameId returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    const auto includeMaximumKeyFrameId =
        [&maximumKeyFrameId](
            const std::vector<keyframe::KeyFrame *> &keyFrames_in)
    {
        for (const keyframe::KeyFrame *p_keyFrame : keyFrames_in)
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

    /* Loop over fixed KeyFrames */
    for (keyframe::KeyFrame *p_fixedKeyFrame : fixedKeyFrames_in)
    {
        bool fixedKeyFrameIsBad{};
        if (!(p_fixedKeyFrame == nullptr) &&
            p_fixedKeyFrame->isBad(fixedKeyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d cameraPose_worldToCamera =
            fixedKeyFramePose.cast<double>();
        g2o::Sim3 Siw(cameraPose_worldToCamera.unit_quaternion(),
                      cameraPose_worldToCamera.translation(),
                      1.0);

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

    /* Loop over fixed corrected KeyFrames */
    std::set<unsigned long> idKeyFrames;
    for (keyframe::KeyFrame *p_fixedKeyFrame : fixedCorrectedKeyFrames_in)
    {
        bool fixedKeyFrameIsBad2{};
        if (!(p_fixedKeyFrame == nullptr) &&
            p_fixedKeyFrame->isBad(fixedKeyFrameIsBad2) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d cameraPose_worldToCamera =
            fixedKeyFramePose2.cast<double>();
        g2o::Sim3 Siw(cameraPose_worldToCamera.unit_quaternion(),
                      cameraPose_worldToCamera.translation(),
                      1.0);

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

    /* Loop over non-fixed KeyFrames */
    for (keyframe::KeyFrame *p_fixedKeyFrame : nonFixedKeyFrames_in)
    {
        bool fixedKeyFrameIsBad3{};
        if (!(p_fixedKeyFrame == nullptr) &&
            p_fixedKeyFrame->isBad(fixedKeyFrameIsBad3) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                idCount)) /* It has already added in the corrected merge KFs */
            continue;

        g2o::VertexSim3Expmap *p_sim3Vertex = new g2o::VertexSim3Expmap();

        Sophus::SE3f fixedKeyFramePose3{};
        if (p_fixedKeyFrame->getPose(fixedKeyFramePose3) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d cameraPose_worldToCamera =
            fixedKeyFramePose3.cast<double>();
        g2o::Sim3 Siw(cameraPose_worldToCamera.unit_quaternion(),
                      cameraPose_worldToCamera.translation(),
                      1.0);

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

    std::vector<keyframe::KeyFrame *> mapKeyFrames;
    std::set<keyframe::KeyFrame *>    keyFrames;
    mapKeyFrames.reserve(fixedKeyFrames_in.size() +
                         fixedCorrectedKeyFrames_in.size() +
                         nonFixedKeyFrames_in.size());

    const auto appendOptimizedKeyFrames =
        [&optimizer, &mapKeyFrames, &keyFrames](
            const std::vector<keyframe::KeyFrame *> &keyFrames_in)
    {
        for (keyframe::KeyFrame *p_keyFrame : keyFrames_in)
        {
            bool keyFrameIsBad{};
            if (!(p_keyFrame == nullptr) &&
                p_keyFrame->isBad(keyFrameIsBad) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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

    for (keyframe::KeyFrame *p_fixedKeyFrame : mapKeyFrames)
    {
        int       connectionCount = 0;
        const int idCount         = p_fixedKeyFrame->id;

        g2o::Sim3 correctedSwi;
        g2o::Sim3 Swi;

        if (goodPoses[idCount])
            correctedSwi = vCorrectedSwc[idCount];
        if (badPoses[idCount])
            Swi = vScw[idCount].inverse();

        keyframe::KeyFrame *p_parentKeyFrame = nullptr;
        if (p_fixedKeyFrame->getParent(p_parentKeyFrame) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParent returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Spanning tree edge */
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

        /* Loop edges */
        std::set<keyframe::KeyFrame *> loopEdges{};
        if (p_fixedKeyFrame->getLoopEdges(loopEdges) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLoopEdges returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (std::set<keyframe::KeyFrame *>::const_iterator
                 sit  = loopEdges.begin(),
                 send = loopEdges.end();
             sit != send;
             sit++)
        {
            keyframe::KeyFrame *p_loopKeyFrame = *sit;
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

        /* Covisibility graph edges */
        std::vector<keyframe::KeyFrame *> connectedKeyFrames{};
        if (p_fixedKeyFrame->getCovisiblesByWeight(minimumFeature,
                                                   connectedKeyFrames) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCovisiblesByWeight returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (std::vector<keyframe::KeyFrame *>::const_iterator vit =
                 connectedKeyFrames.begin();
             vit != connectedKeyFrames.end();
             vit++)
        {
            keyframe::KeyFrame *pKFn = *vit;
            bool                fixedKeyFrameHasChild{};
            if ((pKFn && pKFn != p_parentKeyFrame) &&
                p_fixedKeyFrame->hasChild(pKFn, fixedKeyFrameHasChild) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
            if (system::Verbose::printMess("Opt_Essential: KF " +
                                       std::to_string(p_fixedKeyFrame->id) +
                                       " has 0 connections",
                                   system::Verbose::VERBOSITY_DEBUG) !=
                system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    /* Optimize! */
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    std::unique_lock<std::mutex> lock(p_map->mapUpdateMutex);

    /* Inform the user */
    std::cout << "\n[Optimizer]" << std::endl;

    /*!
     * Loop over non-fixed KeyFrames to correct them
     * [hint] Sim3 [sR t | 0 1] --> SE3 [R t/s| 0 1]
     */
    std::cout << "- Correcting the poses of KeyFrames ..." << std::endl;
    for (keyframe::KeyFrame *p_fixedKeyFrame : nonFixedKeyFrames_in)
    {
        bool fixedKeyFrameIsBad4{};
        if (!(p_fixedKeyFrame == nullptr) &&
            p_fixedKeyFrame->isBad(fixedKeyFrameIsBad4) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
        Sophus::SE3d keyFramePose_worldToCamera(CorrectedSiw.rotation(),
                                                CorrectedSiw.translation() / s);

        Sophus::SE3f fixedKeyFramePose4{};
        if (p_fixedKeyFrame->getPose(fixedKeyFramePose4) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_fixedKeyFrame->tcwBefMerge = fixedKeyFramePose4;
        Sophus::SE3f fixedKeyFramePoseInverse{};
        if (p_fixedKeyFrame->getPoseInverse(fixedKeyFramePoseInverse) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        p_fixedKeyFrame->twcBefMerge = fixedKeyFramePoseInverse;
        if (p_fixedKeyFrame->setPose(
                keyFramePose_worldToCamera.cast<float>()) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    /*!
     * Transform to "non-optimized" reference keyframe pose and transform back
     * with optimized pose
     */
    std::cout << "- Correcting the poses of 3D mapped points ..." << std::endl;
    for (map_point::MapPoint *p_mapPoint : nonCorrectedMapPoints_in)
    {
        bool mapPointIsBad{};
        if (!(p_mapPoint == nullptr) &&
            p_mapPoint->isBad(mapPointIsBad) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
            [&badPoses](keyframe::KeyFrame *p_keyFrame_in)
        {
            bool keyFrameIsBad{};
            if ((p_keyFrame_in != nullptr) &&
                p_keyFrame_in->isBad(keyFrameIsBad) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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

        keyframe::KeyFrame *p_referenceKeyFrame = nullptr;
        if (p_mapPoint->getReferenceKeyFrame(p_referenceKeyFrame) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getReferenceKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (!canCorrectFromReference(p_referenceKeyFrame))
        {
            p_referenceKeyFrame = nullptr;

            std::map<keyframe::KeyFrame *, std::tuple<int, int>> observations{};
            if (p_mapPoint->getObservations(observations) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
            if (system::Verbose::printMess("MP " + std::to_string(p_mapPoint->id) +
                                       " without a valid reference KF",
                                   system::Verbose::VERBOSITY_DEBUG) !=
                system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
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
            Sophus::SE3f referenceKeyFramePose_cameraToWorld{};
            if (p_referenceKeyFrame->getPoseInverse(
                    referenceKeyFramePose_cameraToWorld) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            Eigen::Vector3f mapPointWorldPos{};
            if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3f eigCorrectedP3Dw =
                referenceKeyFramePose_cameraToWorld *
                TNonCorrectedwr.inverse() * mapPointWorldPos;
            if (p_mapPoint->setWorldPos(eigCorrectedP3Dw) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->updateNormalAndDepth() !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3f normal_mergeWorld{};
            if (p_mapPoint->getNormal(normal_mergeWorld) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getNormal returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->setWorldPos(
                    mergeTransform_mergeWorldToCurrentWorld_in
                        .map(position_mergeWorld_m.cast<double>())
                        .cast<float>()) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->setNormalVector(
                    mergeTransform_mergeWorldToCurrentWorld_in.rotation()
                        .cast<float>() *
                    normal_mergeWorld) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNormalVector returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_mapPoint->updateNormalAndDepth() !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
    utils::utils::Utils::KeyFramePoseMap keyFramePosesBefore_worldToCamera;
    utils::utils::Utils::KeyFramePoseMap keyFramePosesAfter_worldToCamera;

    const auto appendSemanticDeformationNodes =
        [&keyFramePosesBefore_worldToCamera, &keyFramePosesAfter_worldToCamera](
            const std::vector<keyframe::KeyFrame *> &keyFrames_in)
    {
        for (keyframe::KeyFrame *p_keyFrame : keyFrames_in)
        {
            bool keyFrameIsBad{};
            if (!(p_keyFrame == nullptr) &&
                p_keyFrame->isBad(keyFrameIsBad) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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

            const Sophus::SE3d poseBefore_worldToCamera =
                p_keyFrame->tcwBefMerge.cast<double>();
            Sophus::SE3f keyFramePose{};
            if (p_keyFrame->getPose(keyFramePose) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            const Sophus::SE3d poseAfter_worldToCamera =
                keyFramePose.cast<double>();

            keyFramePosesBefore_worldToCamera.insert_or_assign(
                p_keyFrame,
                g2o::Sim3(poseBefore_worldToCamera.unit_quaternion(),
                          poseBefore_worldToCamera.translation(),
                          1.0));
            keyFramePosesAfter_worldToCamera.insert_or_assign(
                p_keyFrame,
                g2o::Sim3(poseAfter_worldToCamera.unit_quaternion(),
                          poseAfter_worldToCamera.translation(),
                          1.0));
        }
    };

    appendSemanticDeformationNodes(fixedCorrectedKeyFrames_in);
    appendSemanticDeformationNodes(nonFixedKeyFrames_in);

    if (p_sourceMap_in != nullptr)
    {
        if (utils::utils::Utils::propagateSemanticPoseCorrections(
                p_sourceMap_in,
                keyFramePosesBefore_worldToCamera,
                keyFramePosesAfter_worldToCamera,
                mergeTransform_mergeWorldToCurrentWorld_in) !=
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

OptimizerStatus Optimizer::optimizeEssentialGraph(
    map::Map                                         *p_map_inout,
    keyframe::KeyFrame                               *p_loopKeyFrame_in,
    keyframe::KeyFrame                               *p_currentKeyFrame_in,
    const loop_closing::LoopClosing::KeyFrameAndPose &NonCorrectedSim3_in,
    const loop_closing::LoopClosing::KeyFrameAndPose &CorrectedSim3_in,
    const std::map<keyframe::KeyFrame *, std::set<keyframe::KeyFrame *>>
               &loopConnections_in,
    const bool &isScaleFixed_in)
{
    /* Setup optimizer */
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType *p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(p_linearSolver);
    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    p_solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(p_solver);

    std::vector<keyframe::KeyFrame *> keyFrames{};
    if (p_map_inout->getAllKeyFrames(keyFrames) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<map_point::MapPoint *> mapPoints{};
    if (p_map_inout->getAllMapPoints(mapPoints) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    unsigned long maximumKeyFrameIdCountValue{};
    if (p_map_inout->getMaxKeyFrameId(maximumKeyFrameIdCountValue) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMaxKeyFrameId returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const unsigned int maximumKeyFrameIdCount =
        static_cast<unsigned int>(maximumKeyFrameIdCountValue);

    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(
        maximumKeyFrameIdCount + 1);
    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
        maximumKeyFrameIdCount + 1);
    std::vector<g2o::VertexSim3Expmap *> vertices(maximumKeyFrameIdCount + 1);

    std::vector<Eigen::Vector3d> zvectors(maximumKeyFrameIdCount +
                                          1); /* For debugging */
    Eigen::Vector3d              z_vec;
    z_vec << 0.0, 0.0, 1.0;

    const int minimumFeature = 100;

    /* Set keyframe::KeyFrame vertices */
    for (size_t keyFrameIndex = 0, iend = keyFrames.size();
         keyFrameIndex < iend;
         keyFrameIndex++)
    {
        keyframe::KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];
        bool                keyFrameIsBad{};
        if (p_keyFrame->isBad(keyFrameIsBad) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (keyFrameIsBad)
            continue;
        g2o::VertexSim3Expmap *p_sim3Vertex = new g2o::VertexSim3Expmap();

        const int idCount = p_keyFrame->id;

        loop_closing::LoopClosing::KeyFrameAndPose::const_iterator
            correctedPoseIt = CorrectedSim3_in.find(p_keyFrame);

        if (correctedPoseIt != CorrectedSim3_in.end())
        {
            vScw[idCount] = correctedPoseIt->second;
            p_sim3Vertex->setEstimate(correctedPoseIt->second);
        }
        else
        {
            Sophus::SE3f keyFramePose{};
            if (p_keyFrame->getPose(keyFramePose) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            Sophus::SE3d cameraPose_worldToCamera = keyFramePose.cast<double>();
            g2o::Sim3    Siw(cameraPose_worldToCamera.unit_quaternion(),
                          cameraPose_worldToCamera.translation(),
                          1.0);
            vScw[idCount] = Siw;
            p_sim3Vertex->setEstimate(Siw);
        }

        unsigned long mapInitKeyFrameId{};
        if (p_map_inout->getInitKeyFrameId(mapInitKeyFrameId) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getInitKeyFrameId returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_keyFrame->id == mapInitKeyFrameId)
            p_sim3Vertex->setFixed(true);

        p_sim3Vertex->setId(idCount);
        p_sim3Vertex->setMarginalized(false);
        p_sim3Vertex->_fix_scale = isScaleFixed_in;

        optimizer.addVertex(p_sim3Vertex);
        /* For debugging */
        zvectors[idCount] = vScw[idCount].rotation() * z_vec;

        vertices[idCount] = p_sim3Vertex;
    }

    std::set<std::pair<long unsigned int, long unsigned int>> insertedEdges;

    const Eigen::Matrix<double, 7, 7> matrixLambda =
        Eigen::Matrix<double, 7, 7>::Identity();

    /* Set Loop edges */
    int loopCount = 0;
    for (std::map<keyframe::KeyFrame *,
                  std::set<keyframe::KeyFrame *>>::const_iterator
             mit  = loopConnections_in.begin(),
             mend = loopConnections_in.end();
         mit != mend;
         mit++)
    {
        keyframe::KeyFrame                   *p_keyFrame  = mit->first;
        const long unsigned int               idCount     = p_keyFrame->id;
        const std::set<keyframe::KeyFrame *> &connections = mit->second;
        const g2o::Sim3                       Siw         = vScw[idCount];
        const g2o::Sim3                       Swi         = Siw.inverse();

        for (std::set<keyframe::KeyFrame *>::const_iterator
                 sit  = connections.begin(),
                 send = connections.end();
             sit != send;
             sit++)
        {
            const long unsigned int loopKeyFrameId = (*sit)->id;
            int                     keyFrameWeight{};
            if (((idCount != p_currentKeyFrame_in->id ||
                  loopKeyFrameId != p_loopKeyFrame_in->id)) &&
                p_keyFrame->getWeight(*sit, keyFrameWeight) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWeight returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if ((idCount != p_currentKeyFrame_in->id ||
                 loopKeyFrameId != p_loopKeyFrame_in->id) &&
                keyFrameWeight < minimumFeature)
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
            insertedEdges.insert(
                std::make_pair(std::min(idCount, loopKeyFrameId),
                               std::max(idCount, loopKeyFrameId)));
        }
    }

    /* Set normal edges */
    for (size_t keyFrameIndex = 0, iend = keyFrames.size();
         keyFrameIndex < iend;
         keyFrameIndex++)
    {
        keyframe::KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];

        const int idCount = p_keyFrame->id;

        g2o::Sim3 Swi;

        loop_closing::LoopClosing::KeyFrameAndPose::const_iterator iti =
            NonCorrectedSim3_in.find(p_keyFrame);

        if (iti != NonCorrectedSim3_in.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[idCount].inverse();

        keyframe::KeyFrame *p_parentKeyFrame = nullptr;
        if (p_keyFrame->getParent(p_parentKeyFrame) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParent returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        /* Spanning tree edge */
        if (p_parentKeyFrame)
        {
            int loopKeyFrameId = p_parentKeyFrame->id;

            g2o::Sim3 Sjw;

            loop_closing::LoopClosing::KeyFrameAndPose::const_iterator itj =
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

        /* Loop edges */
        std::set<keyframe::KeyFrame *> loopEdges{};
        if (p_keyFrame->getLoopEdges(loopEdges) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getLoopEdges returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (std::set<keyframe::KeyFrame *>::const_iterator
                 sit  = loopEdges.begin(),
                 send = loopEdges.end();
             sit != send;
             sit++)
        {
            keyframe::KeyFrame *p_loopKeyFrame = *sit;
            if (p_loopKeyFrame->id < p_keyFrame->id)
            {
                g2o::Sim3 Slw;

                loop_closing::LoopClosing::KeyFrameAndPose::const_iterator itl =
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

        /* Covisibility graph edges */
        std::vector<keyframe::KeyFrame *> connectedKeyFrames{};
        if (p_keyFrame->getCovisiblesByWeight(minimumFeature,
                                              connectedKeyFrames) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCovisiblesByWeight returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (std::vector<keyframe::KeyFrame *>::const_iterator vit =
                 connectedKeyFrames.begin();
             vit != connectedKeyFrames.end();
             vit++)
        {
            keyframe::KeyFrame *pKFn = *vit;
            bool                keyFrameHasChild{};
            if ((pKFn && pKFn != p_parentKeyFrame) &&
                p_keyFrame->hasChild(pKFn, keyFrameHasChild) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasChild returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (pKFn && pKFn != p_parentKeyFrame &&
                !keyFrameHasChild /*&& !sLoopEdges.count(pKFn)*/)
            {
                bool pKFnIsBad{};
                if (pKFn->isBad(pKFnIsBad) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!pKFnIsBad && pKFn->id < p_keyFrame->id)
                {
                    if (insertedEdges.count(
                            std::make_pair(std::min(p_keyFrame->id, pKFn->id),
                                           std::max(p_keyFrame->id, pKFn->id))))
                        continue;

                    g2o::Sim3 Snw;

                    loop_closing::LoopClosing::KeyFrameAndPose::const_iterator
                        itn = NonCorrectedSim3_in.find(pKFn);

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

        /* Inertial edges if inertial */
        if (p_keyFrame->isImu && p_keyFrame->p_prevKF)
        {
            g2o::Sim3                                                  Spw;
            loop_closing::LoopClosing::KeyFrameAndPose::const_iterator itp =
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
    std::unique_lock<std::mutex> lock(p_map_inout->mapUpdateMutex);

    /* SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1] */
    for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
         keyFrameIndex++)
    {
        keyframe::KeyFrame *p_mapKeyFrame = keyFrames[keyFrameIndex];

        const int idCount = p_mapKeyFrame->id;

        g2o::VertexSim3Expmap *p_sim3Vertex =
            static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(idCount));
        g2o::Sim3 CorrectedSiw = p_sim3Vertex->estimate();
        vCorrectedSwc[idCount] = CorrectedSiw.inverse();
        double s               = CorrectedSiw.scale();
        s                      = (s == 0.0) ? 1.0 : s;

        Sophus::SE3f keyFramePose_worldToCamera(
            CorrectedSiw.rotation().cast<float>(),
            CorrectedSiw.translation().cast<float>() / s);
        if (p_mapKeyFrame->setPose(keyFramePose_worldToCamera) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    /*!
     * Correct points. Transform to "non-optimized" reference keyframe pose and
     * transform back with optimized pose
     */
    for (size_t keyFrameIndex = 0, iend = mapPoints.size();
         keyFrameIndex < iend;
         keyFrameIndex++)
    {
        map_point::MapPoint *p_mapPoint = mapPoints[keyFrameIndex];

        bool mapPointIsBad{};
        if (p_mapPoint->isBad(mapPointIsBad) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad)
            continue;

        int nIDr;
        if (p_mapPoint->correctedByKeyFrameId == p_currentKeyFrame_in->id)
        {
            nIDr = p_mapPoint->correctedReferenceKeyFrameId;
        }
        else
        {
            keyframe::KeyFrame *p_referenceKeyFrame = nullptr;
            if (p_mapPoint->getReferenceKeyFrame(p_referenceKeyFrame) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getReferenceKeyFrame returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            nIDr = p_referenceKeyFrame->id;
        }

        g2o::Sim3 Srw          = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Vector3f mapPointWorldPos{};
        if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Matrix<double, 3, 1> eigP3Dw = mapPointWorldPos.cast<double>();
        Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
            correctedSwr.map(Srw.map(eigP3Dw));
        if (p_mapPoint->setWorldPos(eigCorrectedP3Dw.cast<float>()) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_mapPoint->updateNormalAndDepth() !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateNormalAndDepth returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    if (p_map_inout->increaseChangeIndex() != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: increaseChangeIndex returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} /* namespace optimizer */
} /* namespace core */
} /* namespace vs_graphs */
