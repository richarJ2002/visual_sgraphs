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
 * @file            optimizeEssentialGraph4DoF.cc
 *
 * @brief           Implements Optimizer::optimizeEssentialGraph4DoF(), declared
 *                  in Optimizer.h.
 */

#include "Optimizer.h"

#include "G2oTypes.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

OptimizerStatus Optimizer::optimizeEssentialGraph4DoF(
    Map                                              *p_map_inout,
    KeyFrame                                         *p_loopKeyFrame_in,
    KeyFrame                                         *p_currentKeyFrame_in,
    const LoopClosing::KeyFrameAndPose               &NonCorrectedSim3_in,
    const LoopClosing::KeyFrameAndPose               &CorrectedSim3_in,
    const std::map<KeyFrame *, std::set<KeyFrame *>> &loopConnections_in)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolverX::LinearSolverType *p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(p_linearSolver);

    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(p_solver);

    std::vector<KeyFrame *> keyFrames{};
    if (p_map_inout->getAllKeyFrames(keyFrames) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<MapPoint *> mapPoints{};
    if (p_map_inout->getAllMapPoints(mapPoints) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    unsigned long maximumKeyFrameIdCountValue{};
    if (p_map_inout->getMaxKeyFrameId(maximumKeyFrameIdCountValue) !=
        MapStatus::MAP_STATUS_SUCCESS)
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

    std::vector<VertexPose4DoF *> vertices(maximumKeyFrameIdCount + 1);

    const int minimumFeature = 100;
    // Set KeyFrame vertices
    for (size_t keyFrameIndex = 0, iend = keyFrames.size();
         keyFrameIndex < iend;
         keyFrameIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];
        bool      keyFrameIsBad{};
        if (p_keyFrame->isBad(keyFrameIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (keyFrameIsBad)
            continue;

        VertexPose4DoF *p_pose4DofVertex;

        const int idCount = p_keyFrame->id;

        LoopClosing::KeyFrameAndPose::const_iterator correctedPoseIt =
            CorrectedSim3_in.find(p_keyFrame);

        if (correctedPoseIt != CorrectedSim3_in.end())
        {
            vScw[idCount]       = correctedPoseIt->second;
            const g2o::Sim3 Swc = correctedPoseIt->second.inverse();
            Eigen::Matrix3d rotationCameraToWorld =
                Swc.rotation().toRotationMatrix();
            Eigen::Vector3d translationCameraToWorld = Swc.translation();
            p_pose4DofVertex = new VertexPose4DoF(rotationCameraToWorld,
                                                  translationCameraToWorld,
                                                  p_keyFrame);
        }
        else
        {
            Sophus::SE3f keyFramePose{};
            if (p_keyFrame->getPose(keyFramePose) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            Sophus::SE3d poseWorldToCamera = keyFramePose.cast<double>();
            g2o::Sim3    Siw(poseWorldToCamera.unit_quaternion(),
                          poseWorldToCamera.translation(),
                          1.0);

            vScw[idCount]    = Siw;
            p_pose4DofVertex = new VertexPose4DoF(p_keyFrame);
        }

        if (p_keyFrame == p_loopKeyFrame_in)
            p_pose4DofVertex->setFixed(true);

        p_pose4DofVertex->setId(idCount);
        p_pose4DofVertex->setMarginalized(false);

        optimizer.addVertex(p_pose4DofVertex);
        vertices[idCount] = p_pose4DofVertex;
    }
    std::set<std::pair<long unsigned int, long unsigned int>> insertedEdges;

    // Edge used in posegraph has still 6Dof, even if updates of camera poses
    // are just in 4DoF
    Eigen::Matrix<double, 6, 6> matrixLambda =
        Eigen::Matrix<double, 6, 6>::Identity();
    matrixLambda(0, 0) = 1e3;
    matrixLambda(1, 1) = 1e3;
    matrixLambda(0, 0) = 1e3;

    // Set Loop edges
    for (std::map<KeyFrame *, std::set<KeyFrame *>>::const_iterator
             mit  = loopConnections_in.begin(),
             mend = loopConnections_in.end();
         mit != mend;
         mit++)
    {
        KeyFrame                   *p_keyFrame  = mit->first;
        const long unsigned int     idCount     = p_keyFrame->id;
        const std::set<KeyFrame *> &connections = mit->second;
        const g2o::Sim3             Siw         = vScw[idCount];

        for (std::set<KeyFrame *>::const_iterator sit  = connections.begin(),
                                                  send = connections.end();
             sit != send;
             sit++)
        {
            const long unsigned int loopKeyFrameId = (*sit)->id;
            int                     keyFrameWeight{};
            if (((idCount != p_currentKeyFrame_in->id ||
                  loopKeyFrameId != p_loopKeyFrame_in->id)) &&
                p_keyFrame->getWeight(*sit, keyFrameWeight) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
            const g2o::Sim3 Sij = Siw * Sjw.inverse();
            Eigen::Matrix4d Tij;
            Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
            Tij.block<3, 1>(0, 3) = Sij.translation();
            Tij(3, 3)             = 1.;

            Edge4DoF *e = new Edge4DoF(Tij);
            e->setVertex(1,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(loopKeyFrameId)));
            e->setVertex(0,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(idCount)));

            e->information() = matrixLambda;
            optimizer.addEdge(e);

            insertedEdges.insert(
                std::make_pair(std::min(idCount, loopKeyFrameId),
                               std::max(idCount, loopKeyFrameId)));
        }
    }

    // 1. Set normal edges
    for (size_t keyFrameIndex = 0, iend = keyFrames.size();
         keyFrameIndex < iend;
         keyFrameIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];

        const int idCount = p_keyFrame->id;

        g2o::Sim3 Siw;

        // Use noncorrected poses for posegraph edges
        LoopClosing::KeyFrameAndPose::const_iterator iti =
            NonCorrectedSim3_in.find(p_keyFrame);

        if (iti != NonCorrectedSim3_in.end())
            Siw = iti->second;
        else
            Siw = vScw[idCount];

        // 1.1.0 Spanning tree edge
        KeyFrame *p_parentKeyFrame = static_cast<KeyFrame *>(nullptr);
        if (p_parentKeyFrame)
        {
            int loopKeyFrameId = p_parentKeyFrame->id;

            g2o::Sim3 Swj;

            LoopClosing::KeyFrameAndPose::const_iterator itj =
                NonCorrectedSim3_in.find(p_parentKeyFrame);

            if (itj != NonCorrectedSim3_in.end())
                Swj = (itj->second).inverse();
            else
                Swj = vScw[loopKeyFrameId].inverse();

            g2o::Sim3       Sij = Siw * Swj;
            Eigen::Matrix4d Tij;
            Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
            Tij.block<3, 1>(0, 3) = Sij.translation();
            Tij(3, 3)             = 1.;

            Edge4DoF *e = new Edge4DoF(Tij);
            e->setVertex(0,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(idCount)));
            e->setVertex(1,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(loopKeyFrameId)));
            e->information() = matrixLambda;
            optimizer.addEdge(e);
        }

        // 1.1.1 Inertial edges
        KeyFrame *p_previousKeyFrame = p_keyFrame->p_prevKF;
        if (p_previousKeyFrame)
        {
            int loopKeyFrameId = p_previousKeyFrame->id;

            g2o::Sim3 Swj;

            LoopClosing::KeyFrameAndPose::const_iterator itj =
                NonCorrectedSim3_in.find(p_previousKeyFrame);

            if (itj != NonCorrectedSim3_in.end())
                Swj = (itj->second).inverse();
            else
                Swj = vScw[loopKeyFrameId].inverse();

            g2o::Sim3       Sij = Siw * Swj;
            Eigen::Matrix4d Tij;
            Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
            Tij.block<3, 1>(0, 3) = Sij.translation();
            Tij(3, 3)             = 1.;

            Edge4DoF *e = new Edge4DoF(Tij);
            e->setVertex(0,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(idCount)));
            e->setVertex(1,
                         dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                             optimizer.vertex(loopKeyFrameId)));
            e->information() = matrixLambda;
            optimizer.addEdge(e);
        }

        // 1.2 Loop edges
        std::set<KeyFrame *> loopEdges{};
        if (p_keyFrame->getLoopEdges(loopEdges) !=
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
            if (p_loopKeyFrame->id < p_keyFrame->id)
            {
                g2o::Sim3 Swl;

                LoopClosing::KeyFrameAndPose::const_iterator itl =
                    NonCorrectedSim3_in.find(p_loopKeyFrame);

                if (itl != NonCorrectedSim3_in.end())
                    Swl = itl->second.inverse();
                else
                    Swl = vScw[p_loopKeyFrame->id].inverse();

                g2o::Sim3       Sil = Siw * Swl;
                Eigen::Matrix4d Til;
                Til.block<3, 3>(0, 0) = Sil.rotation().toRotationMatrix();
                Til.block<3, 1>(0, 3) = Sil.translation();
                Til(3, 3)             = 1.;

                Edge4DoF *e = new Edge4DoF(Til);
                e->setVertex(0,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(idCount)));
                e->setVertex(1,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(p_loopKeyFrame->id)));
                e->information() = matrixLambda;
                optimizer.addEdge(e);
            }
        }

        // 1.3 Covisibility graph edges
        std::vector<KeyFrame *> connectedKeyFrames{};
        if (p_keyFrame->getCovisiblesByWeight(minimumFeature,
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
            bool      keyFrameHasChild{};
            if ((pKFn && pKFn != p_parentKeyFrame &&
                 pKFn != p_previousKeyFrame && pKFn != p_keyFrame->p_nextKF) &&
                p_keyFrame->hasChild(pKFn, keyFrameHasChild) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasChild returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (pKFn && pKFn != p_parentKeyFrame &&
                pKFn != p_previousKeyFrame && pKFn != p_keyFrame->p_nextKF &&
                !keyFrameHasChild && !loopEdges.count(pKFn))
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
                if (!pKFnIsBad && pKFn->id < p_keyFrame->id)
                {
                    if (insertedEdges.count(
                            std::make_pair(std::min(p_keyFrame->id, pKFn->id),
                                           std::max(p_keyFrame->id, pKFn->id))))
                        continue;

                    g2o::Sim3 Swn;

                    LoopClosing::KeyFrameAndPose::const_iterator itn =
                        NonCorrectedSim3_in.find(pKFn);

                    if (itn != NonCorrectedSim3_in.end())
                        Swn = itn->second.inverse();
                    else
                        Swn = vScw[pKFn->id].inverse();

                    g2o::Sim3       Sin = Siw * Swn;
                    Eigen::Matrix4d Tin;
                    Tin.block<3, 3>(0, 0) = Sin.rotation().toRotationMatrix();
                    Tin.block<3, 1>(0, 3) = Sin.translation();
                    Tin(3, 3)             = 1.;
                    Edge4DoF *e           = new Edge4DoF(Tin);
                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(idCount)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(pKFn->id)));
                    e->information() = matrixLambda;
                    optimizer.addEdge(e);
                }
            }
        }
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);

    std::unique_lock<std::mutex> lock(p_map_inout->mapUpdateMutex);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
         keyFrameIndex++)
    {
        KeyFrame *p_mapKeyFrame = keyFrames[keyFrameIndex];

        const int idCount = p_mapKeyFrame->id;

        VertexPose4DoF *Vi =
            static_cast<VertexPose4DoF *>(optimizer.vertex(idCount));
        Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
        Eigen::Vector3d ti = Vi->estimate().tcw[0];

        g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
        vCorrectedSwc[idCount] = CorrectedSiw.inverse();

        Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation());
        if (p_mapKeyFrame->setPose(Tiw.cast<float>()) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and
    // transform back with optimized pose
    for (size_t keyFrameIndex = 0, iend = mapPoints.size();
         keyFrameIndex < iend;
         keyFrameIndex++)
    {
        MapPoint *p_mapPoint = mapPoints[keyFrameIndex];

        bool mapPointIsBad{};
        if (p_mapPoint->isBad(mapPointIsBad) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad)
            continue;

        int nIDr;

        KeyFrame *p_referenceKeyFrame = nullptr;
        if (p_mapPoint->getReferenceKeyFrame(p_referenceKeyFrame) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getReferenceKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        nIDr = p_referenceKeyFrame->id;

        g2o::Sim3 Srw          = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Vector3f mapPointWorldPos{};
        if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        if (p_mapPoint->updateNormalAndDepth() !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateNormalAndDepth returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    if (p_map_inout->increaseChangeIndex() != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: increaseChangeIndex returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
