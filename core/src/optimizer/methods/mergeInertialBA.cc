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
 * @file            mergeInertialBA.cc
 *
 * @brief           Implements Optimizer::mergeInertialBA(), declared in
 *                  Optimizer.h.
 */

#include "optimizer/objects/OptimizerClass.h"
#include "g2o_types/edge_acc_rw/edge_acc_rw.h"
#include "g2o_types/edge_gyro_rw/edge_gyro_rw.h"
#include "imu_types/bias/bias.h"
#include "imu_types/preintegrated/objects/PreintegratedStatusEnum.h"
#include "keyframe/keyframe.h"
#include "keyframe/objects/KeyFrameStatusEnum.h"
#include "loop_closing/loop_closing.h"
#include "map/map.h"
#include "map/objects/MapStatusEnum.h"
#include "map_point/map_point.h"
#include "map_point/objects/MapPointStatusEnum.h"
#include "system/objects/VerboseStatusEnum.h"

#include "g2o_types/edge_inertial/edge_inertial.h"
#include "g2o_types/edge_mono/edge_mono.h"
#include "g2o_types/edge_stereo/edge_stereo.h"
#include "g2o_types/vertex_acc_bias/vertex_acc_bias.h"
#include "g2o_types/vertex_gyro_bias/vertex_gyro_bias.h"
#include "g2o_types/vertex_pose/vertex_pose.h"
#include "g2o_types/vertex_velocity/vertex_velocity.h"

#include "../private_functions/private_functions.h"
#include "system/system.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace optimizer
{

OptimizerStatus Optimizer::mergeInertialBA(
    keyframe::KeyFrame                         *p_currentKeyFrame_inout,
    keyframe::KeyFrame                         *p_mergeKeyFrame_inout,
    bool                                       *p_pbStopFlag_in,
    map::Map                                   *p_map_inout,
    loop_closing::LoopClosing::KeyFrameAndPose &corrPoses_inout)
{
    const int           Nd                = 6;
    const unsigned long maximumKeyFrameId = p_currentKeyFrame_inout->id;

    std::vector<keyframe::KeyFrame *> optimizableKeyFrames;
    optimizableKeyFrames.reserve(2 * Nd);

    /* For cov KFS, inertial parameters are not optimized */
    const int                         maximumCovisibleKeyFrameCount = 30;
    std::vector<keyframe::KeyFrame *> optimizableCovisibleKeyFrames;
    optimizableCovisibleKeyFrames.reserve(maximumCovisibleKeyFrameCount);

    /* Add sliding window for current KF */
    optimizableKeyFrames.push_back(p_currentKeyFrame_inout);
    p_currentKeyFrame_inout->baLocalKeyFrameId = p_currentKeyFrame_inout->id;
    for (int i = 1; i < Nd; i++)
    {
        if (optimizableKeyFrames.back()->p_prevKF)
        {
            optimizableKeyFrames.push_back(
                optimizableKeyFrames.back()->p_prevKF);
            optimizableKeyFrames.back()->baLocalKeyFrameId =
                p_currentKeyFrame_inout->id;
        }
        else
            break;
    }

    std::list<keyframe::KeyFrame *> fixedKeyFrames;
    if (optimizableKeyFrames.back()->p_prevKF)
    {
        optimizableCovisibleKeyFrames.push_back(
            optimizableKeyFrames.back()->p_prevKF);
        optimizableKeyFrames.back()->p_prevKF->baLocalKeyFrameId =
            p_currentKeyFrame_inout->id;
    }
    else
    {
        optimizableCovisibleKeyFrames.push_back(optimizableKeyFrames.back());
        optimizableKeyFrames.pop_back();
    }

    /* Add temporal neighbours to merge KF (previous and next KFs) */
    optimizableKeyFrames.push_back(p_mergeKeyFrame_inout);
    p_mergeKeyFrame_inout->baLocalKeyFrameId = p_currentKeyFrame_inout->id;

    /* Previous KFs */
    for (int i = 1; i < (Nd / 2); i++)
    {
        if (optimizableKeyFrames.back()->p_prevKF)
        {
            optimizableKeyFrames.push_back(
                optimizableKeyFrames.back()->p_prevKF);
            optimizableKeyFrames.back()->baLocalKeyFrameId =
                p_currentKeyFrame_inout->id;
        }
        else
            break;
    }

    /* We fix just once the old map */
    if (optimizableKeyFrames.back()->p_prevKF)
    {
        fixedKeyFrames.push_back(optimizableKeyFrames.back()->p_prevKF);
        optimizableKeyFrames.back()->p_prevKF->baFixedKeyFrameId =
            p_currentKeyFrame_inout->id;
    }
    else
    {
        optimizableKeyFrames.back()->baLocalKeyFrameId = 0;
        optimizableKeyFrames.back()->baFixedKeyFrameId =
            p_currentKeyFrame_inout->id;
        fixedKeyFrames.push_back(optimizableKeyFrames.back());
        optimizableKeyFrames.pop_back();
    }

    /* Next KFs */
    if (p_mergeKeyFrame_inout->p_nextKF)
    {
        optimizableKeyFrames.push_back(p_mergeKeyFrame_inout->p_nextKF);
        optimizableKeyFrames.back()->baLocalKeyFrameId =
            p_currentKeyFrame_inout->id;
    }

    while (optimizableKeyFrames.size() < (2 * Nd))
    {
        if (optimizableKeyFrames.back()->p_nextKF)
        {
            optimizableKeyFrames.push_back(
                optimizableKeyFrames.back()->p_nextKF);
            optimizableKeyFrames.back()->baLocalKeyFrameId =
                p_currentKeyFrame_inout->id;
        }
        else
            break;
    }

    int N = optimizableKeyFrames.size();

    /* Optimizable points seen by optimizable keyframes */
    std::list<map_point::MapPoint *>     localMapPointList;
    std::map<map_point::MapPoint *, int> localObservationCounts;
    for (int i = 0; i < N; i++)
    {
        std::vector<map_point::MapPoint *> mapPoints{};
        if (optimizableKeyFrames[i]->getMapPointMatches(mapPoints) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointMatches returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (std::vector<map_point::MapPoint *>::iterator
                 vit  = mapPoints.begin(),
                 vend = mapPoints.end();
             vit != vend;
             vit++)
        {
            /*!
             * Using mnBALocalForKF we avoid redundance here, one MP can not be
             * added several times to localMapPointList
             */
            map_point::MapPoint *p_mapPoint = *vit;
            if (p_mapPoint)
            {
                bool mapPointIsBad{};
                if (p_mapPoint->isBad(mapPointIsBad) !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!mapPointIsBad)
                {
                    if (p_mapPoint->baLocalKeyFrameId !=
                        p_currentKeyFrame_inout->id)
                    {
                        localObservationCounts[p_mapPoint] = 1;
                        localMapPointList.push_back(p_mapPoint);
                        p_mapPoint->baLocalKeyFrameId =
                            p_currentKeyFrame_inout->id;
                    }
                    else
                    {
                        localObservationCounts[p_mapPoint]++;
                    }
                }
            }
        }
    }

    std::vector<std::pair<map_point::MapPoint *, int>> pairs;
    pairs.reserve(localObservationCounts.size());
    for (std::map<map_point::MapPoint *, int>::iterator itr =
             localObservationCounts.begin();
         itr != localObservationCounts.end();
         ++itr)
        pairs.push_back(*itr);
    std::sort(pairs.begin(), pairs.end(), sortByVal);

    /*!
     * Fixed Keyframes. Keyframes that see Local MapPoints but that are not
     * Local Keyframes
     */
    int processedPairCount = 0;
    for (std::vector<std::pair<map_point::MapPoint *, int>>::iterator
             lit  = pairs.begin(),
             lend = pairs.end();
         lit != lend;
         lit++, processedPairCount++)
    {
        std::map<keyframe::KeyFrame *, std::tuple<int, int>> observations{};
        if (lit->first->getObservations(observations) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (processedPairCount >= maximumCovisibleKeyFrameCount)
            break;
        for (std::map<keyframe::KeyFrame *, std::tuple<int, int>>::iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            keyframe::KeyFrame *p_keyFrame = mit->first;

            if (p_keyFrame->baLocalKeyFrameId != p_currentKeyFrame_inout->id &&
                p_keyFrame->baFixedKeyFrameId !=
                    p_currentKeyFrame_inout
                        ->id) /* If optimizable or already included... */
            {
                p_keyFrame->baLocalKeyFrameId = p_currentKeyFrame_inout->id;
                bool keyFrameIsBad{};
                if (p_keyFrame->isBad(keyFrameIsBad) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!keyFrameIsBad)
                {
                    optimizableCovisibleKeyFrames.push_back(p_keyFrame);
                    break;
                }
            }
        }
    }

    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver;
    p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(p_linearSolver);

    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    p_solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(p_solver);
    optimizer.setVerbose(false);

    /* Set Local keyframe::KeyFrame vertices */
    N = optimizableKeyFrames.size();
    for (int i = 0; i < N; i++)
    {
        keyframe::KeyFrame *p_keyFrame = optimizableKeyFrames[i];

        g2o_types::vertex_pose::VertexPose *p_poseVertex = new g2o_types::vertex_pose::VertexPose(p_keyFrame);
        p_poseVertex->setId(p_keyFrame->id);
        p_poseVertex->setFixed(false);
        optimizer.addVertex(p_poseVertex);

        if (p_keyFrame->isImu)
        {
            g2o_types::vertex_velocity::VertexVelocity *p_velocityVertex = new g2o_types::vertex_velocity::VertexVelocity(p_keyFrame);
            p_velocityVertex->setId(maximumKeyFrameId + 3 * (p_keyFrame->id) +
                                    1);
            p_velocityVertex->setFixed(false);
            optimizer.addVertex(p_velocityVertex);
            g2o_types::vertex_gyro_bias::VertexGyroBias *p_gyroBiasVertex = new g2o_types::vertex_gyro_bias::VertexGyroBias(p_keyFrame);
            p_gyroBiasVertex->setId(maximumKeyFrameId + 3 * (p_keyFrame->id) +
                                    2);
            p_gyroBiasVertex->setFixed(false);
            optimizer.addVertex(p_gyroBiasVertex);
            g2o_types::vertex_acc_bias::VertexAccBias *p_accelerometerBiasVertex =
                new g2o_types::vertex_acc_bias::VertexAccBias(p_keyFrame);
            p_accelerometerBiasVertex->setId(maximumKeyFrameId +
                                             3 * (p_keyFrame->id) + 3);
            p_accelerometerBiasVertex->setFixed(false);
            optimizer.addVertex(p_accelerometerBiasVertex);
        }
    }

    /* Set Local cov keyframes vertices */
    int optimizableCovisibleCount = optimizableCovisibleKeyFrames.size();
    for (int i = 0; i < optimizableCovisibleCount; i++)
    {
        keyframe::KeyFrame *p_keyFrame = optimizableCovisibleKeyFrames[i];

        g2o_types::vertex_pose::VertexPose *p_poseVertex = new g2o_types::vertex_pose::VertexPose(p_keyFrame);
        p_poseVertex->setId(p_keyFrame->id);
        p_poseVertex->setFixed(false);
        optimizer.addVertex(p_poseVertex);

        if (p_keyFrame->isImu)
        {
            g2o_types::vertex_velocity::VertexVelocity *p_velocityVertex = new g2o_types::vertex_velocity::VertexVelocity(p_keyFrame);
            p_velocityVertex->setId(maximumKeyFrameId + 3 * (p_keyFrame->id) +
                                    1);
            p_velocityVertex->setFixed(false);
            optimizer.addVertex(p_velocityVertex);
            g2o_types::vertex_gyro_bias::VertexGyroBias *p_gyroBiasVertex = new g2o_types::vertex_gyro_bias::VertexGyroBias(p_keyFrame);
            p_gyroBiasVertex->setId(maximumKeyFrameId + 3 * (p_keyFrame->id) +
                                    2);
            p_gyroBiasVertex->setFixed(false);
            optimizer.addVertex(p_gyroBiasVertex);
            g2o_types::vertex_acc_bias::VertexAccBias *p_accelerometerBiasVertex =
                new g2o_types::vertex_acc_bias::VertexAccBias(p_keyFrame);
            p_accelerometerBiasVertex->setId(maximumKeyFrameId +
                                             3 * (p_keyFrame->id) + 3);
            p_accelerometerBiasVertex->setFixed(false);
            optimizer.addVertex(p_accelerometerBiasVertex);
        }
    }

    /* Set Fixed keyframe::KeyFrame vertices */
    for (std::list<keyframe::KeyFrame *>::iterator lit = fixedKeyFrames.begin(),
                                                   lend = fixedKeyFrames.end();
         lit != lend;
         lit++)
    {
        keyframe::KeyFrame *p_keyFrame   = *lit;
        g2o_types::vertex_pose::VertexPose         *p_poseVertex = new g2o_types::vertex_pose::VertexPose(p_keyFrame);
        p_poseVertex->setId(p_keyFrame->id);
        p_poseVertex->setFixed(true);
        optimizer.addVertex(p_poseVertex);

        if (p_keyFrame->isImu)
        {
            g2o_types::vertex_velocity::VertexVelocity *p_velocityVertex = new g2o_types::vertex_velocity::VertexVelocity(p_keyFrame);
            p_velocityVertex->setId(maximumKeyFrameId + 3 * (p_keyFrame->id) +
                                    1);
            p_velocityVertex->setFixed(true);
            optimizer.addVertex(p_velocityVertex);
            g2o_types::vertex_gyro_bias::VertexGyroBias *p_gyroBiasVertex = new g2o_types::vertex_gyro_bias::VertexGyroBias(p_keyFrame);
            p_gyroBiasVertex->setId(maximumKeyFrameId + 3 * (p_keyFrame->id) +
                                    2);
            p_gyroBiasVertex->setFixed(true);
            optimizer.addVertex(p_gyroBiasVertex);
            g2o_types::vertex_acc_bias::VertexAccBias *p_accelerometerBiasVertex =
                new g2o_types::vertex_acc_bias::VertexAccBias(p_keyFrame);
            p_accelerometerBiasVertex->setId(maximumKeyFrameId +
                                             3 * (p_keyFrame->id) + 3);
            p_accelerometerBiasVertex->setFixed(true);
            optimizer.addVertex(p_accelerometerBiasVertex);
        }
    }

    /* Create intertial constraints */
    std::vector<g2o_types::edge_inertial::EdgeInertial *> vei(N, nullptr);
    std::vector<g2o_types::edge_gyro_rw::EdgeGyroRW *>   vegr(N, nullptr);
    std::vector<g2o_types::edge_acc_rw::EdgeAccRW *>    vear(N, nullptr);
    for (int i = 0; i < N; i++)
    {
        keyframe::KeyFrame *p_keyFrame = optimizableKeyFrames[i];

        if (!p_keyFrame->p_prevKF)
        {
            if (system::Verbose::printMess("NOT INERTIAL LINK TO PREVIOUS FRAME!!!!",
                                   system::Verbose::VERBOSITY_NORMAL) !=
                system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            continue;
        }
        if (p_keyFrame->isImu && p_keyFrame->p_prevKF->isImu &&
            p_keyFrame->p_imuPreintegrated)
        {
            imu_types::bias::Bias imuBias{};
            if (p_keyFrame->p_prevKF->getImuBias(imuBias) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getImuBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame->p_imuPreintegrated->setNewBias(imuBias) !=
                imu_types::preintegrated::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNewBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            g2o::HyperGraph::Vertex *p_firstPoseVertex =
                optimizer.vertex(p_keyFrame->p_prevKF->id);
            g2o::HyperGraph::Vertex *p_firstVelocityVertex = optimizer.vertex(
                maximumKeyFrameId + 3 * (p_keyFrame->p_prevKF->id) + 1);
            g2o::HyperGraph::Vertex *p_firstGyroBiasVertex = optimizer.vertex(
                maximumKeyFrameId + 3 * (p_keyFrame->p_prevKF->id) + 2);
            g2o::HyperGraph::Vertex *p_firstAccelerometerBiasVertex =
                optimizer.vertex(maximumKeyFrameId +
                                 3 * (p_keyFrame->p_prevKF->id) + 3);
            g2o::HyperGraph::Vertex *p_secondPoseVertex =
                optimizer.vertex(p_keyFrame->id);
            g2o::HyperGraph::Vertex *p_secondVelocityVertex =
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 1);
            g2o::HyperGraph::Vertex *p_secondGyroBiasVertex =
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 2);
            g2o::HyperGraph::Vertex *p_secondAccelerometerBiasVertex =
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 3);

            if (!p_firstPoseVertex || !p_firstVelocityVertex ||
                !p_firstGyroBiasVertex || !p_firstAccelerometerBiasVertex ||
                !p_secondPoseVertex || !p_secondVelocityVertex ||
                !p_secondGyroBiasVertex || !p_secondAccelerometerBiasVertex)
            {
                std::cerr << "Error " << p_firstPoseVertex << ", "
                          << p_firstVelocityVertex << ", "
                          << p_firstGyroBiasVertex << ", "
                          << p_firstAccelerometerBiasVertex << ", "
                          << p_secondPoseVertex << ", "
                          << p_secondVelocityVertex << ", "
                          << p_secondGyroBiasVertex << ", "
                          << p_secondAccelerometerBiasVertex << std::endl;
                continue;
            }

            vei[i] = new g2o_types::edge_inertial::EdgeInertial(p_keyFrame->p_imuPreintegrated);

            vei[i]->setVertex(0,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_firstPoseVertex));
            vei[i]->setVertex(1,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_firstVelocityVertex));
            vei[i]->setVertex(2,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_firstGyroBiasVertex));
            vei[i]->setVertex(3,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_firstAccelerometerBiasVertex));
            vei[i]->setVertex(4,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_secondPoseVertex));
            vei[i]->setVertex(5,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_secondVelocityVertex));

            g2o::RobustKernelHuber *p_rki = new g2o::RobustKernelHuber;
            vei[i]->setRobustKernel(p_rki);
            p_rki->setDelta(sqrt(16.92));
            optimizer.addEdge(vei[i]);

            vegr[i] = new g2o_types::edge_gyro_rw::EdgeGyroRW();
            vegr[i]->setVertex(0, p_firstGyroBiasVertex);
            vegr[i]->setVertex(1, p_secondGyroBiasVertex);
            Eigen::Matrix3d informationG =
                p_keyFrame->p_imuPreintegrated->C.block<3, 3>(9, 9)
                    .cast<double>()
                    .inverse();
            vegr[i]->setInformation(informationG);
            optimizer.addEdge(vegr[i]);

            vear[i] = new g2o_types::edge_acc_rw::EdgeAccRW();
            vear[i]->setVertex(0, p_firstAccelerometerBiasVertex);
            vear[i]->setVertex(1, p_secondAccelerometerBiasVertex);
            Eigen::Matrix3d informationA =
                p_keyFrame->p_imuPreintegrated->C.block<3, 3>(12, 12)
                    .cast<double>()
                    .inverse();
            vear[i]->setInformation(informationA);
            optimizer.addEdge(vear[i]);
        }
        else
        {
            if (system::Verbose::printMess("ERROR building inertial edge",
                                   system::Verbose::VERBOSITY_NORMAL) !=
                system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    if (system::Verbose::printMess("end inserting inertial edges",
                           system::Verbose::VERBOSITY_NORMAL) !=
        system::VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Set map_point::MapPoint vertices */
    const int expectedSizeCount =
        (N + optimizableCovisibleCount + fixedKeyFrames.size()) *
        localMapPointList.size();

    /* Mono */
    std::vector<g2o_types::edge_mono::EdgeMono *> edgesMonos;
    edgesMonos.reserve(expectedSizeCount);

    std::vector<keyframe::KeyFrame *> edgeKeyFrameMonos;
    edgeKeyFrameMonos.reserve(expectedSizeCount);

    std::vector<map_point::MapPoint *> mapPointEdgeMonos;
    mapPointEdgeMonos.reserve(expectedSizeCount);

    /* Stereo */
    std::vector<g2o_types::edge_stereo::EdgeStereo *> edgesStereos;
    edgesStereos.reserve(expectedSizeCount);

    std::vector<keyframe::KeyFrame *> edgeKeyFrameStereos;
    edgeKeyFrameStereos.reserve(expectedSizeCount);

    std::vector<map_point::MapPoint *> mapPointEdgeStereos;
    mapPointEdgeStereos.reserve(expectedSizeCount);

    const float thresholdHuberMono   = sqrt(5.991);
    const float chi2Mono2            = 5.991;
    const float thresholdHuberStereo = sqrt(7.815);
    const float chi2Stereo2          = 7.815;

    const unsigned long initialMapPointId = maximumKeyFrameId * 5;

    for (std::list<map_point::MapPoint *>::iterator
             lit  = localMapPointList.begin(),
             lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        map_point::MapPoint *p_mapPoint = *lit;
        if (!p_mapPoint)
            continue;

        g2o::VertexSBAPointXYZ *p_pointVertex = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f         mapPointWorldPos{};
        if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        p_pointVertex->setEstimate(mapPointWorldPos.cast<double>());

        unsigned long id = p_mapPoint->id + initialMapPointId + 1;
        p_pointVertex->setId(id);
        p_pointVertex->setMarginalized(true);
        optimizer.addVertex(p_pointVertex);

        std::map<keyframe::KeyFrame *, std::tuple<int, int>> observations{};
        if (p_mapPoint->getObservations(observations) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Create visual constraints */
        for (std::map<keyframe::KeyFrame *,
                      std::tuple<int, int>>::const_iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            keyframe::KeyFrame *p_keyFrame = mit->first;

            if (!p_keyFrame)
                continue;

            if ((p_keyFrame->baLocalKeyFrameId !=
                 p_currentKeyFrame_inout->id) &&
                (p_keyFrame->baFixedKeyFrameId != p_currentKeyFrame_inout->id))
                continue;

            if (p_keyFrame->id > maximumKeyFrameId)
            {
                continue;
            }

            if (optimizer.vertex(id) == nullptr ||
                optimizer.vertex(p_keyFrame->id) == nullptr)
                continue;

            bool keyFrameIsBad2{};
            if (p_keyFrame->isBad(keyFrameIsBad2) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!keyFrameIsBad2)
            {
                const cv::KeyPoint &keyPointUn =
                    p_keyFrame->keyPointsUndistorted[std::get<0>(mit->second)];

                if (p_keyFrame->uRight[std::get<0>(mit->second)] <
                    0) /* Monocular observation */
                {
                    Eigen::Matrix<double, 2, 1> observation;
                    observation << keyPointUn.pt.x, keyPointUn.pt.y;

                    g2o_types::edge_mono::EdgeMono *e = new g2o_types::edge_mono::EdgeMono();
                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(p_keyFrame->id)));
                    e->setMeasurement(observation);
                    const float &invSigma2 =
                        p_keyFrame->invLevelSigmaSquared[keyPointUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(thresholdHuberMono);
                    optimizer.addEdge(e);
                    edgesMonos.push_back(e);
                    edgeKeyFrameMonos.push_back(p_keyFrame);
                    mapPointEdgeMonos.push_back(p_mapPoint);
                }
                else /* stereo observation */
                {
                    const float rightKeyPointU =
                        p_keyFrame->uRight[std::get<0>(mit->second)];
                    Eigen::Matrix<double, 3, 1> observation;
                    observation << keyPointUn.pt.x, keyPointUn.pt.y,
                        rightKeyPointU;

                    g2o_types::edge_stereo::EdgeStereo *e = new g2o_types::edge_stereo::EdgeStereo();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(p_keyFrame->id)));
                    e->setMeasurement(observation);
                    const float &invSigma2 =
                        p_keyFrame->invLevelSigmaSquared[keyPointUn.octave];
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(thresholdHuberStereo);

                    optimizer.addEdge(e);
                    edgesStereos.push_back(e);
                    edgeKeyFrameStereos.push_back(p_keyFrame);
                    mapPointEdgeStereos.push_back(p_mapPoint);
                }
            }
        }
    }

    if (p_pbStopFlag_in)
        optimizer.setForceStopFlag(p_pbStopFlag_in);

    if (p_pbStopFlag_in)
        if (*p_pbStopFlag_in)
            return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;

    optimizer.initializeOptimization();
    optimizer.optimize(8);

    std::vector<std::pair<keyframe::KeyFrame *, map_point::MapPoint *>>
        vToErase;
    vToErase.reserve(edgesMonos.size() + edgesStereos.size());

    /*!
     * Check inlier observations
     * Mono
     */
    for (size_t i = 0, iend = edgesMonos.size(); i < iend; i++)
    {
        g2o_types::edge_mono::EdgeMono            *e          = edgesMonos[i];
        map_point::MapPoint *p_mapPoint = mapPointEdgeMonos[i];

        bool mapPointIsBad2{};
        if (p_mapPoint->isBad(mapPointIsBad2) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad2)
            continue;

        if (e->chi2() > chi2Mono2)
        {
            keyframe::KeyFrame *p_keyFrame = edgeKeyFrameMonos[i];
            vToErase.push_back(std::make_pair(p_keyFrame, p_mapPoint));
        }
    }

    /* Stereo */
    for (size_t i = 0, iend = edgesStereos.size(); i < iend; i++)
    {
        g2o_types::edge_stereo::EdgeStereo          *e          = edgesStereos[i];
        map_point::MapPoint *p_mapPoint = mapPointEdgeStereos[i];

        bool mapPointIsBad3{};
        if (p_mapPoint->isBad(mapPointIsBad3) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad3)
            continue;

        if (e->chi2() > chi2Stereo2)
        {
            keyframe::KeyFrame *p_keyFrame = edgeKeyFrameStereos[i];
            vToErase.push_back(std::make_pair(p_keyFrame, p_mapPoint));
        }
    }

    /* Get map::Map Mutex and erase outliers */
    std::unique_lock<std::mutex> lock(p_map_inout->mapUpdateMutex);
    if (!vToErase.empty())
    {
        for (size_t i = 0; i < vToErase.size(); i++)
        {
            keyframe::KeyFrame  *p_keyFrame        = vToErase[i].first;
            map_point::MapPoint *p_mapPointToErase = vToErase[i].second;
            if (p_keyFrame->eraseMapPointMatch(p_mapPointToErase) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPointMatch returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapPointToErase->eraseObservation(p_keyFrame) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseObservation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    /*!
     * Recover optimized data
     * Keyframes
     */
    for (int i = 0; i < N; i++)
    {
        keyframe::KeyFrame *p_keyFrame = optimizableKeyFrames[i];

        g2o_types::vertex_pose::VertexPose *p_poseVertex =
            static_cast<g2o_types::vertex_pose::VertexPose *>(optimizer.vertex(p_keyFrame->id));
        Sophus::SE3f cameraPose_worldToCamera(
            p_poseVertex->estimate().Rcw[0].cast<float>(),
            p_poseVertex->estimate().tcw[0].cast<float>());
        if (p_keyFrame->setPose(cameraPose_worldToCamera) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        Sophus::SE3f keyFramePose{};
        if (p_keyFrame->getPose(keyFramePose) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d keyFramePose_worldToCamera = keyFramePose.cast<double>();
        g2o::Sim3    g2oSiw(keyFramePose_worldToCamera.unit_quaternion(),
                         keyFramePose_worldToCamera.translation(),
                         1.0);
        corrPoses_inout[p_keyFrame] = g2oSiw;

        if (p_keyFrame->isImu)
        {
            g2o_types::vertex_velocity::VertexVelocity *p_velocityVertex = static_cast<g2o_types::vertex_velocity::VertexVelocity *>(
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 1));
            if (p_keyFrame->setVelocity(
                    p_velocityVertex->estimate().cast<float>()) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            g2o_types::vertex_gyro_bias::VertexGyroBias *p_gyroBiasVertex = static_cast<g2o_types::vertex_gyro_bias::VertexGyroBias *>(
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 2));
            g2o_types::vertex_acc_bias::VertexAccBias *p_accelerometerBiasVertex =
                static_cast<g2o_types::vertex_acc_bias::VertexAccBias *>(optimizer.vertex(
                    maximumKeyFrameId + 3 * (p_keyFrame->id) + 3));
            Eigen::Matrix<double, 6, 1> b;
            b << p_gyroBiasVertex->estimate(),
                p_accelerometerBiasVertex->estimate();
            if (p_keyFrame->setNewBias(
                    imu_types::bias::Bias(b[3], b[4], b[5], b[0], b[1], b[2])) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNewBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (int i = 0; i < optimizableCovisibleCount; i++)
    {
        keyframe::KeyFrame *p_keyFrame = optimizableCovisibleKeyFrames[i];

        g2o_types::vertex_pose::VertexPose *p_poseVertex =
            static_cast<g2o_types::vertex_pose::VertexPose *>(optimizer.vertex(p_keyFrame->id));
        Sophus::SE3f cameraPose_worldToCamera(
            p_poseVertex->estimate().Rcw[0].cast<float>(),
            p_poseVertex->estimate().tcw[0].cast<float>());
        if (p_keyFrame->setPose(cameraPose_worldToCamera) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        Sophus::SE3f keyFramePose2{};
        if (p_keyFrame->getPose(keyFramePose2) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3d keyFramePose_worldToCamera = keyFramePose2.cast<double>();
        g2o::Sim3    g2oSiw(keyFramePose_worldToCamera.unit_quaternion(),
                         keyFramePose_worldToCamera.translation(),
                         1.0);
        corrPoses_inout[p_keyFrame] = g2oSiw;

        if (p_keyFrame->isImu)
        {
            g2o_types::vertex_velocity::VertexVelocity *p_velocityVertex = static_cast<g2o_types::vertex_velocity::VertexVelocity *>(
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 1));
            if (p_keyFrame->setVelocity(
                    p_velocityVertex->estimate().cast<float>()) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            g2o_types::vertex_gyro_bias::VertexGyroBias *p_gyroBiasVertex = static_cast<g2o_types::vertex_gyro_bias::VertexGyroBias *>(
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 2));
            g2o_types::vertex_acc_bias::VertexAccBias *p_accelerometerBiasVertex =
                static_cast<g2o_types::vertex_acc_bias::VertexAccBias *>(optimizer.vertex(
                    maximumKeyFrameId + 3 * (p_keyFrame->id) + 3));
            Eigen::Matrix<double, 6, 1> b;
            b << p_gyroBiasVertex->estimate(),
                p_accelerometerBiasVertex->estimate();
            if (p_keyFrame->setNewBias(
                    imu_types::bias::Bias(b[3], b[4], b[5], b[0], b[1], b[2])) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNewBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    /* Points */
    for (std::list<map_point::MapPoint *>::iterator
             lit  = localMapPointList.begin(),
             lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        map_point::MapPoint    *p_mapPoint = *lit;
        g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<g2o::VertexSBAPointXYZ *>(
                optimizer.vertex(p_mapPoint->id + initialMapPointId + 1));
        if (p_mapPoint->setWorldPos(p_pointVertex->estimate().cast<float>()) !=
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
