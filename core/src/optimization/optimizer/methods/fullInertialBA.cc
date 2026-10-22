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
 * @file            fullInertialBA.cc
 *
 * @brief           Implements Optimizer::fullInertialBA(), declared in
 *                  Optimizer.h.
 */

#include "optimization/optimizer/objects/OptimizerClass.h"
#include "optimization/g2o_types/edge_acc_rw/edge_acc_rw.h"
#include "optimization/g2o_types/edge_gyro_rw/edge_gyro_rw.h"
#include "sensor_models/imu_types/bias/bias.h"
#include "sensor_models/imu_types/preintegrated/objects/PreintegratedStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"
#include "execution/system/objects/VerboseStatusEnum.h"

#include "optimization/g2o_types/edge_inertial/edge_inertial.h"
#include "optimization/g2o_types/edge_mono/edge_mono.h"
#include "optimization/g2o_types/edge_prior_acc/edge_prior_acc.h"
#include "optimization/g2o_types/edge_prior_gyro/edge_prior_gyro.h"
#include "optimization/g2o_types/edge_stereo/edge_stereo.h"
#include "optimization/g2o_types/vertex_acc_bias/vertex_acc_bias.h"
#include "optimization/g2o_types/vertex_gyro_bias/vertex_gyro_bias.h"
#include "optimization/g2o_types/vertex_pose/vertex_pose.h"
#include "optimization/g2o_types/vertex_velocity/vertex_velocity.h"

#include "../private_functions/private_functions.h"
#include "execution/system/system.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace optimizer
{

OptimizerStatus
    Optimizer::fullInertialBA(map::Map               *p_map_inout,
                              int                     iterationCount_in,
                              const bool              fixLocalKeyFrames_in,
                              const long unsigned int loopKeyFrameId_in,
                              bool                   *p_stopFlag_inout,
                              bool                    isImuInitialization_in,
                              float                   gyroBiasPriorWeight_in,
                              float                   accelBiasPriorWeight_in,
                              const std::atomic_bool *p_stopRequested_in)
{
    unsigned long maxKeyFrameIdValue{};
    if (p_map_inout->getMaxKeyFrameId(maxKeyFrameIdValue) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMaxKeyFrameId returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    long unsigned int maxKeyFrameId =
        static_cast<long unsigned int>(maxKeyFrameIdValue);
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

    if (keyFrames.empty())
    {
        return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
    }

    AtomicOptimizerStopBridge stopBridge(p_stopRequested_in, p_stopFlag_inout);

    /* Setup optimizer */
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver;

    p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *p_blockSolver = new g2o::BlockSolverX(p_linearSolver);

    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(p_blockSolver);
    p_solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(p_solver);
    optimizer.setVerbose(false);

    if (p_stopFlag_inout)
        optimizer.setForceStopFlag(p_stopFlag_inout);

    if (p_stopRequested_in != nullptr && p_stopFlag_inout != nullptr)
    {
        optimizer.addPreIterationAction(&stopBridge);
        optimizer.addPostIterationAction(&stopBridge);
    }

    int nonFixedKeyFrameCount = 0;

    /* Set keyframe::KeyFrame vertices */
    keyframe::KeyFrame *p_initializationKeyFrame = nullptr;
    for (size_t elementIndex = 0; elementIndex < keyFrames.size();
         elementIndex++)
    {
        keyframe::KeyFrame *p_keyFrame = keyFrames[elementIndex];
        if (p_keyFrame->id > maxKeyFrameId)
            continue;
        g2o_types::vertex_pose::VertexPose *p_poseVertex = new g2o_types::vertex_pose::VertexPose(p_keyFrame);
        p_poseVertex->setId(p_keyFrame->id);
        p_initializationKeyFrame = p_keyFrame;
        bool isKeyFrameFixed     = false;
        if (fixLocalKeyFrames_in)
        {
            isKeyFrameFixed =
                (p_keyFrame->baLocalKeyFrameId >= (maxKeyFrameId - 1)) ||
                (p_keyFrame->baFixedKeyFrameId >= (maxKeyFrameId - 1));
            if (!isKeyFrameFixed)
                nonFixedKeyFrameCount++;
            p_poseVertex->setFixed(isKeyFrameFixed);
        }
        if (elementIndex == 0) /* Fix the first keyframe at origin */
            p_poseVertex->setFixed(true);
        optimizer.addVertex(p_poseVertex);

        if (p_keyFrame->isImu)
        {
            g2o_types::vertex_velocity::VertexVelocity *p_velocityVertex = new g2o_types::vertex_velocity::VertexVelocity(p_keyFrame);
            p_velocityVertex->setId(maxKeyFrameId + 3 * (p_keyFrame->id) + 1);
            p_velocityVertex->setFixed(isKeyFrameFixed);
            optimizer.addVertex(p_velocityVertex);
            if (!isImuInitialization_in)
            {
                g2o_types::vertex_gyro_bias::VertexGyroBias *p_gyroBiasVertex =
                    new g2o_types::vertex_gyro_bias::VertexGyroBias(p_keyFrame);
                p_gyroBiasVertex->setId(maxKeyFrameId + 3 * (p_keyFrame->id) +
                                        2);
                p_gyroBiasVertex->setFixed(isKeyFrameFixed);
                optimizer.addVertex(p_gyroBiasVertex);
                g2o_types::vertex_acc_bias::VertexAccBias *p_accelBiasVertex =
                    new g2o_types::vertex_acc_bias::VertexAccBias(p_keyFrame);
                p_accelBiasVertex->setId(maxKeyFrameId + 3 * (p_keyFrame->id) +
                                         3);
                p_accelBiasVertex->setFixed(isKeyFrameFixed);
                optimizer.addVertex(p_accelBiasVertex);
            }
        }
    }

    if (isImuInitialization_in)
    {
        if (p_initializationKeyFrame == nullptr)
        {
            return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
        }

        g2o_types::vertex_gyro_bias::VertexGyroBias *p_gyroBiasVertex =
            new g2o_types::vertex_gyro_bias::VertexGyroBias(p_initializationKeyFrame);
        p_gyroBiasVertex->setId(4 * maxKeyFrameId + 2);
        p_gyroBiasVertex->setFixed(false);
        optimizer.addVertex(p_gyroBiasVertex);
        g2o_types::vertex_acc_bias::VertexAccBias *p_accelBiasVertex =
            new g2o_types::vertex_acc_bias::VertexAccBias(p_initializationKeyFrame);
        p_accelBiasVertex->setId(4 * maxKeyFrameId + 3);
        p_accelBiasVertex->setFixed(false);
        optimizer.addVertex(p_accelBiasVertex);
    }

    if (fixLocalKeyFrames_in)
    {
        if (nonFixedKeyFrameCount < 3)
            return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
    }

    /* IMU links */
    for (size_t elementIndex = 0; elementIndex < keyFrames.size();
         elementIndex++)
    {
        keyframe::KeyFrame *p_keyFrame = keyFrames[elementIndex];

        if (!p_keyFrame->p_prevKF)
        {
            if (system::Verbose::printMess("NOT INERTIAL LINK TO PREVIOUS FRAME!",
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

        if (p_keyFrame->p_prevKF && p_keyFrame->id <= maxKeyFrameId)
        {
            bool keyFrameIsBad{};
            if (p_keyFrame->isBad(keyFrameIsBad) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (keyFrameIsBad || p_keyFrame->p_prevKF->id > maxKeyFrameId)
                continue;
            if (p_keyFrame->isImu && p_keyFrame->p_prevKF->isImu)
            {
                imu_types::bias::Bias imuBias2{};
                if (p_keyFrame->p_prevKF->getImuBias(imuBias2) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getImuBias returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_keyFrame->p_imuPreintegrated->setNewBias(imuBias2) !=
                    imu_types::preintegrated::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setNewBias returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                g2o::HyperGraph::Vertex *p_previousPoseVertex =
                    optimizer.vertex(p_keyFrame->p_prevKF->id);
                g2o::HyperGraph::Vertex *p_previousVelocityVertex =
                    optimizer.vertex(maxKeyFrameId +
                                     3 * (p_keyFrame->p_prevKF->id) + 1);

                g2o::HyperGraph::Vertex *p_previousGyroBiasVertex;
                g2o::HyperGraph::Vertex *p_previousAccelBiasVertex;
                g2o::HyperGraph::Vertex *p_currentGyroBiasVertex;
                g2o::HyperGraph::Vertex *p_currentAccelBiasVertex;
                if (!isImuInitialization_in)
                {
                    p_previousGyroBiasVertex = optimizer.vertex(
                        maxKeyFrameId + 3 * (p_keyFrame->p_prevKF->id) + 2);
                    p_previousAccelBiasVertex = optimizer.vertex(
                        maxKeyFrameId + 3 * (p_keyFrame->p_prevKF->id) + 3);
                    p_currentGyroBiasVertex = optimizer.vertex(
                        maxKeyFrameId + 3 * (p_keyFrame->id) + 2);
                    p_currentAccelBiasVertex = optimizer.vertex(
                        maxKeyFrameId + 3 * (p_keyFrame->id) + 3);
                }
                else
                {
                    p_previousGyroBiasVertex =
                        optimizer.vertex(4 * maxKeyFrameId + 2);
                    p_previousAccelBiasVertex =
                        optimizer.vertex(4 * maxKeyFrameId + 3);
                }

                g2o::HyperGraph::Vertex *p_currentPoseVertex =
                    optimizer.vertex(p_keyFrame->id);
                g2o::HyperGraph::Vertex *p_currentVelocityVertex =
                    optimizer.vertex(maxKeyFrameId + 3 * (p_keyFrame->id) + 1);

                if (!isImuInitialization_in)
                {
                    if (!p_previousPoseVertex || !p_previousVelocityVertex ||
                        !p_previousGyroBiasVertex ||
                        !p_previousAccelBiasVertex || !p_currentPoseVertex ||
                        !p_currentVelocityVertex || !p_currentGyroBiasVertex ||
                        !p_currentAccelBiasVertex)
                    {
                        std::cout << "Error" << p_previousPoseVertex << ", "
                                  << p_previousVelocityVertex << ", "
                                  << p_previousGyroBiasVertex << ", "
                                  << p_previousAccelBiasVertex << ", "
                                  << p_currentPoseVertex << ", "
                                  << p_currentVelocityVertex << ", "
                                  << p_currentGyroBiasVertex << ", "
                                  << p_currentAccelBiasVertex << std::endl;
                        continue;
                    }
                }
                else
                {
                    if (!p_previousPoseVertex || !p_previousVelocityVertex ||
                        !p_previousGyroBiasVertex ||
                        !p_previousAccelBiasVertex || !p_currentPoseVertex ||
                        !p_currentVelocityVertex)
                    {
                        std::cout << "Error" << p_previousPoseVertex << ", "
                                  << p_previousVelocityVertex << ", "
                                  << p_previousGyroBiasVertex << ", "
                                  << p_previousAccelBiasVertex << ", "
                                  << p_currentPoseVertex << ", "
                                  << p_currentVelocityVertex << std::endl;
                        continue;
                    }
                }

                g2o_types::edge_inertial::EdgeInertial *p_inertialEdge =
                    new g2o_types::edge_inertial::EdgeInertial(p_keyFrame->p_imuPreintegrated);
                p_inertialEdge->setVertex(
                    0,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        p_previousPoseVertex));
                p_inertialEdge->setVertex(
                    1,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        p_previousVelocityVertex));
                p_inertialEdge->setVertex(
                    2,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        p_previousGyroBiasVertex));
                p_inertialEdge->setVertex(
                    3,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        p_previousAccelBiasVertex));
                p_inertialEdge->setVertex(
                    4,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        p_currentPoseVertex));
                p_inertialEdge->setVertex(
                    5,
                    dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                        p_currentVelocityVertex));

                g2o::RobustKernelHuber *p_inertialRobustKernel =
                    new g2o::RobustKernelHuber;
                p_inertialEdge->setRobustKernel(p_inertialRobustKernel);
                p_inertialRobustKernel->setDelta(sqrt(16.92));

                optimizer.addEdge(p_inertialEdge);

                if (!isImuInitialization_in)
                {
                    g2o_types::edge_gyro_rw::EdgeGyroRW *p_gyroRandomWalkEdge = new g2o_types::edge_gyro_rw::EdgeGyroRW();
                    p_gyroRandomWalkEdge->setVertex(0,
                                                    p_previousGyroBiasVertex);
                    p_gyroRandomWalkEdge->setVertex(1, p_currentGyroBiasVertex);
                    Eigen::Matrix3d gyroBiasInformationMatrix =
                        p_keyFrame->p_imuPreintegrated->C.block<3, 3>(9, 9)
                            .cast<double>()
                            .inverse();
                    p_gyroRandomWalkEdge->setInformation(
                        gyroBiasInformationMatrix);
                    p_gyroRandomWalkEdge->computeError();
                    optimizer.addEdge(p_gyroRandomWalkEdge);

                    g2o_types::edge_acc_rw::EdgeAccRW *p_accelRandomWalkEdge = new g2o_types::edge_acc_rw::EdgeAccRW();
                    p_accelRandomWalkEdge->setVertex(0,
                                                     p_previousAccelBiasVertex);
                    p_accelRandomWalkEdge->setVertex(1,
                                                     p_currentAccelBiasVertex);
                    Eigen::Matrix3d accelBiasInformationMatrix =
                        p_keyFrame->p_imuPreintegrated->C.block<3, 3>(12, 12)
                            .cast<double>()
                            .inverse();
                    p_accelRandomWalkEdge->setInformation(
                        accelBiasInformationMatrix);
                    p_accelRandomWalkEdge->computeError();
                    optimizer.addEdge(p_accelRandomWalkEdge);
                }
            }
            else
                std::cout << p_keyFrame->id << " or "
                          << p_keyFrame->p_prevKF->id << " no imu" << std::endl;
        }
    }

    if (isImuInitialization_in)
    {
        g2o::HyperGraph::Vertex *p_gyroBiasVertex =
            optimizer.vertex(4 * maxKeyFrameId + 2);
        g2o::HyperGraph::Vertex *p_accelBiasVertex =
            optimizer.vertex(4 * maxKeyFrameId + 3);

        /* Add prior to comon biases */
        Eigen::Vector3f biasPrior;
        biasPrior.setZero();

        g2o_types::edge_prior_acc::EdgePriorAcc *p_accelBiasPriorEdge = new g2o_types::edge_prior_acc::EdgePriorAcc(biasPrior);
        p_accelBiasPriorEdge->setVertex(
            0,
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_accelBiasVertex));
        double accelBiasPriorInformation = accelBiasPriorWeight_in;
        p_accelBiasPriorEdge->setInformation(accelBiasPriorInformation *
                                             Eigen::Matrix3d::Identity());
        optimizer.addEdge(p_accelBiasPriorEdge);

        g2o_types::edge_prior_gyro::EdgePriorGyro *p_gyroBiasPriorEdge = new g2o_types::edge_prior_gyro::EdgePriorGyro(biasPrior);
        p_gyroBiasPriorEdge->setVertex(
            0,
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_gyroBiasVertex));
        double gyroBiasPriorInformation = gyroBiasPriorWeight_in;
        p_gyroBiasPriorEdge->setInformation(gyroBiasPriorInformation *
                                            Eigen::Matrix3d::Identity());
        optimizer.addEdge(p_gyroBiasPriorEdge);
    }

    const float huberThresholdMono   = sqrt(5.991);
    const float huberThresholdStereo = sqrt(7.815);

    const unsigned long mapPointVertexIdBase = maxKeyFrameId * 5;

    std::vector<bool> mapPointExcludedFlags(mapPoints.size(), false);

    for (size_t elementIndex = 0; elementIndex < mapPoints.size();
         elementIndex++)
    {
        map_point::MapPoint    *p_mapPoint       = mapPoints[elementIndex];
        g2o::VertexSBAPointXYZ *p_mapPointVertex = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f         mapPointWorldPos{};
        if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        p_mapPointVertex->setEstimate(mapPointWorldPos.cast<double>());
        unsigned long mapPointVertexId =
            p_mapPoint->id + mapPointVertexIdBase + 1;
        p_mapPointVertex->setId(mapPointVertexId);
        p_mapPointVertex->setMarginalized(true);
        optimizer.addVertex(p_mapPointVertex);

        std::map<keyframe::KeyFrame *, std::tuple<int, int>> observations{};
        if (p_mapPoint->getObservations(observations) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        bool allVerticesFixed = true;

        /* Set edges */
        for (std::map<keyframe::KeyFrame *,
                      std::tuple<int, int>>::const_iterator
                 featureObservationIt  = observations.begin(),
                 featureObservationEnd = observations.end();
             featureObservationIt != featureObservationEnd;
             featureObservationIt++)
        {
            keyframe::KeyFrame *p_keyFrame = featureObservationIt->first;

            if (p_keyFrame->id > maxKeyFrameId)
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
                const int leftIndex = std::get<0>(featureObservationIt->second);
                cv::KeyPoint undistortedKeyPoint;

                if (leftIndex != -1 && p_keyFrame->uRight[std::get<0>(
                                           featureObservationIt->second)] <
                                           0) /* Monocular observation */
                {
                    undistortedKeyPoint =
                        p_keyFrame->keyPointsUndistorted[leftIndex];
                    Eigen::Matrix<double, 2, 1> observation;
                    observation << undistortedKeyPoint.pt.x,
                        undistortedKeyPoint.pt.y;

                    g2o_types::edge_mono::EdgeMono *p_edge = new g2o_types::edge_mono::EdgeMono(0);

                    g2o::OptimizableGraph::Vertex *p_poseVertex =
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(p_keyFrame->id));
                    if (allVerticesFixed)
                        if (!p_poseVertex->fixed())
                            allVerticesFixed = false;

                    p_edge->setVertex(
                        0,
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(mapPointVertexId)));
                    p_edge->setVertex(1, p_poseVertex);
                    p_edge->setMeasurement(observation);
                    const float inverseSigmaSquared =
                        p_keyFrame
                            ->invLevelSigmaSquared[undistortedKeyPoint.octave];

                    p_edge->setInformation(Eigen::Matrix2d::Identity() *
                                           inverseSigmaSquared);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    p_edge->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(huberThresholdMono);

                    optimizer.addEdge(p_edge);
                }
                else if (leftIndex != -1 && p_keyFrame->uRight[leftIndex] >=
                                                0) /* stereo observation */
                {
                    undistortedKeyPoint =
                        p_keyFrame->keyPointsUndistorted[leftIndex];
                    const float rightUCoordinate =
                        p_keyFrame->uRight[leftIndex];
                    Eigen::Matrix<double, 3, 1> observation;
                    observation << undistortedKeyPoint.pt.x,
                        undistortedKeyPoint.pt.y, rightUCoordinate;

                    g2o_types::edge_stereo::EdgeStereo *p_edge = new g2o_types::edge_stereo::EdgeStereo(0);

                    g2o::OptimizableGraph::Vertex *p_poseVertex =
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(p_keyFrame->id));
                    if (allVerticesFixed)
                        if (!p_poseVertex->fixed())
                            allVerticesFixed = false;

                    p_edge->setVertex(
                        0,
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                            optimizer.vertex(mapPointVertexId)));
                    p_edge->setVertex(1, p_poseVertex);
                    p_edge->setMeasurement(observation);
                    const float inverseSigmaSquared =
                        p_keyFrame
                            ->invLevelSigmaSquared[undistortedKeyPoint.octave];

                    p_edge->setInformation(Eigen::Matrix3d::Identity() *
                                           inverseSigmaSquared);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    p_edge->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(huberThresholdStereo);

                    optimizer.addEdge(p_edge);
                }

                if (p_keyFrame->p_camera2)
                { /* Monocular right observation */
                    int rightIndex = std::get<1>(featureObservationIt->second);

                    if (rightIndex != -1 &&
                        static_cast<size_t>(rightIndex) <
                            p_keyFrame->keyPointsRight.size())
                    {
                        rightIndex -= p_keyFrame->leftKeyPointCount;

                        Eigen::Matrix<double, 2, 1> observation;
                        undistortedKeyPoint =
                            p_keyFrame->keyPointsRight[rightIndex];
                        observation << undistortedKeyPoint.pt.x,
                            undistortedKeyPoint.pt.y;

                        g2o_types::edge_mono::EdgeMono *p_edge = new g2o_types::edge_mono::EdgeMono(1);

                        g2o::OptimizableGraph::Vertex *p_poseVertex =
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(p_keyFrame->id));
                        if (allVerticesFixed)
                            if (!p_poseVertex->fixed())
                                allVerticesFixed = false;

                        p_edge->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(mapPointVertexId)));
                        p_edge->setVertex(1, p_poseVertex);
                        p_edge->setMeasurement(observation);
                        const float inverseSigmaSquared =
                            p_keyFrame->invLevelSigmaSquared[undistortedKeyPoint
                                                                 .octave];
                        p_edge->setInformation(Eigen::Matrix2d::Identity() *
                                               inverseSigmaSquared);

                        g2o::RobustKernelHuber *p_robustKernel =
                            new g2o::RobustKernelHuber;
                        p_edge->setRobustKernel(p_robustKernel);
                        p_robustKernel->setDelta(huberThresholdMono);

                        optimizer.addEdge(p_edge);
                    }
                }
            }
        }

        if (allVerticesFixed)
        {
            optimizer.removeVertex(p_mapPointVertex);
            mapPointExcludedFlags[elementIndex] = true;
        }
    }

    if (p_stopFlag_inout)
        if (*p_stopFlag_inout)
            return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;

    optimizer.initializeOptimization();
    optimizer.optimize(iterationCount_in);
    optimizer.removePreIterationAction(&stopBridge);
    optimizer.removePostIterationAction(&stopBridge);

    /*!
     * Recover optimized data
     * Keyframes
     */
    for (size_t elementIndex = 0; elementIndex < keyFrames.size();
         elementIndex++)
    {
        keyframe::KeyFrame *p_keyFrame = keyFrames[elementIndex];
        if (p_keyFrame->id > maxKeyFrameId)
            continue;
        g2o_types::vertex_pose::VertexPose *p_poseVertex =
            static_cast<g2o_types::vertex_pose::VertexPose *>(optimizer.vertex(p_keyFrame->id));
        if (loopKeyFrameId_in == 0)
        {
            Sophus::SE3f cameraPose_worldToCamera(
                p_poseVertex->estimate().Rcw[0].cast<float>(),
                p_poseVertex->estimate().tcw[0].cast<float>());
            if (p_keyFrame->setPose(cameraPose_worldToCamera) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {
            p_keyFrame->tcwGBA =
                Sophus::SE3f(p_poseVertex->estimate().Rcw[0].cast<float>(),
                             p_poseVertex->estimate().tcw[0].cast<float>());
            p_keyFrame->baGlobalKeyFrameId = loopKeyFrameId_in;
        }
        if (p_keyFrame->isImu)
        {
            g2o_types::vertex_velocity::VertexVelocity *p_velocityVertex = static_cast<g2o_types::vertex_velocity::VertexVelocity *>(
                optimizer.vertex(maxKeyFrameId + 3 * (p_keyFrame->id) + 1));
            if (loopKeyFrameId_in == 0)
            {
                if (p_keyFrame->setVelocity(
                        p_velocityVertex->estimate().cast<float>()) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setVelocity returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
            else
            {
                p_keyFrame->vwbGBA = p_velocityVertex->estimate().cast<float>();
            }

            g2o_types::vertex_gyro_bias::VertexGyroBias *p_gyroBiasVertex;
            g2o_types::vertex_acc_bias::VertexAccBias  *p_accelBiasVertex;
            if (!isImuInitialization_in)
            {
                p_gyroBiasVertex = static_cast<g2o_types::vertex_gyro_bias::VertexGyroBias *>(
                    optimizer.vertex(maxKeyFrameId + 3 * (p_keyFrame->id) + 2));
                p_accelBiasVertex = static_cast<g2o_types::vertex_acc_bias::VertexAccBias *>(
                    optimizer.vertex(maxKeyFrameId + 3 * (p_keyFrame->id) + 3));
            }
            else
            {
                p_gyroBiasVertex = static_cast<g2o_types::vertex_gyro_bias::VertexGyroBias *>(
                    optimizer.vertex(4 * maxKeyFrameId + 2));
                p_accelBiasVertex = static_cast<g2o_types::vertex_acc_bias::VertexAccBias *>(
                    optimizer.vertex(4 * maxKeyFrameId + 3));
            }

            Eigen::Matrix<double, 6, 1> biasVector;
            biasVector << p_gyroBiasVertex->estimate(),
                p_accelBiasVertex->estimate();
            imu_types::bias::Bias imuBias(biasVector[3],
                              biasVector[4],
                              biasVector[5],
                              biasVector[0],
                              biasVector[1],
                              biasVector[2]);
            if (loopKeyFrameId_in == 0)
            {
                if (p_keyFrame->setNewBias(imuBias) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setNewBias returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
            else
            {
                p_keyFrame->biasGBA = imuBias;
            }
        }
    }

    /* Points */
    for (size_t elementIndex = 0; elementIndex < mapPoints.size();
         elementIndex++)
    {
        if (mapPointExcludedFlags[elementIndex])
            continue;

        map_point::MapPoint    *p_mapPoint = mapPoints[elementIndex];
        g2o::VertexSBAPointXYZ *p_mapPointVertex =
            static_cast<g2o::VertexSBAPointXYZ *>(
                optimizer.vertex(p_mapPoint->id + mapPointVertexIdBase + 1));

        if (loopKeyFrameId_in == 0)
        {
            if (p_mapPoint->setWorldPos(
                    p_mapPointVertex->estimate().cast<float>()) !=
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
        }
        else
        {
            p_mapPoint->posGBA = p_mapPointVertex->estimate().cast<float>();
            p_mapPoint->baGlobalKeyFrameId = loopKeyFrameId_in;
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
