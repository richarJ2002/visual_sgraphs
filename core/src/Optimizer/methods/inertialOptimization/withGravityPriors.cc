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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

OptimizerStatus Optimizer::inertialOptimization(Map             *p_map_in,
                                                Eigen::Vector3d &gyroBias_out,
                                                Eigen::Vector3d &accelBias_out,
                                                float gyroBiasPriorWeight_in,
                                                float accelBiasPriorWeight_in)
{
    int           iterationCount = 200; // Check number of iterations
    unsigned long maxKeyFrameIdValue{};
    if (p_map_in->getMaxKeyFrameId(maxKeyFrameIdValue) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMaxKeyFrameId returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    long unsigned int maxKeyFrameId =
        static_cast<long unsigned int>(maxKeyFrameIdValue);
    std::vector<KeyFrame *> keyFrames{};
    if (p_map_in->getAllKeyFrames(keyFrames) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    // Setup optimizer
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver;

    p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *p_blockSolver = new g2o::BlockSolverX(p_linearSolver);

    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(p_blockSolver);
    p_solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(p_solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for (size_t elementIndex = 0; elementIndex < keyFrames.size();
         elementIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[elementIndex];
        if (p_keyFrame->id > maxKeyFrameId)
            continue;
        VertexPose *p_poseVertex = new VertexPose(p_keyFrame);
        p_poseVertex->setId(p_keyFrame->id);
        p_poseVertex->setFixed(true);
        optimizer.addVertex(p_poseVertex);

        VertexVelocity *p_velocityVertex = new VertexVelocity(p_keyFrame);
        p_velocityVertex->setId(maxKeyFrameId + (p_keyFrame->id) + 1);
        p_velocityVertex->setFixed(false);

        optimizer.addVertex(p_velocityVertex);
    }

    // Biases
    VertexGyroBias *p_gyroBiasVertex = new VertexGyroBias(keyFrames.front());
    p_gyroBiasVertex->setId(maxKeyFrameId * 2 + 2);
    p_gyroBiasVertex->setFixed(false);
    optimizer.addVertex(p_gyroBiasVertex);

    VertexAccBias *p_accelBiasVertex = new VertexAccBias(keyFrames.front());
    p_accelBiasVertex->setId(maxKeyFrameId * 2 + 3);
    p_accelBiasVertex->setFixed(false);

    optimizer.addVertex(p_accelBiasVertex);
    // prior acc bias
    Eigen::Vector3f biasPrior;
    biasPrior.setZero();

    EdgePriorAcc *p_accelBiasPriorEdge = new EdgePriorAcc(biasPrior);
    p_accelBiasPriorEdge->setVertex(
        0,
        dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_accelBiasVertex));
    double accelBiasPriorInformation = accelBiasPriorWeight_in;
    p_accelBiasPriorEdge->setInformation(accelBiasPriorInformation *
                                         Eigen::Matrix3d::Identity());
    optimizer.addEdge(p_accelBiasPriorEdge);
    EdgePriorGyro *p_gyroBiasPriorEdge = new EdgePriorGyro(biasPrior);
    p_gyroBiasPriorEdge->setVertex(
        0,
        dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_gyroBiasVertex));
    double gyroBiasPriorInformation = gyroBiasPriorWeight_in;
    p_gyroBiasPriorEdge->setInformation(gyroBiasPriorInformation *
                                        Eigen::Matrix3d::Identity());
    optimizer.addEdge(p_gyroBiasPriorEdge);

    // Gravity and scale
    VertexGDir *p_gravityDirectionVertex =
        new VertexGDir(Eigen::Matrix3d::Identity());
    p_gravityDirectionVertex->setId(maxKeyFrameId * 2 + 4);
    p_gravityDirectionVertex->setFixed(true);
    optimizer.addVertex(p_gravityDirectionVertex);
    VertexScale *p_scaleVertex = new VertexScale(1.0);
    p_scaleVertex->setId(maxKeyFrameId * 2 + 5);
    p_scaleVertex->setFixed(true); // Fixed since scale is obtained from already
                                   // well initialized map
    optimizer.addVertex(p_scaleVertex);

    // Graph edges
    // IMU links with gravity and scale
    vector<EdgeInertialGS *> inertialEdges;
    inertialEdges.reserve(keyFrames.size());
    vector<pair<KeyFrame *, KeyFrame *>> usedKeyFramePairs;
    usedKeyFramePairs.reserve(keyFrames.size());

    for (size_t elementIndex = 0; elementIndex < keyFrames.size();
         elementIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[elementIndex];

        if (p_keyFrame->p_prevKF && p_keyFrame->id <= maxKeyFrameId)
        {
            bool keyFrameIsBad{};
            if (p_keyFrame->isBad(keyFrameIsBad) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (keyFrameIsBad || p_keyFrame->p_prevKF->id > maxKeyFrameId)
                continue;

            IMU::Bias imuBias2{};
            if (p_keyFrame->p_prevKF->getImuBias(imuBias2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getImuBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame->p_imuPreintegrated->setNewBias(imuBias2) !=
                IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNewBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            g2o::HyperGraph::Vertex *p_previousPoseVertex =
                optimizer.vertex(p_keyFrame->p_prevKF->id);
            g2o::HyperGraph::Vertex *p_previousVelocityVertex =
                optimizer.vertex(maxKeyFrameId + (p_keyFrame->p_prevKF->id) +
                                 1);
            g2o::HyperGraph::Vertex *p_currentPoseVertex =
                optimizer.vertex(p_keyFrame->id);
            g2o::HyperGraph::Vertex *p_currentVelocityVertex =
                optimizer.vertex(maxKeyFrameId + (p_keyFrame->id) + 1);
            g2o::HyperGraph::Vertex *p_gyroBiasVertex =
                optimizer.vertex(maxKeyFrameId * 2 + 2);
            g2o::HyperGraph::Vertex *p_accelBiasVertex =
                optimizer.vertex(maxKeyFrameId * 2 + 3);
            g2o::HyperGraph::Vertex *p_gravityDirectionVertex =
                optimizer.vertex(maxKeyFrameId * 2 + 4);
            g2o::HyperGraph::Vertex *p_scaleVertex =
                optimizer.vertex(maxKeyFrameId * 2 + 5);
            if (!p_previousPoseVertex || !p_previousVelocityVertex ||
                !p_gyroBiasVertex || !p_accelBiasVertex ||
                !p_currentPoseVertex || !p_currentVelocityVertex ||
                !p_gravityDirectionVertex || !p_scaleVertex)
            {
                cout << "Error" << p_previousPoseVertex << ", "
                     << p_previousVelocityVertex << ", " << p_gyroBiasVertex
                     << ", " << p_accelBiasVertex << ", " << p_currentPoseVertex
                     << ", " << p_currentVelocityVertex << ", "
                     << p_gravityDirectionVertex << ", " << p_scaleVertex
                     << endl;

                continue;
            }
            EdgeInertialGS *p_inertialEdge =
                new EdgeInertialGS(p_keyFrame->p_imuPreintegrated);
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
                    p_gyroBiasVertex));
            p_inertialEdge->setVertex(
                3,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                    p_accelBiasVertex));
            p_inertialEdge->setVertex(
                4,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                    p_currentPoseVertex));
            p_inertialEdge->setVertex(
                5,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                    p_currentVelocityVertex));
            p_inertialEdge->setVertex(
                6,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                    p_gravityDirectionVertex));
            p_inertialEdge->setVertex(
                7,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_scaleVertex));

            inertialEdges.push_back(p_inertialEdge);

            usedKeyFramePairs.push_back(
                make_pair(p_keyFrame->p_prevKF, p_keyFrame));
            optimizer.addEdge(p_inertialEdge);
        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(iterationCount);

    // Recover optimized data
    // Biases
    p_gyroBiasVertex =
        static_cast<VertexGyroBias *>(optimizer.vertex(maxKeyFrameId * 2 + 2));
    p_accelBiasVertex =
        static_cast<VertexAccBias *>(optimizer.vertex(maxKeyFrameId * 2 + 3));
    Vector6d biasVector;
    biasVector << p_gyroBiasVertex->estimate(), p_accelBiasVertex->estimate();
    gyroBias_out << p_gyroBiasVertex->estimate();
    accelBias_out << p_accelBiasVertex->estimate();

    IMU::Bias imuBias(biasVector[3],
                      biasVector[4],
                      biasVector[5],
                      biasVector[0],
                      biasVector[1],
                      biasVector[2]);

    // Keyframes velocities and biases
    const size_t keyFrameCount = keyFrames.size();
    for (size_t elementIndex = 0; elementIndex < keyFrameCount; elementIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[elementIndex];
        if (p_keyFrame->id > maxKeyFrameId)
            continue;

        VertexVelocity *p_velocityVertex = static_cast<VertexVelocity *>(
            optimizer.vertex(maxKeyFrameId + (p_keyFrame->id) + 1));
        Eigen::Vector3d keyFrameVelocity = p_velocityVertex->estimate();
        if (p_keyFrame->setVelocity(keyFrameVelocity.cast<float>()) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        Eigen::Vector3f keyFrameGyroBias{};
        if (p_keyFrame->getGyroBias(keyFrameGyroBias) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGyroBias returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if ((keyFrameGyroBias - gyroBias_out.cast<float>()).norm() > 0.01)
        {
            if (p_keyFrame->setNewBias(imuBias) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNewBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame->p_imuPreintegrated)
            {
                if (p_keyFrame->p_imuPreintegrated->reintegrate() !=
                    IMU::PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: reintegrate returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
        }
        else
        {
            if (p_keyFrame->setNewBias(imuBias) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNewBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
