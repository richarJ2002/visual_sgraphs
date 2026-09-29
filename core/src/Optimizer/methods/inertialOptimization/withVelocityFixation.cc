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
#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void Optimizer::inertialOptimization(
    Map                              *p_map_in,
    Eigen::Matrix3d                  &Rwg_inout,
    double                           &scale_inout,
    Eigen::Vector3d                  &bg_in,
    Eigen::Vector3d                  &ba_in,
    bool                              isMono_in,
    [[maybe_unused]] Eigen::MatrixXd &covInertial_in,
    bool                              isFixedVelocity_in,
    [[maybe_unused]] bool             shouldUseGaussNewton_in,
    float                             priorG_in,
    float                             priorA_in)
{
    Verbose::printMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
    int           its = 200;
    unsigned long maximumKeyFrameIdValue{};
    if (p_map_in->getMaxKeyFrameId(maximumKeyFrameIdValue) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMaxKeyFrameId returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    long unsigned int maximumKeyFrameId =
        static_cast<long unsigned int>(maximumKeyFrameIdValue);
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

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(p_linearSolver);

    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    if (priorG_in != 0.f)
        p_solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(p_solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
         keyFrameIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];
        if (p_keyFrame->id > maximumKeyFrameId)
            continue;
        VertexPose *p_poseVertex = new VertexPose(p_keyFrame);
        p_poseVertex->setId(p_keyFrame->id);
        p_poseVertex->setFixed(true);
        optimizer.addVertex(p_poseVertex);

        VertexVelocity *p_velocityVertex = new VertexVelocity(p_keyFrame);
        p_velocityVertex->setId(maximumKeyFrameId + (p_keyFrame->id) + 1);
        if (isFixedVelocity_in)
            p_velocityVertex->setFixed(true);
        else
            p_velocityVertex->setFixed(false);

        optimizer.addVertex(p_velocityVertex);
    }

    // Biases
    VertexGyroBias *p_gyroBiasVertex = new VertexGyroBias(keyFrames.front());
    p_gyroBiasVertex->setId(maximumKeyFrameId * 2 + 2);
    if (isFixedVelocity_in)
        p_gyroBiasVertex->setFixed(true);
    else
        p_gyroBiasVertex->setFixed(false);
    optimizer.addVertex(p_gyroBiasVertex);
    VertexAccBias *p_accelerometerBiasVertex =
        new VertexAccBias(keyFrames.front());
    p_accelerometerBiasVertex->setId(maximumKeyFrameId * 2 + 3);
    if (isFixedVelocity_in)
        p_accelerometerBiasVertex->setFixed(true);
    else
        p_accelerometerBiasVertex->setFixed(false);

    optimizer.addVertex(p_accelerometerBiasVertex);
    // prior acc bias
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc *p_epa = new EdgePriorAcc(bprior);
    p_epa->setVertex(0,
                     dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                         p_accelerometerBiasVertex));
    double informationPriorA = priorA_in;
    p_epa->setInformation(informationPriorA * Eigen::Matrix3d::Identity());
    optimizer.addEdge(p_epa);
    EdgePriorGyro *p_epg = new EdgePriorGyro(bprior);
    p_epg->setVertex(
        0,
        dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_gyroBiasVertex));
    double informationPriorG = priorG_in;
    p_epg->setInformation(informationPriorG * Eigen::Matrix3d::Identity());
    optimizer.addEdge(p_epg);

    // Gravity and scale
    VertexGDir *p_gravityDirectionVertex = new VertexGDir(Rwg_inout);
    p_gravityDirectionVertex->setId(maximumKeyFrameId * 2 + 4);
    p_gravityDirectionVertex->setFixed(false);
    optimizer.addVertex(p_gravityDirectionVertex);
    VertexScale *p_scaleVertex = new VertexScale(scale_inout);
    p_scaleVertex->setId(maximumKeyFrameId * 2 + 5);
    p_scaleVertex->setFixed(!isMono_in); // Fixed for stereo case
    optimizer.addVertex(p_scaleVertex);

    // Graph edges
    // IMU links with gravity and scale
    vector<EdgeInertialGS *> vpei;
    vpei.reserve(keyFrames.size());
    vector<pair<KeyFrame *, KeyFrame *>> vppUsedKeyFrame;
    vppUsedKeyFrame.reserve(keyFrames.size());
    // std::cout << "build optimization graph" << std::endl;

    for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
         keyFrameIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];

        if (p_keyFrame->p_prevKF && p_keyFrame->id <= maximumKeyFrameId)
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
            if (keyFrameIsBad || p_keyFrame->p_prevKF->id > maximumKeyFrameId)
                continue;
            if (!p_keyFrame->p_imuPreintegrated)
                std::cout << "Not preintegrated measurement" << std::endl;

            IMU::Bias imuBias{};
            if (p_keyFrame->p_prevKF->getImuBias(imuBias) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getImuBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            p_keyFrame->p_imuPreintegrated->setNewBias(imuBias);
            g2o::HyperGraph::Vertex *p_firstPoseVertex =
                optimizer.vertex(p_keyFrame->p_prevKF->id);
            g2o::HyperGraph::Vertex *p_firstVelocityVertex = optimizer.vertex(
                maximumKeyFrameId + (p_keyFrame->p_prevKF->id) + 1);
            g2o::HyperGraph::Vertex *p_secondPoseVertex =
                optimizer.vertex(p_keyFrame->id);
            g2o::HyperGraph::Vertex *p_secondVelocityVertex =
                optimizer.vertex(maximumKeyFrameId + (p_keyFrame->id) + 1);
            g2o::HyperGraph::Vertex *p_gyroBiasVertex =
                optimizer.vertex(maximumKeyFrameId * 2 + 2);
            g2o::HyperGraph::Vertex *p_accelerometerBiasVertex =
                optimizer.vertex(maximumKeyFrameId * 2 + 3);
            g2o::HyperGraph::Vertex *p_gravityDirectionVertex =
                optimizer.vertex(maximumKeyFrameId * 2 + 4);
            g2o::HyperGraph::Vertex *p_scaleVertex =
                optimizer.vertex(maximumKeyFrameId * 2 + 5);
            if (!p_firstPoseVertex || !p_firstVelocityVertex ||
                !p_gyroBiasVertex || !p_accelerometerBiasVertex ||
                !p_secondPoseVertex || !p_secondVelocityVertex ||
                !p_gravityDirectionVertex || !p_scaleVertex)
            {
                cout << "Error" << p_firstPoseVertex << ", "
                     << p_firstVelocityVertex << ", " << p_gyroBiasVertex
                     << ", " << p_accelerometerBiasVertex << ", "
                     << p_secondPoseVertex << ", " << p_secondVelocityVertex
                     << ", " << p_gravityDirectionVertex << ", "
                     << p_scaleVertex << endl;

                continue;
            }
            EdgeInertialGS *ei =
                new EdgeInertialGS(p_keyFrame->p_imuPreintegrated);
            ei->setVertex(0,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              p_firstPoseVertex));
            ei->setVertex(1,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              p_firstVelocityVertex));
            ei->setVertex(2,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              p_gyroBiasVertex));
            ei->setVertex(3,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              p_accelerometerBiasVertex));
            ei->setVertex(4,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              p_secondPoseVertex));
            ei->setVertex(5,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              p_secondVelocityVertex));
            ei->setVertex(6,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                              p_gravityDirectionVertex));
            ei->setVertex(
                7,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_scaleVertex));

            vpei.push_back(ei);

            vppUsedKeyFrame.push_back(
                make_pair(p_keyFrame->p_prevKF, p_keyFrame));
            optimizer.addEdge(ei);
        }
    }

    // Compute error for different scales
    std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);

    scale_inout = p_scaleVertex->estimate();

    // Recover optimized data
    // Biases
    p_gyroBiasVertex = static_cast<VertexGyroBias *>(
        optimizer.vertex(maximumKeyFrameId * 2 + 2));
    p_accelerometerBiasVertex = static_cast<VertexAccBias *>(
        optimizer.vertex(maximumKeyFrameId * 2 + 3));
    Vector6d vb;
    vb << p_gyroBiasVertex->estimate(), p_accelerometerBiasVertex->estimate();
    bg_in << p_gyroBiasVertex->estimate();
    ba_in << p_accelerometerBiasVertex->estimate();
    scale_inout = p_scaleVertex->estimate();

    IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
    Rwg_inout = p_gravityDirectionVertex->estimate().Rwg;

    // Keyframes velocities and biases
    const size_t N = keyFrames.size();
    for (size_t keyFrameIndex = 0; keyFrameIndex < N; keyFrameIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];
        if (p_keyFrame->id > maximumKeyFrameId)
            continue;

        VertexVelocity *p_velocityVertex = static_cast<VertexVelocity *>(
            optimizer.vertex(maximumKeyFrameId + (p_keyFrame->id) + 1));
        Eigen::Vector3d Vw =
            p_velocityVertex->estimate(); // Velocity is scaled after
        if (p_keyFrame->setVelocity(Vw.cast<float>()) !=
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
        if ((keyFrameGyroBias - bg_in.cast<float>()).norm() > 0.01)
        {
            if (p_keyFrame->setNewBias(b) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNewBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrame->p_imuPreintegrated)
            {
                p_keyFrame->p_imuPreintegrated->reintegrate();
            }
        }
        else
        {
            if (p_keyFrame->setNewBias(b) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNewBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }
}

} // namespace core
} // namespace vs_graphs
