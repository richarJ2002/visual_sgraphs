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

OptimizerStatus Optimizer::inertialOptimization(Map             *p_map_in,
                                                Eigen::Matrix3d &Rwg_inout,
                                                double          &scale_inout)
{
    int           its = 10;
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

    g2o::OptimizationAlgorithmGaussNewton *p_solver =
        new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(p_solver);

    // Set KeyFrame vertices (all variables are fixed)
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
        p_velocityVertex->setId(maximumKeyFrameId + 1 + (p_keyFrame->id));
        p_velocityVertex->setFixed(true);
        optimizer.addVertex(p_velocityVertex);

        // Vertex of fixed biases
        VertexGyroBias *p_gyroBiasVertex =
            new VertexGyroBias(keyFrames.front());
        p_gyroBiasVertex->setId(2 * (maximumKeyFrameId + 1) + (p_keyFrame->id));
        p_gyroBiasVertex->setFixed(true);
        optimizer.addVertex(p_gyroBiasVertex);
        VertexAccBias *p_accelerometerBiasVertex =
            new VertexAccBias(keyFrames.front());
        p_accelerometerBiasVertex->setId(3 * (maximumKeyFrameId + 1) +
                                         (p_keyFrame->id));
        p_accelerometerBiasVertex->setFixed(true);
        optimizer.addVertex(p_accelerometerBiasVertex);
    }

    // Gravity and scale
    VertexGDir *p_gravityDirectionVertex = new VertexGDir(Rwg_inout);
    p_gravityDirectionVertex->setId(4 * (maximumKeyFrameId + 1));
    p_gravityDirectionVertex->setFixed(false);
    optimizer.addVertex(p_gravityDirectionVertex);
    VertexScale *p_scaleVertex = new VertexScale(scale_inout);
    p_scaleVertex->setId(4 * (maximumKeyFrameId + 1) + 1);
    p_scaleVertex->setFixed(false);
    optimizer.addVertex(p_scaleVertex);

    // Graph edges
    int edgeCount = 0;
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

            g2o::HyperGraph::Vertex *p_firstPoseVertex =
                optimizer.vertex(p_keyFrame->p_prevKF->id);
            g2o::HyperGraph::Vertex *p_firstVelocityVertex = optimizer.vertex(
                (maximumKeyFrameId + 1) + p_keyFrame->p_prevKF->id);
            g2o::HyperGraph::Vertex *p_secondPoseVertex =
                optimizer.vertex(p_keyFrame->id);
            g2o::HyperGraph::Vertex *p_secondVelocityVertex =
                optimizer.vertex((maximumKeyFrameId + 1) + p_keyFrame->id);
            g2o::HyperGraph::Vertex *p_gyroBiasVertex = optimizer.vertex(
                2 * (maximumKeyFrameId + 1) + p_keyFrame->p_prevKF->id);
            g2o::HyperGraph::Vertex *p_accelerometerBiasVertex =
                optimizer.vertex(3 * (maximumKeyFrameId + 1) +
                                 p_keyFrame->p_prevKF->id);
            g2o::HyperGraph::Vertex *p_gravityDirectionVertex =
                optimizer.vertex(4 * (maximumKeyFrameId + 1));
            g2o::HyperGraph::Vertex *p_scaleVertex =
                optimizer.vertex(4 * (maximumKeyFrameId + 1) + 1);
            if (!p_firstPoseVertex || !p_firstVelocityVertex ||
                !p_gyroBiasVertex || !p_accelerometerBiasVertex ||
                !p_secondPoseVertex || !p_secondVelocityVertex ||
                !p_gravityDirectionVertex || !p_scaleVertex)
            {
                if (Verbose::printMess(
                        "Error" + std::to_string(p_firstPoseVertex->id()) +
                            ", " + std::to_string(p_firstVelocityVertex->id()) +
                            ", " + std::to_string(p_gyroBiasVertex->id()) +
                            ", " +
                            std::to_string(p_accelerometerBiasVertex->id()) +
                            ", " + std::to_string(p_secondPoseVertex->id()) +
                            ", " +
                            std::to_string(p_secondVelocityVertex->id()) +
                            ", " +
                            std::to_string(p_gravityDirectionVertex->id()) +
                            ", " + std::to_string(p_scaleVertex->id()),
                        Verbose::VERBOSITY_NORMAL) !=
                    VerboseStatus::VERBOSE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: printMess returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                continue;
            }
            edgeCount++;
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
            g2o::RobustKernelHuber *p_robustKernel = new g2o::RobustKernelHuber;
            ei->setRobustKernel(p_robustKernel);
            p_robustKernel->setDelta(1.f);
            optimizer.addEdge(ei);
        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    /* Kept as bare calls: the chi2 readings had no consumer, but
     * computeActiveErrors() updates the edge error state. */
    optimizer.activeRobustChi2();
    optimizer.optimize(its);
    optimizer.computeActiveErrors();
    optimizer.activeRobustChi2();
    // Recover optimized data
    scale_inout = p_scaleVertex->estimate();
    Rwg_inout   = p_gravityDirectionVertex->estimate().Rwg;

    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
