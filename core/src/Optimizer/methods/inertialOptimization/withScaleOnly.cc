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

namespace vs_graphs
{
namespace core
{

void Optimizer::inertialOptimization(Map             *pMap,
                                     Eigen::Matrix3d &Rwg,
                                     double          &scale)
{
    int                      its     = 10;
    long unsigned int        maxKFid = pMap->getMaxKeyFrameId();
    const vector<KeyFrame *> vpKFs   = pMap->getAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton *solver =
        new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (all variables are fixed)
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;
        VertexPose *VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity *VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid + 1 + (pKFi->mnId));
        VV->setFixed(true);
        optimizer.addVertex(VV);

        // Vertex of fixed biases
        VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
        VG->setId(2 * (maxKFid + 1) + (pKFi->mnId));
        VG->setFixed(true);
        optimizer.addVertex(VG);
        VertexAccBias *VA = new VertexAccBias(vpKFs.front());
        VA->setId(3 * (maxKFid + 1) + (pKFi->mnId));
        VA->setFixed(true);
        optimizer.addVertex(VA);
    }

    // Gravity and scale
    VertexGDir *VGDir = new VertexGDir(Rwg);
    VGDir->setId(4 * (maxKFid + 1));
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);
    VertexScale *VS = new VertexScale(scale);
    VS->setId(4 * (maxKFid + 1) + 1);
    VS->setFixed(false);
    optimizer.addVertex(VS);

    // Graph edges
    int count_edges = 0;
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKFi = vpKFs[i];

        if (pKFi->p_prevKF && pKFi->mnId <= maxKFid)
        {
            if (pKFi->isBad() || pKFi->p_prevKF->mnId > maxKFid)
                continue;

            g2o::HyperGraph::Vertex *VP1 =
                optimizer.vertex(pKFi->p_prevKF->mnId);
            g2o::HyperGraph::Vertex *VV1 =
                optimizer.vertex((maxKFid + 1) + pKFi->p_prevKF->mnId);
            g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex *VV2 =
                optimizer.vertex((maxKFid + 1) + pKFi->mnId);
            g2o::HyperGraph::Vertex *VG =
                optimizer.vertex(2 * (maxKFid + 1) + pKFi->p_prevKF->mnId);
            g2o::HyperGraph::Vertex *VA =
                optimizer.vertex(3 * (maxKFid + 1) + pKFi->p_prevKF->mnId);
            g2o::HyperGraph::Vertex *VGDir =
                optimizer.vertex(4 * (maxKFid + 1));
            g2o::HyperGraph::Vertex *VS =
                optimizer.vertex(4 * (maxKFid + 1) + 1);
            if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                Verbose::printMess(
                    "Error" + to_string(VP1->id()) + ", " +
                        to_string(VV1->id()) + ", " + to_string(VG->id()) +
                        ", " + to_string(VA->id()) + ", " +
                        to_string(VP2->id()) + ", " + to_string(VV2->id()) +
                        ", " + to_string(VGDir->id()) + ", " +
                        to_string(VS->id()),
                    Verbose::VERBOSITY_NORMAL);

                continue;
            }
            count_edges++;
            EdgeInertialGS *ei = new EdgeInertialGS(pKFi->p_imuPreintegrated);
            ei->setVertex(0,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
            ei->setVertex(1,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
            ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
            ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
            ei->setVertex(4,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
            ei->setVertex(5,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
            ei->setVertex(6,
                          dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDir));
            ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VS));
            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            ei->setRobustKernel(rk);
            rk->setDelta(1.f);
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
    scale = VS->estimate();
    Rwg   = VGDir->estimate().Rwg;
}

} // namespace core
} // namespace vs_graphs
