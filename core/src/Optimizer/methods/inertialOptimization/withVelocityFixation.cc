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

void Optimizer::inertialOptimization(
    Map                              *pMap,
    Eigen::Matrix3d                  &Rwg,
    double                           &scale,
    Eigen::Vector3d                  &bg,
    Eigen::Vector3d                  &ba,
    bool                              bMono,
    [[maybe_unused]] Eigen::MatrixXd &covInertial,
    bool                              bFixedVel,
    [[maybe_unused]] bool             bGauss,
    float                             priorG,
    float                             priorA)
{
    Verbose::printMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
    int                      its     = 200;
    long unsigned int        maxKFid = pMap->getMaxKeyFrameId();
    const vector<KeyFrame *> vpKFs   = pMap->getAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    if (priorG != 0.f)
        solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
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
        VV->setId(maxKFid + (pKFi->mnId) + 1);
        if (bFixedVel)
            VV->setFixed(true);
        else
            VV->setFixed(false);

        optimizer.addVertex(VV);
    }

    // Biases
    VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid * 2 + 2);
    if (bFixedVel)
        VG->setFixed(true);
    else
        VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias *VA = new VertexAccBias(vpKFs.front());
    VA->setId(maxKFid * 2 + 3);
    if (bFixedVel)
        VA->setFixed(true);
    else
        VA->setFixed(false);

    optimizer.addVertex(VA);
    // prior acc bias
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc *epa = new EdgePriorAcc(bprior);
    epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
    double infoPriorA = priorA;
    epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);
    EdgePriorGyro *epg = new EdgePriorGyro(bprior);
    epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
    double infoPriorG = priorG;
    epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);

    // Gravity and scale
    VertexGDir *VGDir = new VertexGDir(Rwg);
    VGDir->setId(maxKFid * 2 + 4);
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);
    VertexScale *VS = new VertexScale(scale);
    VS->setId(maxKFid * 2 + 5);
    VS->setFixed(!bMono); // Fixed for stereo case
    optimizer.addVertex(VS);

    // Graph edges
    // IMU links with gravity and scale
    vector<EdgeInertialGS *> vpei;
    vpei.reserve(vpKFs.size());
    vector<pair<KeyFrame *, KeyFrame *>> vppUsedKF;
    vppUsedKF.reserve(vpKFs.size());
    // std::cout << "build optimization graph" << std::endl;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKFi = vpKFs[i];

        if (pKFi->p_prevKF && pKFi->mnId <= maxKFid)
        {
            if (pKFi->isBad() || pKFi->p_prevKF->mnId > maxKFid)
                continue;
            if (!pKFi->p_imuPreintegrated)
                std::cout << "Not preintegrated measurement" << std::endl;

            pKFi->p_imuPreintegrated->setNewBias(pKFi->p_prevKF->getImuBias());
            g2o::HyperGraph::Vertex *VP1 =
                optimizer.vertex(pKFi->p_prevKF->mnId);
            g2o::HyperGraph::Vertex *VV1 =
                optimizer.vertex(maxKFid + (pKFi->p_prevKF->mnId) + 1);
            g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex *VV2 =
                optimizer.vertex(maxKFid + (pKFi->mnId) + 1);
            g2o::HyperGraph::Vertex *VG    = optimizer.vertex(maxKFid * 2 + 2);
            g2o::HyperGraph::Vertex *VA    = optimizer.vertex(maxKFid * 2 + 3);
            g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(maxKFid * 2 + 4);
            g2o::HyperGraph::Vertex *VS    = optimizer.vertex(maxKFid * 2 + 5);
            if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                cout << "Error" << VP1 << ", " << VV1 << ", " << VG << ", "
                     << VA << ", " << VP2 << ", " << VV2 << ", " << VGDir
                     << ", " << VS << endl;

                continue;
            }
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

            vpei.push_back(ei);

            vppUsedKF.push_back(make_pair(pKFi->p_prevKF, pKFi));
            optimizer.addEdge(ei);
        }
    }

    // Compute error for different scales
    std::set<g2o::HyperGraph::Edge *> setEdges = optimizer.edges();

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);

    scale = VS->estimate();

    // Recover optimized data
    // Biases
    VG = static_cast<VertexGyroBias *>(optimizer.vertex(maxKFid * 2 + 2));
    VA = static_cast<VertexAccBias *>(optimizer.vertex(maxKFid * 2 + 3));
    Vector6d vb;
    vb << VG->estimate(), VA->estimate();
    bg << VG->estimate();
    ba << VA->estimate();
    scale = VS->estimate();

    IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
    Rwg = VGDir->estimate().Rwg;

    // Keyframes velocities and biases
    const size_t N = vpKFs.size();
    for (size_t i = 0; i < N; i++)
    {
        KeyFrame *pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
            continue;

        VertexVelocity *VV = static_cast<VertexVelocity *>(
            optimizer.vertex(maxKFid + (pKFi->mnId) + 1));
        Eigen::Vector3d Vw = VV->estimate(); // Velocity is scaled after
        pKFi->setVelocity(Vw.cast<float>());

        if ((pKFi->getGyroBias() - bg.cast<float>()).norm() > 0.01)
        {
            pKFi->setNewBias(b);
            if (pKFi->p_imuPreintegrated)
                pKFi->p_imuPreintegrated->reintegrate();
        }
        else
            pKFi->setNewBias(b);
    }
}

} // namespace core
} // namespace vs_graphs
