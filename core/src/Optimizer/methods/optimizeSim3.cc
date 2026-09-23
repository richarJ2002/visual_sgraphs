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

namespace vs_graphs
{
namespace core
{

int Optimizer::optimizeSim3(KeyFrame                    *pKF1,
                            KeyFrame                    *pKF2,
                            vector<MapPoint *>          &vpMatches1,
                            g2o::Sim3                   &g2oS12,
                            const float                  th2,
                            const bool                   bFixScale,
                            Eigen::Matrix<double, 7, 7> &mAcumHessian,
                            const bool                   bAllPoints)
{
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver =
        new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Camera poses
    const Eigen::Matrix3f R1w = pKF1->getRotation();
    const Eigen::Vector3f t1w = pKF1->getTranslation();
    const Eigen::Matrix3f R2w = pKF2->getRotation();
    const Eigen::Vector3f t2w = pKF2->getTranslation();

    // Set Sim3 vertex
    vs_graphs::core::VertexSim3Expmap *vSim3 =
        new vs_graphs::core::VertexSim3Expmap();
    vSim3->_fix_scale = bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->pCamera1 = pKF1->p_camera;
    vSim3->pCamera2 = pKF2->p_camera;
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int                N            = vpMatches1.size();
    const vector<MapPoint *> vpMapPoints1 = pKF1->getMapPointMatches();
    vector<vs_graphs::core::EdgeSim3ProjectXYZ *>        vpEdges12;
    vector<vs_graphs::core::EdgeInverseSim3ProjectXYZ *> vpEdges21;
    vector<size_t>                                       vnIndexEdge;
    vector<bool>                                         vbIsInKF2;

    vnIndexEdge.reserve(2 * N);
    vpEdges12.reserve(2 * N);
    vpEdges21.reserve(2 * N);
    vbIsInKF2.reserve(2 * N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;
    int nBadMPs          = 0;
    int nInKF2           = 0;
    int nOutKF2          = 0;
    int nMatchWithoutMP  = 0;

    vector<int> vIdsOnlyInKF2;

    for (int i = 0; i < N; i++)
    {
        if (!vpMatches1[i])
            continue;

        MapPoint *pMP1 = vpMapPoints1[i];
        MapPoint *pMP2 = vpMatches1[i];

        const int id1 = 2 * i + 1;
        const int id2 = 2 * (i + 1);

        const int i2 = get<0>(pMP2->getIndexInKeyFrame(pKF2));

        Eigen::Vector3f P3D1c;
        Eigen::Vector3f P3D2c;

        if (pMP1 && pMP2)
        {
            if (!pMP1->isBad() && !pMP2->isBad())
            {
                g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f         P3D1w   = pMP1->getWorldPos();
                P3D1c                           = R1w * P3D1w + t1w;
                vPoint1->setEstimate(P3D1c.cast<double>());
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f         P3D2w   = pMP2->getWorldPos();
                P3D2c                           = R2w * P3D2w + t2w;
                vPoint2->setEstimate(P3D2c.cast<double>());
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
            {
                nBadMPs++;
                continue;
            }
        }
        else
        {
            nMatchWithoutMP++;

            // TODO The 3D position in KF1 doesn't exist
            if (!pMP2->isBad())
            {
                g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f         P3D2w   = pMP2->getWorldPos();
                P3D2c                           = R2w * P3D2w + t2w;
                vPoint2->setEstimate(P3D2c.cast<double>());
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);

                vIdsOnlyInKF2.push_back(id2);
            }
            continue;
        }

        if (i2 < 0 && !bAllPoints)
        {
            Verbose::printMess("    Remove point -> i2: " + to_string(i2) +
                                   "; bAllPoints: " + to_string(bAllPoints),
                               Verbose::VERBOSITY_DEBUG);
            continue;
        }

        if (P3D2c(2) < 0)
        {
            Verbose::printMess("Sim3: Z coordinate is negative",
                               Verbose::VERBOSITY_DEBUG);
            continue;
        }

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double, 2, 1> obs1;
        const cv::KeyPoint         &kpUn1 = pKF1->keyPointsUndistorted[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        vs_graphs::core::EdgeSim3ProjectXYZ *e12 =
            new vs_graphs::core::EdgeSim3ProjectXYZ();

        e12->setVertex(0,
                       dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                           optimizer.vertex(id2)));
        e12->setVertex(
            1,
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->invLevelSigmaSquared[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

        g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double, 2, 1> obs2;
        cv::KeyPoint                kpUn2;
        bool                        inKF2;
        if (i2 >= 0)
        {
            kpUn2 = pKF2->keyPointsUndistorted[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;
            inKF2 = true;

            nInKF2++;
        }
        else
        {
            const double zc2 = P3D2c(2);
            float invz = (zc2 == 0.0) ? 0.0f : static_cast<float>(1.0 / zc2);
            float x    = P3D2c(0) * invz;
            float y    = P3D2c(1) * invz;

            obs2 << x, y;
            kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->trackScaleLevel);

            inKF2 = false;
            nOutKF2++;
        }

        vs_graphs::core::EdgeInverseSim3ProjectXYZ *e21 =
            new vs_graphs::core::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0,
                       dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                           optimizer.vertex(id1)));
        e21->setVertex(
            1,
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->invLevelSigmaSquared[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

        g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);

        vbIsInKF2.push_back(inKF2);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad       = 0;
    int nBadOutKF2 = 0;
    for (size_t i = 0; i < vpEdges12.size(); i++)
    {
        vs_graphs::core::EdgeSim3ProjectXYZ        *e12 = vpEdges12[i];
        vs_graphs::core::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
        if (!e12 || !e21)
            continue;

        if (e12->chi2() > th2 || e21->chi2() > th2)
        {
            size_t idx      = vnIndexEdge[i];
            vpMatches1[idx] = static_cast<MapPoint *>(nullptr);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i] =
                static_cast<vs_graphs::core::EdgeSim3ProjectXYZ *>(nullptr);
            vpEdges21[i] =
                static_cast<vs_graphs::core::EdgeInverseSim3ProjectXYZ *>(
                    nullptr);
            nBad++;

            if (!vbIsInKF2[i])
            {
                nBadOutKF2++;
            }
            continue;
        }

        // Check if remove the robust adjustment improve the result
        e12->setRobustKernel(0);
        e21->setRobustKernel(0);
    }

    int nMoreIterations;
    if (nBad > 0)
        nMoreIterations = 10;
    else
        nMoreIterations = 5;

    if (nCorrespondences - nBad < 10)
        return 0;

    // Optimize again only with inliers
    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn      = 0;
    mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
    for (size_t i = 0; i < vpEdges12.size(); i++)
    {
        vs_graphs::core::EdgeSim3ProjectXYZ        *e12 = vpEdges12[i];
        vs_graphs::core::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
        if (!e12 || !e21)
            continue;

        e12->computeError();
        e21->computeError();

        if (e12->chi2() > th2 || e21->chi2() > th2)
        {
            size_t idx      = vnIndexEdge[i];
            vpMatches1[idx] = static_cast<MapPoint *>(nullptr);
        }
        else
        {
            nIn++;
        }
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap *vSim3_recov =
        static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
    g2oS12 = vSim3_recov->estimate();

    return nIn;
}

} // namespace core
} // namespace vs_graphs
