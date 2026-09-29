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
#include "System.h"

namespace vs_graphs
{
namespace core
{

int Optimizer::optimizeSim3(KeyFrame                    *p_keyFrame1_in,
                            KeyFrame                    *p_keyFrame2_in,
                            vector<MapPoint *>          &matches1_inout,
                            g2o::Sim3                   &g2oS12_inout,
                            const float                  threshold2_in,
                            const bool                   isScaleFixed_in,
                            Eigen::Matrix<double, 7, 7> &acumHessian_out,
                            const bool                   shouldUseAllPoints_in)
{
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver;

    p_linearSolver =
        new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(p_linearSolver);

    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(p_solver);

    // Camera poses
    const Eigen::Matrix3f R1w = p_keyFrame1_in->getRotation();
    const Eigen::Vector3f t1w = p_keyFrame1_in->getTranslation();
    const Eigen::Matrix3f R2w = p_keyFrame2_in->getRotation();
    const Eigen::Vector3f t2w = p_keyFrame2_in->getTranslation();

    // Set Sim3 vertex
    vs_graphs::core::VertexSim3Expmap *vSim3 =
        new vs_graphs::core::VertexSim3Expmap();
    vSim3->isScaleFixed = isScaleFixed_in;
    vSim3->setEstimate(g2oS12_inout);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->p_firstCamera  = p_keyFrame1_in->p_camera;
    vSim3->p_secondCamera = p_keyFrame2_in->p_camera;
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int                N          = matches1_inout.size();
    const vector<MapPoint *> mapPoints1 = p_keyFrame1_in->getMapPointMatches();
    vector<vs_graphs::core::EdgeSim3ProjectXYZ *>        edges12;
    vector<vs_graphs::core::EdgeInverseSim3ProjectXYZ *> edges21;
    vector<size_t>                                       edgeIndices;
    vector<bool>                                         isInKeyFrame2Flags;

    edgeIndices.reserve(2 * N);
    edges12.reserve(2 * N);
    edges21.reserve(2 * N);
    isInKeyFrame2Flags.reserve(2 * N);

    const float deltaHuber = sqrt(threshold2_in);

    int correspondenceCount       = 0;
    int badMapPointCount          = 0;
    int inKeyFrame2Count          = 0;
    int outKeyFrame2Count         = 0;
    int matchWithoutMapPointCount = 0;

    vector<int> idsOnlyInKeyFrame2;

    for (int edges12Index = 0; edges12Index < N; edges12Index++)
    {
        if (!matches1_inout[edges12Index])
            continue;

        MapPoint *p_mapPoint1 = mapPoints1[edges12Index];
        MapPoint *p_mapPoint2 = matches1_inout[edges12Index];

        const int id1 = 2 * edges12Index + 1;
        const int id2 = 2 * (edges12Index + 1);

        const int i2 = get<0>(p_mapPoint2->getIndexInKeyFrame(p_keyFrame2_in));

        Eigen::Vector3f P3D1c;
        Eigen::Vector3f P3D2c;

        if (p_mapPoint1 && p_mapPoint2)
        {
            if (!p_mapPoint1->isBad() && !p_mapPoint2->isBad())
            {
                g2o::VertexSBAPointXYZ *p_point1Vertex =
                    new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f P3D1w = p_mapPoint1->getWorldPos();
                P3D1c                 = R1w * P3D1w + t1w;
                p_point1Vertex->setEstimate(P3D1c.cast<double>());
                p_point1Vertex->setId(id1);
                p_point1Vertex->setFixed(true);
                optimizer.addVertex(p_point1Vertex);

                g2o::VertexSBAPointXYZ *p_point2Vertex =
                    new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f P3D2w = p_mapPoint2->getWorldPos();
                P3D2c                 = R2w * P3D2w + t2w;
                p_point2Vertex->setEstimate(P3D2c.cast<double>());
                p_point2Vertex->setId(id2);
                p_point2Vertex->setFixed(true);
                optimizer.addVertex(p_point2Vertex);
            }
            else
            {
                badMapPointCount++;
                continue;
            }
        }
        else
        {
            matchWithoutMapPointCount++;

            // TODO The 3D position in KF1 doesn't exist
            if (!p_mapPoint2->isBad())
            {
                g2o::VertexSBAPointXYZ *p_point2Vertex =
                    new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f P3D2w = p_mapPoint2->getWorldPos();
                P3D2c                 = R2w * P3D2w + t2w;
                p_point2Vertex->setEstimate(P3D2c.cast<double>());
                p_point2Vertex->setId(id2);
                p_point2Vertex->setFixed(true);
                optimizer.addVertex(p_point2Vertex);

                idsOnlyInKeyFrame2.push_back(id2);
            }
            continue;
        }

        if (i2 < 0 && !shouldUseAllPoints_in)
        {
            Verbose::printMess(
                "    Remove point -> i2: " + to_string(i2) +
                    "; bAllPoints: " + to_string(shouldUseAllPoints_in),
                Verbose::VERBOSITY_DEBUG);
            continue;
        }

        if (P3D2c(2) < 0)
        {
            Verbose::printMess("Sim3: Z coordinate is negative",
                               Verbose::VERBOSITY_DEBUG);
            continue;
        }

        correspondenceCount++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double, 2, 1> observation1;
        const cv::KeyPoint         &undistortedKeyPoint1 =
            p_keyFrame1_in->keyPointsUndistorted[edges12Index];
        observation1 << undistortedKeyPoint1.pt.x, undistortedKeyPoint1.pt.y;

        vs_graphs::core::EdgeSim3ProjectXYZ *e12 =
            new vs_graphs::core::EdgeSim3ProjectXYZ();

        e12->setVertex(0,
                       dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                           optimizer.vertex(id2)));
        e12->setVertex(
            1,
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        e12->setMeasurement(observation1);
        const float &invSigmaSquare1 =
            p_keyFrame1_in->invLevelSigmaSquared[undistortedKeyPoint1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

        g2o::RobustKernelHuber *p_robustKernel1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(p_robustKernel1);
        p_robustKernel1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double, 2, 1> observation2;
        cv::KeyPoint                undistortedKeyPoint2;
        bool                        inKeyFrame2;
        if (i2 >= 0)
        {
            undistortedKeyPoint2 = p_keyFrame2_in->keyPointsUndistorted[i2];
            observation2 << undistortedKeyPoint2.pt.x,
                undistortedKeyPoint2.pt.y;
            inKeyFrame2 = true;

            inKeyFrame2Count++;
        }
        else
        {
            const double zc2 = P3D2c(2);
            float invz = (zc2 == 0.0) ? 0.0f : static_cast<float>(1.0 / zc2);
            float x    = P3D2c(0) * invz;
            float y    = P3D2c(1) * invz;

            observation2 << x, y;
            undistortedKeyPoint2 =
                cv::KeyPoint(cv::Point2f(x, y), p_mapPoint2->trackScaleLevel);

            inKeyFrame2 = false;
            outKeyFrame2Count++;
        }

        vs_graphs::core::EdgeInverseSim3ProjectXYZ *e21 =
            new vs_graphs::core::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0,
                       dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                           optimizer.vertex(id1)));
        e21->setVertex(
            1,
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        e21->setMeasurement(observation2);
        float invSigmaSquare2 =
            p_keyFrame2_in->invLevelSigmaSquared[undistortedKeyPoint2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

        g2o::RobustKernelHuber *p_robustKernel2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(p_robustKernel2);
        p_robustKernel2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        edges12.push_back(e12);
        edges21.push_back(e21);
        edgeIndices.push_back(edges12Index);

        isInKeyFrame2Flags.push_back(inKeyFrame2);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int badCount             = 0;
    int badOutKeyFrame2Count = 0;
    for (size_t edges12Index = 0; edges12Index < edges12.size(); edges12Index++)
    {
        vs_graphs::core::EdgeSim3ProjectXYZ        *e12 = edges12[edges12Index];
        vs_graphs::core::EdgeInverseSim3ProjectXYZ *e21 = edges21[edges12Index];
        if (!e12 || !e21)
            continue;

        if (e12->chi2() > threshold2_in || e21->chi2() > threshold2_in)
        {
            size_t edgeIndex          = edgeIndices[edges12Index];
            matches1_inout[edgeIndex] = static_cast<MapPoint *>(nullptr);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            edges12[edges12Index] =
                static_cast<vs_graphs::core::EdgeSim3ProjectXYZ *>(nullptr);
            edges21[edges12Index] =
                static_cast<vs_graphs::core::EdgeInverseSim3ProjectXYZ *>(
                    nullptr);
            badCount++;

            if (!isInKeyFrame2Flags[edges12Index])
            {
                badOutKeyFrame2Count++;
            }
            continue;
        }

        // Check if remove the robust adjustment improve the result
        e12->setRobustKernel(0);
        e21->setRobustKernel(0);
    }

    int moreIterationCount;
    if (badCount > 0)
        moreIterationCount = 10;
    else
        moreIterationCount = 5;

    if (correspondenceCount - badCount < 10)
        return 0;

    // Optimize again only with inliers
    optimizer.initializeOptimization();
    optimizer.optimize(moreIterationCount);

    int inCount     = 0;
    acumHessian_out = Eigen::MatrixXd::Zero(7, 7);
    for (size_t edges12Index = 0; edges12Index < edges12.size(); edges12Index++)
    {
        vs_graphs::core::EdgeSim3ProjectXYZ        *e12 = edges12[edges12Index];
        vs_graphs::core::EdgeInverseSim3ProjectXYZ *e21 = edges21[edges12Index];
        if (!e12 || !e21)
            continue;

        e12->computeError();
        e21->computeError();

        if (e12->chi2() > threshold2_in || e21->chi2() > threshold2_in)
        {
            size_t edgeIndex          = edgeIndices[edges12Index];
            matches1_inout[edgeIndex] = static_cast<MapPoint *>(nullptr);
        }
        else
        {
            inCount++;
        }
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap *vSim3_recov =
        static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
    g2oS12_inout = vSim3_recov->estimate();

    return inCount;
}

} // namespace core
} // namespace vs_graphs
