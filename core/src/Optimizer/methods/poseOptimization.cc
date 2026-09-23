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
#include "Utils/Utils/objects/Utils.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

int Optimizer::poseOptimization(Frame *pFrame)
{
    types::SystemParams *p_sysParams = types::SystemParams::getParams();

    g2o::SparseOptimizer                    optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver =
        new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences = 0;

    // Set Frame vertex
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float>    Tcw  = pFrame->getPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                   Tcw.translation().cast<double>()));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<vs_graphs::core::EdgeSE3ProjectXYZOnlyPose *>       vpEdgesMono;
    vector<vs_graphs::core::EdgeSE3ProjectXYZOnlyPoseToBody *> vpEdgesMono_FHR;
    vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
    vpEdgesMono.reserve(N);
    vpEdgesMono_FHR.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeRight.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
    vector<size_t>                                 vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    // DEPTH-AIDED TRACKING: For RGB-D, add depth residuals
    vector<vs_graphs::core::EdgeSE3ProjectXYZDepth *> vpEdgesDepth;
    vector<size_t>                                    vnIndexEdgeDepth;
    vpEdgesDepth.reserve(N);
    vnIndexEdgeDepth.reserve(N);

    const float deltaMono   = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);
    const float deltaDepth  = sqrt(3.841); // chi2 for 1 DoF at 95%

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N; i++)
        {
            MapPoint *pMP = pFrame->mapPoints[i];
            if (pMP)
            {
                // Conventional SLAM
                if (!pFrame->p_camera2)
                {
                    // Monocular observation
                    if (pFrame->uRight[i] < 0)
                    {
                        nInitialCorrespondences++;
                        pFrame->outlierFlags[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        const cv::KeyPoint         &kpUn =
                            pFrame->keyPointsUndistorted[i];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        vs_graphs::core::EdgeSE3ProjectXYZOnlyPose *e =
                            new vs_graphs::core::EdgeSE3ProjectXYZOnlyPose();

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 =
                            pFrame->invLevelSigmaSquared[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaMono);

                        e->pCamera = pFrame->p_camera;
                        e->Xw      = pMP->getWorldPos().cast<double>();

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                    else // Stereo observation
                    {
                        nInitialCorrespondences++;
                        pFrame->outlierFlags[i] = false;

                        Eigen::Matrix<double, 3, 1> obs;
                        const cv::KeyPoint         &kpUn =
                            pFrame->keyPointsUndistorted[i];
                        const float &kp_ur = pFrame->uRight[i];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZOnlyPose *e =
                            new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 =
                            pFrame->invLevelSigmaSquared[kpUn.octave];
                        Eigen::Matrix3d Info =
                            Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaStereo);

                        e->fx = pFrame->fx;
                        e->fy = pFrame->fy;
                        e->cx = pFrame->cx;
                        e->cy = pFrame->cy;
                        e->bf = pFrame->mbf;
                        e->Xw = pMP->getWorldPos().cast<double>();

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }
                }
                // SLAM with respect a rigid body
                else
                {
                    nInitialCorrespondences++;

                    cv::KeyPoint kpUn;

                    if (i < pFrame->Nleft)
                    { // Left camera observation
                        kpUn = pFrame->keyPoints[i];

                        pFrame->outlierFlags[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        vs_graphs::core::EdgeSE3ProjectXYZOnlyPose *e =
                            new vs_graphs::core::EdgeSE3ProjectXYZOnlyPose();

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 =
                            pFrame->invLevelSigmaSquared[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaMono);

                        e->pCamera = pFrame->p_camera;
                        e->Xw      = pMP->getWorldPos().cast<double>();

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                    else
                    {
                        kpUn = pFrame->keyPointsRight[i - pFrame->Nleft];

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        pFrame->outlierFlags[i] = false;

                        vs_graphs::core::EdgeSE3ProjectXYZOnlyPoseToBody *e =
                            new vs_graphs::core::
                                EdgeSE3ProjectXYZOnlyPoseToBody();

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 =
                            pFrame->invLevelSigmaSquared[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaMono);

                        e->pCamera = pFrame->p_camera2;
                        e->Xw      = pMP->getWorldPos().cast<double>();

                        e->mTrl = g2o::SE3Quat(pFrame->getRelativePoseTrl()
                                                   .unit_quaternion()
                                                   .cast<double>(),
                                               pFrame->getRelativePoseTrl()
                                                   .translation()
                                                   .cast<double>());

                        optimizer.addEdge(e);

                        vpEdgesMono_FHR.push_back(e);
                        vnIndexEdgeRight.push_back(i);
                    }
                }
            }
        }
    }

    // DEPTH-AIDED TRACKING: Add depth residuals for RGB-D frames
    // This provides additional constraints from depth measurements
    bool isRGBD = (pFrame->depths.size() > 0 && !pFrame->p_camera2);
    if (isRGBD)
    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);
        for (int i = 0; i < N; i++)
        {
            MapPoint *pMP = pFrame->mapPoints[i];
            if (pMP && !pFrame->outlierFlags[i] &&
                i < (int)pFrame->depths.size())
            {
                float depth = pFrame->depths[i];
                if (depth > 0 &&
                    depth <
                        pFrame
                            ->depthThreshold) // Only use reliable close depths
                {
                    nInitialCorrespondences++;

                    vs_graphs::core::EdgeSE3ProjectXYZDepth *e =
                        new vs_graphs::core::EdgeSE3ProjectXYZDepth();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(0)));
                    e->setMeasurement(depth);

                    // Information matrix for depth (inverse variance)
                    // Higher weight for closer points
                    float invSigma2 =
                        1.0f / (depth * depth * 0.01f); // 1% relative error
                    invSigma2 =
                        std::min(invSigma2, 1000.0f); // Cap maximum weight
                    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                                      invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaDepth);

                    e->pCamera = pFrame->p_camera;
                    e->Xw      = pMP->getWorldPos().cast<double>();

                    optimizer.addEdge(e);

                    vpEdgesDepth.push_back(e);
                    vnIndexEdgeDepth.push_back(i);
                }
            }
        }
    }

    if (nInitialCorrespondences < 3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify
    // observation as inlier/outlier At the next optimization, outliers are not
    // included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]   = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
    const int   its[4]        = {10, 10, 10, 10};

    int nBad = 0;
    for (size_t it = 0; it < 4; it++)
    {
        Tcw = pFrame->getPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),
                                       Tcw.translation().cast<double>()));

        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        // before the last step, remove bad map points
        KeyFrame *refKF = pFrame->p_referenceKeyFrame;
        if (p_sysParams->refineMapPoints.enabled && refKF && it == 2)
        {
            vector<geometric::Plane *>    vpPlanes;
            std::unordered_map<int, bool> planeCheck;

            // populate the vector of planes using the covisibility graph of the
            // reference keyframe
            vector<KeyFrame *> vpRefCovKFs =
                refKF->getBestCovisibilityKeyFrames(25);
            vpRefCovKFs.push_back(refKF);
            for (const auto &pKFi : vpRefCovKFs)
            {
                for (const auto &plane : pKFi->getMapPlanes())
                {
                    if (!plane)
                        continue;
                    if (plane->getPlaneType() !=
                        geometric::Plane::PlaneVariant::UNDEFINED)
                    {
                        if (planeCheck.find(plane->getId()) == planeCheck.end())
                        {
                            vpPlanes.push_back(plane);
                            planeCheck[plane->getId()] = true;
                        }
                    }
                }
            }

            g2o::VertexSE3Expmap *vSE3_recov =
                static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
            Eigen::Isometry3d framePose = vSE3_recov->estimate();
            Eigen::Vector3d   camCenter = framePose.inverse().translation();
            for (const auto &pPlane : vpPlanes)
            {
                if (pPlane->getPlaneType() ==
                    geometric::Plane::PlaneVariant::UNDEFINED)
                    continue;

                Eigen::Vector4d planeEq = pPlane->getGlobalEquation().coeffs();

                // if the camera center is behind the plane, skip the plane
                if (planeEq.head<3>().dot(camCenter) + planeEq(3) < 0)
                    continue;

                // for each map point in the frame, check if it is on the plane
                for (size_t j = 0; j < static_cast<size_t>(pFrame->N); j++)
                {
                    MapPoint *pMP = pFrame->mapPoints[j];
                    if (!pMP || pMP->isBad())
                        continue;

                    // calculate distance from the map point to the plane
                    Eigen::Vector3d pMPw = pMP->getWorldPos().cast<double>();
                    double distance = planeEq.head<3>().dot(pMPw) + planeEq(3);
                    if (distance <
                        -p_sysParams->refineMapPoints.maxDistanceForDelete)
                    {
                        // get the intersection point of the line joining the
                        // camera center and the map point with the plane
                        Eigen::Vector3d intersect =
                            utils::utils::Utils::lineIntersectsPlane(planeEq,
                                                                     camCenter,
                                                                     pMPw);

                        // check if the map point is in the plane cloud
                        if (pPlane->isPointinPlaneCloud(intersect))
                        {
                            pFrame->mapPoints[j]->setBadFlag();
                            pFrame->mapPoints[j] =
                                static_cast<MapPoint *>(nullptr);
                            pFrame->outlierFlags[j] = true;
                        }
                    }
                }
            }
        }

        nBad = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            vs_graphs::core::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];
            if (it == 2)
                e->setRobustKernel(0);

            if (pFrame->outlierFlags[idx])
            {
                if (!pFrame->mapPoints[idx])
                {
                    optimizer.removeEdge(e);
                    nBad++;
                    continue;
                }
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it])
            {
                pFrame->outlierFlags[idx] = true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->outlierFlags[idx] = false;
                e->setLevel(0);
            }
        }

        for (size_t i = 0, iend = vpEdgesMono_FHR.size(); i < iend; i++)
        {
            vs_graphs::core::EdgeSE3ProjectXYZOnlyPoseToBody *e =
                vpEdgesMono_FHR[i];

            const size_t idx = vnIndexEdgeRight[i];
            if (it == 2)
                e->setRobustKernel(0);

            if (pFrame->outlierFlags[idx])
            {
                if (!pFrame->mapPoints[idx])
                {
                    optimizer.removeEdge(e);
                    nBad++;
                    continue;
                }
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it])
            {
                pFrame->outlierFlags[idx] = true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->outlierFlags[idx] = false;
                e->setLevel(0);
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];
            if (it == 2)
                e->setRobustKernel(0);

            if (pFrame->outlierFlags[idx])
            {
                if (!pFrame->mapPoints[idx])
                {
                    optimizer.removeEdge(e);
                    nBad++;
                    continue;
                }
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[it])
            {
                pFrame->outlierFlags[idx] = true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                e->setLevel(0);
                pFrame->outlierFlags[idx] = false;
            }
        }

        // DEPTH-AIDED TRACKING: Process depth edges
        for (size_t i = 0, iend = vpEdgesDepth.size(); i < iend; i++)
        {
            vs_graphs::core::EdgeSE3ProjectXYZDepth *e = vpEdgesDepth[i];

            const size_t idx = vnIndexEdgeDepth[i];
            if (it == 2)
                e->setRobustKernel(0);

            if (pFrame->outlierFlags[idx])
            {
                if (!pFrame->mapPoints[idx])
                {
                    optimizer.removeEdge(e);
                    nBad++;
                    continue;
                }
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > deltaDepth)
            {
                pFrame->outlierFlags[idx] = true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                e->setLevel(0);
                pFrame->outlierFlags[idx] = false;
            }
        }

        if (optimizer.edges().size() < 10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap *vSE3_recov =
        static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
    g2o::SE3Quat       SE3quat_recov = vSE3_recov->estimate();
    Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                            SE3quat_recov.translation().cast<float>());
    pFrame->setPose(pose);

    return nInitialCorrespondences - nBad;
}

} // namespace core
} // namespace vs_graphs
