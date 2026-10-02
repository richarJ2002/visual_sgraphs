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
 * @file            poseOptimization.cc
 *
 * @brief           Implements Optimizer::poseOptimization(), declared in
 *                  Optimizer.h.
 */

#include "Optimizer.h"

#include "OptimizableTypes.h"
#include "Utils/Utils/objects/Utils.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

OptimizerStatus Optimizer::poseOptimization(Frame *p_frame_inout,
                                            int   &inlierCount_out)
{
    types::SystemParams *p_sysParams = nullptr;
    if (types::SystemParams::getParams(p_sysParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    g2o::SparseOptimizer                    optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *p_linearSolver;

    p_linearSolver =
        new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(p_linearSolver);

    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(p_solver);

    int initialCorrespondenceCount = 0;

    // Set Frame vertex
    g2o::VertexSE3Expmap *p_se3Vertex = new g2o::VertexSE3Expmap();
    Sophus::SE3<float>    cameraPose_worldToCamera{};
    if (p_frame_inout->getPose(cameraPose_worldToCamera) !=
        FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    p_se3Vertex->setEstimate(
        g2o::SE3Quat(cameraPose_worldToCamera.unit_quaternion().cast<double>(),
                     cameraPose_worldToCamera.translation().cast<double>()));
    p_se3Vertex->setId(0);
    p_se3Vertex->setFixed(false);
    optimizer.addVertex(p_se3Vertex);

    // Set MapPoint vertices
    const int N = p_frame_inout->keyPointCount;

    std::vector<vs_graphs::core::EdgeSE3ProjectXYZOnlyPose *> edgesMonos;
    std::vector<vs_graphs::core::EdgeSE3ProjectXYZOnlyPoseToBody *>
                        vpEdgesMonoFhr;
    std::vector<size_t> monoEdgeIndices, rightEdgeIndices;
    edgesMonos.reserve(N);
    vpEdgesMonoFhr.reserve(N);
    monoEdgeIndices.reserve(N);
    rightEdgeIndices.reserve(N);

    std::vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> edgesStereos;
    std::vector<size_t>                                 stereoEdgeIndices;
    edgesStereos.reserve(N);
    stereoEdgeIndices.reserve(N);

    // DEPTH-AIDED TRACKING: For RGB-D, add depth residuals
    std::vector<vs_graphs::core::EdgeSE3ProjectXYZDepth *> edgesDepths;
    std::vector<size_t>                                    depthEdgeIndices;
    edgesDepths.reserve(N);
    depthEdgeIndices.reserve(N);

    const float deltaMono   = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);
    const float deltaDepth  = sqrt(3.841); // chi2 for 1 DoF at 95%

    {
        std::unique_lock<std::mutex> lock(MapPoint::globalMutex);

        for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
        {
            MapPoint *p_mapPoint = p_frame_inout->mapPoints[keyPointIndex];
            if (p_mapPoint)
            {
                // Conventional SLAM
                if (!p_frame_inout->p_camera2)
                {
                    // Monocular observation
                    if (p_frame_inout->uRight[keyPointIndex] < 0)
                    {
                        initialCorrespondenceCount++;
                        p_frame_inout->outlierFlags[keyPointIndex] = false;

                        Eigen::Matrix<double, 2, 1> observation;
                        const cv::KeyPoint         &keyPointUn =
                            p_frame_inout->keyPointsUndistorted[keyPointIndex];
                        observation << keyPointUn.pt.x, keyPointUn.pt.y;

                        vs_graphs::core::EdgeSE3ProjectXYZOnlyPose *e =
                            new vs_graphs::core::EdgeSE3ProjectXYZOnlyPose();

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
                        e->setMeasurement(observation);
                        const float invSigma2 =
                            p_frame_inout
                                ->invLevelSigmaSquared[keyPointUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber *p_robustKernel =
                            new g2o::RobustKernelHuber;
                        e->setRobustKernel(p_robustKernel);
                        p_robustKernel->setDelta(deltaMono);

                        e->p_camera = p_frame_inout->p_camera;
                        Eigen::Vector3f mapPointWorldPos{};
                        if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getWorldPos returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        e->Xw = mapPointWorldPos.cast<double>();

                        optimizer.addEdge(e);

                        edgesMonos.push_back(e);
                        monoEdgeIndices.push_back(keyPointIndex);
                    }
                    else // Stereo observation
                    {
                        initialCorrespondenceCount++;
                        p_frame_inout->outlierFlags[keyPointIndex] = false;

                        Eigen::Matrix<double, 3, 1> observation;
                        const cv::KeyPoint         &keyPointUn =
                            p_frame_inout->keyPointsUndistorted[keyPointIndex];
                        const float &rightKeyPointU =
                            p_frame_inout->uRight[keyPointIndex];
                        observation << keyPointUn.pt.x, keyPointUn.pt.y,
                            rightKeyPointU;

                        g2o::EdgeStereoSE3ProjectXYZOnlyPose *e =
                            new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
                        e->setMeasurement(observation);
                        const float invSigma2 =
                            p_frame_inout
                                ->invLevelSigmaSquared[keyPointUn.octave];
                        Eigen::Matrix3d Info =
                            Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *p_robustKernel =
                            new g2o::RobustKernelHuber;
                        e->setRobustKernel(p_robustKernel);
                        p_robustKernel->setDelta(deltaStereo);

                        e->fx = p_frame_inout->fx;
                        e->fy = p_frame_inout->fy;
                        e->cx = p_frame_inout->cx;
                        e->cy = p_frame_inout->cy;
                        e->bf = p_frame_inout->mbf;
                        Eigen::Vector3f mapPointWorldPos2{};
                        if (p_mapPoint->getWorldPos(mapPointWorldPos2) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getWorldPos returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        e->Xw = mapPointWorldPos2.cast<double>();

                        optimizer.addEdge(e);

                        edgesStereos.push_back(e);
                        stereoEdgeIndices.push_back(keyPointIndex);
                    }
                }
                // SLAM with respect a rigid body
                else
                {
                    initialCorrespondenceCount++;

                    cv::KeyPoint keyPointUn;

                    if (keyPointIndex < p_frame_inout->leftKeyPointCount)
                    { // Left camera observation
                        keyPointUn = p_frame_inout->keyPoints[keyPointIndex];

                        p_frame_inout->outlierFlags[keyPointIndex] = false;

                        Eigen::Matrix<double, 2, 1> observation;
                        observation << keyPointUn.pt.x, keyPointUn.pt.y;

                        vs_graphs::core::EdgeSE3ProjectXYZOnlyPose *e =
                            new vs_graphs::core::EdgeSE3ProjectXYZOnlyPose();

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
                        e->setMeasurement(observation);
                        const float invSigma2 =
                            p_frame_inout
                                ->invLevelSigmaSquared[keyPointUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber *p_robustKernel =
                            new g2o::RobustKernelHuber;
                        e->setRobustKernel(p_robustKernel);
                        p_robustKernel->setDelta(deltaMono);

                        e->p_camera = p_frame_inout->p_camera;
                        Eigen::Vector3f mapPointWorldPos3{};
                        if (p_mapPoint->getWorldPos(mapPointWorldPos3) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getWorldPos returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        e->Xw = mapPointWorldPos3.cast<double>();

                        optimizer.addEdge(e);

                        edgesMonos.push_back(e);
                        monoEdgeIndices.push_back(keyPointIndex);
                    }
                    else
                    {
                        keyPointUn = p_frame_inout->keyPointsRight
                                         [keyPointIndex -
                                          p_frame_inout->leftKeyPointCount];

                        Eigen::Matrix<double, 2, 1> observation;
                        observation << keyPointUn.pt.x, keyPointUn.pt.y;

                        p_frame_inout->outlierFlags[keyPointIndex] = false;

                        vs_graphs::core::EdgeSE3ProjectXYZOnlyPoseToBody *e =
                            new vs_graphs::core::
                                EdgeSE3ProjectXYZOnlyPoseToBody();

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(0)));
                        e->setMeasurement(observation);
                        const float invSigma2 =
                            p_frame_inout
                                ->invLevelSigmaSquared[keyPointUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber *p_robustKernel =
                            new g2o::RobustKernelHuber;
                        e->setRobustKernel(p_robustKernel);
                        p_robustKernel->setDelta(deltaMono);

                        e->p_camera = p_frame_inout->p_camera2;
                        Eigen::Vector3f mapPointWorldPos4{};
                        if (p_mapPoint->getWorldPos(mapPointWorldPos4) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getWorldPos returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        e->Xw = mapPointWorldPos4.cast<double>();

                        Sophus::SE3f frameRelativePoseTrl{};
                        if (p_frame_inout->getRelativePoseTrl(
                                frameRelativePoseTrl) !=
                            FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getRelativePoseTrl returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        Sophus::SE3f frameRelativePoseTrl2{};
                        if (p_frame_inout->getRelativePoseTrl(
                                frameRelativePoseTrl2) !=
                            FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getRelativePoseTrl returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        e->mTrl = g2o::SE3Quat(
                            frameRelativePoseTrl.unit_quaternion()
                                .cast<double>(),
                            frameRelativePoseTrl2.translation().cast<double>());

                        optimizer.addEdge(e);

                        vpEdgesMonoFhr.push_back(e);
                        rightEdgeIndices.push_back(keyPointIndex);
                    }
                }
            }
        }
    }

    // DEPTH-AIDED TRACKING: Add depth residuals for RGB-D frames
    // This provides additional constraints from depth measurements
    bool isRgbd =
        (p_frame_inout->depths.size() > 0 && !p_frame_inout->p_camera2);
    if (isRgbd)
    {
        std::unique_lock<std::mutex> lock(MapPoint::globalMutex);
        for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
        {
            MapPoint *p_mapPoint = p_frame_inout->mapPoints[keyPointIndex];
            if (p_mapPoint && !p_frame_inout->outlierFlags[keyPointIndex] &&
                keyPointIndex < static_cast<int>(p_frame_inout->depths.size()))
            {
                float depth = p_frame_inout->depths[keyPointIndex];
                if (depth > 0 &&
                    depth <
                        p_frame_inout
                            ->depthThreshold) // Only use reliable close depths
                {
                    initialCorrespondenceCount++;

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

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(deltaDepth);

                    e->p_camera = p_frame_inout->p_camera;
                    Eigen::Vector3f mapPointWorldPos5{};
                    if (p_mapPoint->getWorldPos(mapPointWorldPos5) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWorldPos returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    e->Xw = mapPointWorldPos5.cast<double>();

                    optimizer.addEdge(e);

                    edgesDepths.push_back(e);
                    depthEdgeIndices.push_back(keyPointIndex);
                }
            }
        }
    }

    if (initialCorrespondenceCount < 3)
    {
        inlierCount_out = 0;
        return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
    }

    // We perform 4 optimizations, after each optimization we classify
    // observation as inlier/outlier At the next optimization, outliers are not
    // included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]   = {5.991, 5.991, 5.991, 5.991};
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
    const int   its[4]        = {10, 10, 10, 10};

    int badCount = 0;
    for (size_t iterationIndex = 0; iterationIndex < 4; iterationIndex++)
    {
        Sophus::SE3<float> frameGetPose{};
        if (p_frame_inout->getPose(frameGetPose) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        cameraPose_worldToCamera = frameGetPose;
        p_se3Vertex->setEstimate(g2o::SE3Quat(
            cameraPose_worldToCamera.unit_quaternion().cast<double>(),
            cameraPose_worldToCamera.translation().cast<double>()));

        optimizer.initializeOptimization(0);
        optimizer.optimize(its[iterationIndex]);

        // before the last step, remove bad map points
        KeyFrame *p_referenceKeyFrame = p_frame_inout->p_referenceKeyFrame;
        if (p_sysParams->refineMapPoints.enabled && p_referenceKeyFrame &&
            iterationIndex == 2)
        {
            std::vector<geometric::Plane *> planes;
            std::unordered_map<int, bool>   planeCheck;

            // populate the vector of planes using the covisibility graph of the
            // reference keyframe
            std::vector<KeyFrame *> referenceCovisibleKeyFrames{};
            if (p_referenceKeyFrame->getBestCovisibilityKeyFrames(
                    25,
                    referenceCovisibleKeyFrames) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getBestCovisibilityKeyFrames returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            referenceCovisibleKeyFrames.push_back(p_referenceKeyFrame);
            for (KeyFrame *const &keyFrame : referenceCovisibleKeyFrames)
            {
                std::vector<geometric::Plane *> keyFrameMapPlanes{};
                if (keyFrame->getMapPlanes(keyFrameMapPlanes) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMapPlanes returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                for (geometric::Plane *const &plane : keyFrameMapPlanes)
                {
                    if (!plane)
                        continue;
                    geometric::Plane::PlaneVariant planeType{};
                    if (plane->getPlaneType(planeType) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getPlaneType returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (planeType != geometric::Plane::PlaneVariant::UNDEFINED)
                    {
                        int planeGetId{};
                        if (plane->getId(planeGetId) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        if (planeCheck.find(planeGetId) == planeCheck.end())
                        {
                            planes.push_back(plane);
                            int planeGetId2{};
                            if (plane->getId(planeGetId2) !=
                                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: getId returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            planeCheck[planeGetId2] = true;
                        }
                    }
                }
            }

            g2o::VertexSE3Expmap *p_recoveredPoseVertex =
                static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
            Eigen::Isometry3d framePose    = p_recoveredPoseVertex->estimate();
            Eigen::Vector3d   cameraCenter = framePose.inverse().translation();
            for (geometric::Plane *const &candidatePlane : planes)
            {
                geometric::Plane::PlaneVariant candidatePlanePlaneType{};
                if (candidatePlane->getPlaneType(candidatePlanePlaneType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPlaneType returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (candidatePlanePlaneType ==
                    geometric::Plane::PlaneVariant::UNDEFINED)
                    continue;

                g2o::Plane3D candidatePlaneGetGlobalEquation{};
                if (candidatePlane->getGlobalEquation(
                        candidatePlaneGetGlobalEquation) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getGlobalEquation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector4d planeEq =
                    candidatePlaneGetGlobalEquation.coeffs();

                // if the camera center is behind the plane, skip the plane
                if (planeEq.head<3>().dot(cameraCenter) + planeEq(3) < 0)
                    continue;

                // for each map point in the frame, check if it is on the plane
                for (size_t j = 0;
                     j < static_cast<size_t>(p_frame_inout->keyPointCount);
                     j++)
                {
                    MapPoint *p_mapPoint = p_frame_inout->mapPoints[j];
                    bool      mapPointIsBad{};
                    if (!(!p_mapPoint) &&
                        p_mapPoint->isBad(mapPointIsBad) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (!p_mapPoint || mapPointIsBad)
                        continue;

                    // calculate distance from the map point to the plane
                    Eigen::Vector3f mapPointWorldPos6{};
                    if (p_mapPoint->getWorldPos(mapPointWorldPos6) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWorldPos returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    Eigen::Vector3d pMPw = mapPointWorldPos6.cast<double>();
                    double distance = planeEq.head<3>().dot(pMPw) + planeEq(3);
                    if (distance <
                        -p_sysParams->refineMapPoints.maxDistanceForDelete)
                    {
                        // get the intersection point of the line joining the
                        // camera center and the map point with the plane
                        Eigen::Vector3d intersect{};
                        if (utils::utils::Utils::lineIntersectsPlane(
                                planeEq,
                                cameraCenter,
                                pMPw,
                                intersect) !=
                            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
                        {
                            // lineIntersectsPlane cannot fail; continue as
                            // before.
                        }

                        // check if the map point is in the plane cloud
                        bool candidatePlaneIsPointinPlaneCloud{};
                        if (candidatePlane->isPointinPlaneCloud(
                                intersect,
                                candidatePlaneIsPointinPlaneCloud) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // isPointinPlaneCloud cannot fail; continue as
                            // before.
                        }
                        if (candidatePlaneIsPointinPlaneCloud)
                        {
                            if (p_frame_inout->mapPoints[j]->setBadFlag() !=
                                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: setBadFlag returned a failure status "
                                    "although it cannot fail; continuing as "
                                    "before.",
                                    __func__);
                            }
                            p_frame_inout->mapPoints[j] =
                                static_cast<MapPoint *>(nullptr);
                            p_frame_inout->outlierFlags[j] = true;
                        }
                    }
                }
            }
        }

        badCount = 0;
        for (size_t keyPointIndex = 0, iend = edgesMonos.size();
             keyPointIndex < iend;
             keyPointIndex++)
        {
            vs_graphs::core::EdgeSE3ProjectXYZOnlyPose *e =
                edgesMonos[keyPointIndex];

            const size_t featureIndex = monoEdgeIndices[keyPointIndex];
            if (iterationIndex == 2)
                e->setRobustKernel(0);

            if (p_frame_inout->outlierFlags[featureIndex])
            {
                if (!p_frame_inout->mapPoints[featureIndex])
                {
                    optimizer.removeEdge(e);
                    badCount++;
                    continue;
                }
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[iterationIndex])
            {
                p_frame_inout->outlierFlags[featureIndex] = true;
                e->setLevel(1);
                badCount++;
            }
            else
            {
                p_frame_inout->outlierFlags[featureIndex] = false;
                e->setLevel(0);
            }
        }

        for (size_t keyPointIndex = 0, iend = vpEdgesMonoFhr.size();
             keyPointIndex < iend;
             keyPointIndex++)
        {
            vs_graphs::core::EdgeSE3ProjectXYZOnlyPoseToBody *e =
                vpEdgesMonoFhr[keyPointIndex];

            const size_t featureIndex = rightEdgeIndices[keyPointIndex];
            if (iterationIndex == 2)
                e->setRobustKernel(0);

            if (p_frame_inout->outlierFlags[featureIndex])
            {
                if (!p_frame_inout->mapPoints[featureIndex])
                {
                    optimizer.removeEdge(e);
                    badCount++;
                    continue;
                }
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[iterationIndex])
            {
                p_frame_inout->outlierFlags[featureIndex] = true;
                e->setLevel(1);
                badCount++;
            }
            else
            {
                p_frame_inout->outlierFlags[featureIndex] = false;
                e->setLevel(0);
            }
        }

        for (size_t keyPointIndex = 0, iend = edgesStereos.size();
             keyPointIndex < iend;
             keyPointIndex++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose *e =
                edgesStereos[keyPointIndex];

            const size_t featureIndex = stereoEdgeIndices[keyPointIndex];
            if (iterationIndex == 2)
                e->setRobustKernel(0);

            if (p_frame_inout->outlierFlags[featureIndex])
            {
                if (!p_frame_inout->mapPoints[featureIndex])
                {
                    optimizer.removeEdge(e);
                    badCount++;
                    continue;
                }
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Stereo[iterationIndex])
            {
                p_frame_inout->outlierFlags[featureIndex] = true;
                e->setLevel(1);
                badCount++;
            }
            else
            {
                e->setLevel(0);
                p_frame_inout->outlierFlags[featureIndex] = false;
            }
        }

        // DEPTH-AIDED TRACKING: Process depth edges
        for (size_t keyPointIndex = 0, iend = edgesDepths.size();
             keyPointIndex < iend;
             keyPointIndex++)
        {
            vs_graphs::core::EdgeSE3ProjectXYZDepth *e =
                edgesDepths[keyPointIndex];

            const size_t featureIndex = depthEdgeIndices[keyPointIndex];
            if (iterationIndex == 2)
                e->setRobustKernel(0);

            if (p_frame_inout->outlierFlags[featureIndex])
            {
                if (!p_frame_inout->mapPoints[featureIndex])
                {
                    optimizer.removeEdge(e);
                    badCount++;
                    continue;
                }
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > deltaDepth)
            {
                p_frame_inout->outlierFlags[featureIndex] = true;
                e->setLevel(1);
                badCount++;
            }
            else
            {
                e->setLevel(0);
                p_frame_inout->outlierFlags[featureIndex] = false;
            }
        }

        if (optimizer.edges().size() < 10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap *p_recoveredPoseVertex =
        static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
    g2o::SE3Quat       recoveredPose = p_recoveredPoseVertex->estimate();
    Sophus::SE3<float> pose(recoveredPose.rotation().cast<float>(),
                            recoveredPose.translation().cast<float>());
    if (p_frame_inout->setPose(pose) != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    inlierCount_out = initialCorrespondenceCount - badCount;
    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
