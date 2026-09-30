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
#include "OptimizerEdgeLookup.h"
#include "System.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

OptimizerStatus Optimizer::loopClosureLocalBundleAdjustment(
    KeyFrame               *p_mainKeyFrame_in,
    std::vector<KeyFrame *> adjustKeyFrames_in,
    std::vector<KeyFrame *> fixedKeyFrames_in,
    bool                   *p_pbStopFlag_in)
{
    // Variables
    std::vector<MapPoint *> mapPoints;
    std::set<KeyFrame *>    keyFrameBas;
    long unsigned int       maximumKeyFrameId = 0;
    g2o::SparseOptimizer    optimizer;

    // Define a linear solver to solve the linear system arising while
    // optimization
    g2o::BlockSolver_6_3::LinearSolverType *p_linearSolver;
    p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(p_linearSolver);

    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(p_solver);
    optimizer.setVerbose(false);

    // Force stop flag
    if (p_pbStopFlag_in)
        optimizer.setForceStopFlag(p_pbStopFlag_in);

    // Get the current map
    Map *p_currentMap = nullptr;
    if (p_mainKeyFrame_in->getMap(p_currentMap) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    // Set fixed KeyFrame vertices
    int insertedPointCount = 0;
    for (KeyFrame *p_adjustKeyFrame : fixedKeyFrames_in)
    {
        // Skip the KeyFrame if it is bad or is not in the current map
        bool adjustKeyFrameIsBad{};
        if (p_adjustKeyFrame->isBad(adjustKeyFrameIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Map *p_adjustKeyFrameMap = nullptr;
        if (!(adjustKeyFrameIsBad) &&
            p_adjustKeyFrame->getMap(p_adjustKeyFrameMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (adjustKeyFrameIsBad || p_adjustKeyFrameMap != p_currentMap)
        {
            if (Verbose::printMess(
                    "[Error in LoopClosureLocalBundleAdjustment] "
                    "KeyFrame is bad or is not in the current map!",
                    Verbose::VERBOSITY_NORMAL) !=
                VerboseStatus::VERBOSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: printMess returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            continue;
        }

        // Set the local BA id for the KeyFrame
        p_adjustKeyFrame->baLocalMergeId = p_mainKeyFrame_in->id;

        // Create a new vertex for the KeyFrame
        g2o::VertexSE3Expmap *p_se3Vertex = new g2o::VertexSE3Expmap();
        Sophus::SE3<float>    poseWorldToCamera{};
        if (p_adjustKeyFrame->getPose(poseWorldToCamera) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_se3Vertex->setEstimate(
            g2o::SE3Quat(poseWorldToCamera.unit_quaternion().cast<double>(),
                         poseWorldToCamera.translation().cast<double>()));
        p_se3Vertex->setId(p_adjustKeyFrame->id);
        p_se3Vertex->setFixed(true);
        optimizer.addVertex(p_se3Vertex);
        if (p_adjustKeyFrame->id > maximumKeyFrameId)
            maximumKeyFrameId = p_adjustKeyFrame->id;

        // Get the map points observed by the KeyFrame
        std::set<MapPoint *> viewMapPoints{};
        if (p_adjustKeyFrame->getMapPoints(viewMapPoints) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPoints returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (MapPoint *p_viewMapPoint : viewMapPoints)
        {
            if (p_viewMapPoint)
            {
                bool viewMapPointIsBad{};
                if (p_viewMapPoint->isBad(viewMapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                Map *p_viewMapPointMap = nullptr;
                if ((!viewMapPointIsBad) &&
                    p_viewMapPoint->getMap(p_viewMapPointMap) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (!viewMapPointIsBad && p_viewMapPointMap == p_currentMap)
                {
                    if (p_viewMapPoint->baLocalMergeId != p_mainKeyFrame_in->id)
                    {
                        // Add the map point to the list of optimizable map
                        // points
                        mapPoints.push_back(p_viewMapPoint);
                        p_viewMapPoint->baLocalMergeId = p_mainKeyFrame_in->id;
                        insertedPointCount++;
                    }
                }
            }
        }

        keyFrameBas.insert(p_adjustKeyFrame);
    }

    // Set non-fixed KeyFrame vertices
    std::set<KeyFrame *> adjustKeyFrames(adjustKeyFrames_in.begin(),
                                         adjustKeyFrames_in.end());
    insertedPointCount = 0;
    for (KeyFrame *p_adjustKeyFrame : adjustKeyFrames_in)
    {
        bool adjustKeyFrameIsBad2{};
        if (p_adjustKeyFrame->isBad(adjustKeyFrameIsBad2) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Map *p_adjustKeyFrameMap2 = nullptr;
        if (!(adjustKeyFrameIsBad2) &&
            p_adjustKeyFrame->getMap(p_adjustKeyFrameMap2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (adjustKeyFrameIsBad2 || p_adjustKeyFrameMap2 != p_currentMap)
            continue;

        p_adjustKeyFrame->baLocalMergeId = p_mainKeyFrame_in->id;

        g2o::VertexSE3Expmap *p_se3Vertex = new g2o::VertexSE3Expmap();
        Sophus::SE3<float>    poseWorldToCamera{};
        if (p_adjustKeyFrame->getPose(poseWorldToCamera) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_se3Vertex->setEstimate(
            g2o::SE3Quat(poseWorldToCamera.unit_quaternion().cast<double>(),
                         poseWorldToCamera.translation().cast<double>()));
        p_se3Vertex->setId(p_adjustKeyFrame->id);
        optimizer.addVertex(p_se3Vertex);
        if (p_adjustKeyFrame->id > maximumKeyFrameId)
            maximumKeyFrameId = p_adjustKeyFrame->id;

        std::set<MapPoint *> viewMapPoints{};
        if (p_adjustKeyFrame->getMapPoints(viewMapPoints) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPoints returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (MapPoint *p_viewMapPoint : viewMapPoints)
        {
            if (p_viewMapPoint)
            {
                bool viewMapPointIsBad2{};
                if (p_viewMapPoint->isBad(viewMapPointIsBad2) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                Map *p_viewMapPointMap2 = nullptr;
                if ((!viewMapPointIsBad2) &&
                    p_viewMapPoint->getMap(p_viewMapPointMap2) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (!viewMapPointIsBad2 && p_viewMapPointMap2 == p_currentMap)
                {
                    if (p_viewMapPoint->baLocalMergeId != p_mainKeyFrame_in->id)
                    {
                        mapPoints.push_back(p_viewMapPoint);
                        p_viewMapPoint->baLocalMergeId = p_mainKeyFrame_in->id;
                        insertedPointCount++;
                    }
                }
            }
        }

        keyFrameBas.insert(p_adjustKeyFrame);
    }

    const int expectedSizeCount =
        (adjustKeyFrames_in.size() + fixedKeyFrames_in.size()) *
        mapPoints.size();

    std::vector<vs_graphs::core::EdgeSE3ProjectXYZ *> edgesMonos;
    edgesMonos.reserve(expectedSizeCount);

    std::vector<KeyFrame *> edgeKeyFrameMonos;
    edgeKeyFrameMonos.reserve(expectedSizeCount);

    std::vector<MapPoint *> mapPointEdgeMonos;
    mapPointEdgeMonos.reserve(expectedSizeCount);

    std::vector<g2o::EdgeStereoSE3ProjectXYZ *> edgesStereos;
    edgesStereos.reserve(expectedSizeCount);

    std::vector<KeyFrame *> edgeKeyFrameStereos;
    edgeKeyFrameStereos.reserve(expectedSizeCount);

    std::vector<MapPoint *> mapPointEdgeStereos;
    mapPointEdgeStereos.reserve(expectedSizeCount);

    const float thresholdHuber2d = sqrt(5.99);
    const float thresholdHuber3d = sqrt(7.815);

    // Set MapPoint vertices
    std::map<KeyFrame *, int> observationKeyFrames;
    std::map<KeyFrame *, int> observationFinalKeyFrames;
    std::map<MapPoint *, int> observationMapPoints;
    for (unsigned int mapPointIndex = 0; mapPointIndex < mapPoints.size();
         ++mapPointIndex)
    {
        MapPoint *p_viewMapPoint = mapPoints[mapPointIndex];
        bool      viewMapPointIsBad3{};
        if (p_viewMapPoint->isBad(viewMapPointIsBad3) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (viewMapPointIsBad3)
            continue;

        g2o::VertexSBAPointXYZ *p_pointVertex = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f         viewMapPointWorldPos{};
        if (p_viewMapPoint->getWorldPos(viewMapPointWorldPos) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        p_pointVertex->setEstimate(viewMapPointWorldPos.cast<double>());
        const int id = p_viewMapPoint->id + maximumKeyFrameId + 1;
        p_pointVertex->setId(id);
        p_pointVertex->setMarginalized(true);
        optimizer.addVertex(p_pointVertex);

        std::map<KeyFrame *, std::tuple<int, int>> observations{};
        if (p_viewMapPoint->getObservations(observations) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int edgeCount = 0;
        // SET EDGES
        for (std::map<KeyFrame *, std::tuple<int, int>>::const_iterator mit =
                 observations.begin();
             mit != observations.end();
             mit++)
        {
            KeyFrame *p_keyFrame = mit->first;
            bool      keyFrameIsBad{};
            if (p_keyFrame->isBad(keyFrameIsBad) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            MapPoint *p_keyFrameMapPoint = nullptr;
            if (!(keyFrameIsBad || p_keyFrame->id > maximumKeyFrameId ||
                  p_keyFrame->baLocalMergeId != p_mainKeyFrame_in->id) &&
                p_keyFrame->getMapPoint(std::get<0>(mit->second),
                                        p_keyFrameMapPoint) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (keyFrameIsBad || p_keyFrame->id > maximumKeyFrameId ||
                p_keyFrame->baLocalMergeId != p_mainKeyFrame_in->id ||
                !p_keyFrameMapPoint)
                continue;

            edgeCount++;

            const cv::KeyPoint &keyPointUn =
                p_keyFrame->keyPointsUndistorted[std::get<0>(mit->second)];

            if (p_keyFrame->uRight[std::get<0>(mit->second)] < 0) // Monocular
            {
                observationMapPoints[p_viewMapPoint]++;
                Eigen::Matrix<double, 2, 1> observation;
                observation << keyPointUn.pt.x, keyPointUn.pt.y;

                vs_graphs::core::EdgeSE3ProjectXYZ *e =
                    new vs_graphs::core::EdgeSE3ProjectXYZ();

                e->setVertex(0,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(id)));
                e->setVertex(1,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(p_keyFrame->id)));
                e->setMeasurement(observation);
                const float &invSigma2 =
                    p_keyFrame->invLevelSigmaSquared[keyPointUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                g2o::RobustKernelHuber *p_robustKernel =
                    new g2o::RobustKernelHuber;
                e->setRobustKernel(p_robustKernel);
                p_robustKernel->setDelta(thresholdHuber2d);

                e->p_camera = p_keyFrame->p_camera;

                optimizer.addEdge(e);

                edgesMonos.push_back(e);
                edgeKeyFrameMonos.push_back(p_keyFrame);
                mapPointEdgeMonos.push_back(p_viewMapPoint);

                observationKeyFrames[p_keyFrame]++;
            }
            else // RGBD or Stereo
            {
                observationMapPoints[p_viewMapPoint] += 2;
                Eigen::Matrix<double, 3, 1> observation;
                const float                 rightKeyPointU =
                    p_keyFrame->uRight[std::get<0>(mit->second)];
                observation << keyPointUn.pt.x, keyPointUn.pt.y, rightKeyPointU;

                g2o::EdgeStereoSE3ProjectXYZ *e =
                    new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(id)));
                e->setVertex(1,
                             dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                 optimizer.vertex(p_keyFrame->id)));
                e->setMeasurement(observation);
                const float &invSigma2 =
                    p_keyFrame->invLevelSigmaSquared[keyPointUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber *p_robustKernel =
                    new g2o::RobustKernelHuber;
                e->setRobustKernel(p_robustKernel);
                p_robustKernel->setDelta(thresholdHuber3d);

                e->fx = p_keyFrame->fx;
                e->fy = p_keyFrame->fy;
                e->cx = p_keyFrame->cx;
                e->cy = p_keyFrame->cy;
                e->bf = p_keyFrame->mbf;

                optimizer.addEdge(e);

                edgesStereos.push_back(e);
                edgeKeyFrameStereos.push_back(p_keyFrame);
                mapPointEdgeStereos.push_back(p_viewMapPoint);

                observationKeyFrames[p_keyFrame]++;
            }
        }
    }

    if (p_pbStopFlag_in)
        if (*p_pbStopFlag_in)
            return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool shouldOptimizeMore = true;

    if (p_pbStopFlag_in)
        if (*p_pbStopFlag_in)
            shouldOptimizeMore = false;

    std::map<unsigned long int, int> wrongObservationKeyFrame;
    if (shouldOptimizeMore)
    {
        // Check inlier observations
        int badMonoMapPoint = 0, badStereoMapPoint = 0;
        for (size_t mapPointIndex = 0, iend = edgesMonos.size();
             mapPointIndex < iend;
             mapPointIndex++)
        {
            vs_graphs::core::EdgeSE3ProjectXYZ *e = edgesMonos[mapPointIndex];
            MapPoint *p_mapPoint = mapPointEdgeMonos[mapPointIndex];

            bool mapPointIsBad{};
            if (p_mapPoint->isBad(mapPointIsBad) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (mapPointIsBad)
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
                badMonoMapPoint++;
            }
            e->setRobustKernel(0);
        }

        for (size_t mapPointIndex = 0, iend = edgesStereos.size();
             mapPointIndex < iend;
             mapPointIndex++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = edgesStereos[mapPointIndex];
            MapPoint *p_mapPoint = mapPointEdgeStereos[mapPointIndex];

            bool mapPointIsBad2{};
            if (p_mapPoint->isBad(mapPointIsBad2) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (mapPointIsBad2)
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
                badStereoMapPoint++;
            }

            e->setRobustKernel(0);
        }
        if (Verbose::printMess(
                "[BA]: First optimization(Huber), there are " +
                    std::to_string(badMonoMapPoint) + " monocular and " +
                    std::to_string(badStereoMapPoint) + " stereo bad edges",
                Verbose::VERBOSITY_DEBUG) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }

    std::vector<std::pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(edgesMonos.size() + edgesStereos.size());
    std::set<MapPoint *> erasedMapPoints;
    std::set<KeyFrame *> erasedKeyFrames;

    // Check inlier observations
    int badMonoMapPoint = 0, badStereoMapPoint = 0;
    for (size_t mapPointIndex = 0, iend = edgesMonos.size();
         mapPointIndex < iend;
         mapPointIndex++)
    {
        vs_graphs::core::EdgeSE3ProjectXYZ *e = edgesMonos[mapPointIndex];
        MapPoint *p_mapPoint = mapPointEdgeMonos[mapPointIndex];

        bool mapPointIsBad3{};
        if (p_mapPoint->isBad(mapPointIsBad3) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad3)
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive())
        {
            KeyFrame *p_adjustKeyFrame = edgeKeyFrameMonos[mapPointIndex];
            vToErase.push_back(std::make_pair(p_adjustKeyFrame, p_mapPoint));
            wrongObservationKeyFrame[p_adjustKeyFrame->id]++;
            badMonoMapPoint++;

            erasedMapPoints.insert(p_mapPoint);
            erasedKeyFrames.insert(p_adjustKeyFrame);
        }
    }

    for (size_t mapPointIndex = 0, iend = edgesStereos.size();
         mapPointIndex < iend;
         mapPointIndex++)
    {
        g2o::EdgeStereoSE3ProjectXYZ *e = edgesStereos[mapPointIndex];
        MapPoint *p_mapPoint            = mapPointEdgeStereos[mapPointIndex];

        bool mapPointIsBad4{};
        if (p_mapPoint->isBad(mapPointIsBad4) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad4)
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive())
        {
            KeyFrame *p_adjustKeyFrame = edgeKeyFrameStereos[mapPointIndex];
            vToErase.push_back(std::make_pair(p_adjustKeyFrame, p_mapPoint));
            wrongObservationKeyFrame[p_adjustKeyFrame->id]++;
            badStereoMapPoint++;

            erasedMapPoints.insert(p_mapPoint);
            erasedKeyFrames.insert(p_adjustKeyFrame);
        }
    }

    if (Verbose::printMess(
            "[BA]: Second optimization, there are " +
                std::to_string(badMonoMapPoint) + " monocular and " +
                std::to_string(badStereoMapPoint) + " sterero bad edges",
            Verbose::VERBOSITY_DEBUG) != VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // Get Map Mutex
    Map *p_mainKeyFrameMap = nullptr;
    if (p_mainKeyFrame_in->getMap(p_mainKeyFrameMap) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::unique_lock<std::mutex> lock(p_mainKeyFrameMap->mapUpdateMutex);

    if (!vToErase.empty())
    {
        for (size_t mapPointIndex = 0; mapPointIndex < vToErase.size();
             mapPointIndex++)
        {
            KeyFrame *p_adjustKeyFrame = vToErase[mapPointIndex].first;
            MapPoint *p_viewMapPoint   = vToErase[mapPointIndex].second;
            if (p_adjustKeyFrame->eraseMapPointMatch(p_viewMapPoint) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPointMatch returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_viewMapPoint->eraseObservation(p_adjustKeyFrame) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseObservation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }
    for (unsigned int mapPointIndex = 0; mapPointIndex < mapPoints.size();
         ++mapPointIndex)
    {
        MapPoint *p_viewMapPoint = mapPoints[mapPointIndex];
        bool      viewMapPointIsBad4{};
        if (p_viewMapPoint->isBad(viewMapPointIsBad4) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (viewMapPointIsBad4)
            continue;

        std::map<KeyFrame *, std::tuple<int, int>> observations{};
        if (p_viewMapPoint->getObservations(observations) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (std::map<KeyFrame *, std::tuple<int, int>>::const_iterator mit =
                 observations.begin();
             mit != observations.end();
             mit++)
        {
            KeyFrame *p_keyFrame = mit->first;
            bool      keyFrameIsBad2{};
            if (p_keyFrame->isBad(keyFrameIsBad2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            MapPoint *p_keyFrameMapPoint2 = nullptr;
            if (!(keyFrameIsBad2 || p_keyFrame->id > maximumKeyFrameId ||
                  p_keyFrame->baLocalKeyFrameId != p_mainKeyFrame_in->id) &&
                p_keyFrame->getMapPoint(std::get<0>(mit->second),
                                        p_keyFrameMapPoint2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (keyFrameIsBad2 || p_keyFrame->id > maximumKeyFrameId ||
                p_keyFrame->baLocalKeyFrameId != p_mainKeyFrame_in->id ||
                !p_keyFrameMapPoint2)
                continue;

            if (p_keyFrame->uRight[std::get<0>(mit->second)] < 0) // Monocular
            {
                observationFinalKeyFrames[p_keyFrame]++;
            }
            else // RGBD or Stereo
            {
                observationFinalKeyFrames[p_keyFrame]++;
            }
        }
    }

    // Recover optimized data
    // Keyframes
    for (KeyFrame *p_adjustKeyFrame : adjustKeyFrames_in)
    {
        bool adjustKeyFrameIsBad3{};
        if (p_adjustKeyFrame->isBad(adjustKeyFrameIsBad3) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (adjustKeyFrameIsBad3)
            continue;

        g2o::VertexSE3Expmap *p_se3Vertex = static_cast<g2o::VertexSE3Expmap *>(
            optimizer.vertex(p_adjustKeyFrame->id));
        g2o::SE3Quat poseEstimate = p_se3Vertex->estimate();
        Sophus::SE3f Tiw(poseEstimate.rotation().cast<float>(),
                         poseEstimate.translation().cast<float>());

        int monoBadPointCount = 0, monoOptPointCount = 0;
        int stereoBadPointCount = 0, stereoOptPointCount = 0;
        std::vector<MapPoint *> monoMapPointsOpts, stereoMapPointsOpts;
        std::vector<MapPoint *> monoMapPointsBads, stereoMapPointsBads;

        for (size_t mapPointIndex = 0, iend = edgesMonos.size();
             mapPointIndex < iend;
             mapPointIndex++)
        {
            vs_graphs::core::EdgeSE3ProjectXYZ *e = edgesMonos[mapPointIndex];
            MapPoint *p_mapPoint     = mapPointEdgeMonos[mapPointIndex];
            KeyFrame *p_keyFrameEdge = nullptr;
            if (edgeSourceKeyFrame(edgeKeyFrameMonos,
                                   mapPointIndex,
                                   p_keyFrameEdge) !=
                OptimizerEdgeLookupStatus::OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: edgeSourceKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_keyFrameEdge == nullptr || p_adjustKeyFrame != p_keyFrameEdge)
            {
                continue;
            }

            bool mapPointIsBad5{};
            if (p_mapPoint->isBad(mapPointIsBad5) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (mapPointIsBad5)
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                monoBadPointCount++;
                monoMapPointsBads.push_back(p_mapPoint);
            }
            else
            {
                monoOptPointCount++;
                monoMapPointsOpts.push_back(p_mapPoint);
            }
        }

        for (size_t mapPointIndex = 0, iend = edgesStereos.size();
             mapPointIndex < iend;
             mapPointIndex++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = edgesStereos[mapPointIndex];
            MapPoint *p_mapPoint     = mapPointEdgeStereos[mapPointIndex];
            KeyFrame *p_keyFrameEdge = nullptr;
            if (edgeSourceKeyFrame(edgeKeyFrameStereos,
                                   mapPointIndex,
                                   p_keyFrameEdge) !=
                OptimizerEdgeLookupStatus::OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: edgeSourceKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (p_keyFrameEdge == nullptr || p_adjustKeyFrame != p_keyFrameEdge)
            {
                continue;
            }

            bool mapPointIsBad6{};
            if (p_mapPoint->isBad(mapPointIsBad6) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (mapPointIsBad6)
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                stereoBadPointCount++;
                stereoMapPointsBads.push_back(p_mapPoint);
            }
            else
            {
                stereoOptPointCount++;
                stereoMapPointsOpts.push_back(p_mapPoint);
            }
        }

        if (p_adjustKeyFrame->setPose(Tiw) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    // Points
    for (MapPoint *p_viewMapPoint : mapPoints)
    {
        bool viewMapPointIsBad5{};
        if (p_viewMapPoint->isBad(viewMapPointIsBad5) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (viewMapPointIsBad5)
            continue;

        g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<g2o::VertexSBAPointXYZ *>(
                optimizer.vertex(p_viewMapPoint->id + maximumKeyFrameId + 1));
        if (p_viewMapPoint->setWorldPos(
                p_pointVertex->estimate().cast<float>()) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_viewMapPoint->updateNormalAndDepth() !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateNormalAndDepth returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
