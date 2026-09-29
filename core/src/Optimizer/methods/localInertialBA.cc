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

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void Optimizer::localInertialBA(KeyFrame *p_keyFrame_inout,
                                bool     *p_pbStopFlag_in,
                                Map      *p_map_inout,
                                int      &fixedKeyFrameCount_in,
                                int      &optKeyFrameCount_in,
                                int      &mapPointCount_in,
                                int      &edgeCount_in,
                                bool      isLargeWindow_in,
                                bool      isRecentlyInitialized_in)
{
    Map *p_currentMap = nullptr;
    if (p_keyFrame_inout->getMap(p_currentMap) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    int maximumOpt                 = 10;
    int optimizationIterationCount = 10;
    if (isLargeWindow_in)
    {
        maximumOpt                 = 25;
        optimizationIterationCount = 4;
    }
    unsigned long currentMapKeyFrameCount{};
    if (p_currentMap->getKeyFrameCount(currentMapKeyFrameCount) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKeyFrameCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const int Nd = std::min((int)currentMapKeyFrameCount - 2, maximumOpt);
    const unsigned long maximumKeyFrameId = p_keyFrame_inout->id;

    std::vector<KeyFrame *> optimizableKeyFrames;
    std::vector<KeyFrame *> neighborsKeyFrames{};
    if (p_keyFrame_inout->getVectorCovisibleKeyFrames(neighborsKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getVectorCovisibleKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    list<KeyFrame *> optVisKeyFrames;

    optimizableKeyFrames.reserve(Nd);
    optimizableKeyFrames.push_back(p_keyFrame_inout);
    p_keyFrame_inout->baLocalKeyFrameId = p_keyFrame_inout->id;
    for (int neighborIndex = 1; neighborIndex < Nd; neighborIndex++)
    {
        if (optimizableKeyFrames.back()->p_prevKF)
        {
            optimizableKeyFrames.push_back(
                optimizableKeyFrames.back()->p_prevKF);
            optimizableKeyFrames.back()->baLocalKeyFrameId =
                p_keyFrame_inout->id;
        }
        else
            break;
    }

    int N = optimizableKeyFrames.size();

    // Optimizable points seen by temporal optimizable keyframes
    list<MapPoint *> localMapPointList;
    for (int neighborIndex = 0; neighborIndex < N; neighborIndex++)
    {
        std::vector<MapPoint *> mapPoints{};
        if (optimizableKeyFrames[neighborIndex]->getMapPointMatches(
                mapPoints) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointMatches returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (vector<MapPoint *>::iterator vit  = mapPoints.begin(),
                                          vend = mapPoints.end();
             vit != vend;
             vit++)
        {
            MapPoint *p_mapPoint = *vit;
            if (p_mapPoint)
            {
                bool mapPointIsBad{};
                if (p_mapPoint->isBad(mapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!mapPointIsBad)
                {
                    if (p_mapPoint->baLocalKeyFrameId != p_keyFrame_inout->id)
                    {
                        localMapPointList.push_back(p_mapPoint);
                        p_mapPoint->baLocalKeyFrameId = p_keyFrame_inout->id;
                    }
                }
            }
        }
    }

    // Fixed Keyframe: First frame previous KF to optimization window)
    list<KeyFrame *> fixedKeyFrames;
    if (optimizableKeyFrames.back()->p_prevKF)
    {
        fixedKeyFrames.push_back(optimizableKeyFrames.back()->p_prevKF);
        optimizableKeyFrames.back()->p_prevKF->baFixedKeyFrameId =
            p_keyFrame_inout->id;
    }
    else
    {
        optimizableKeyFrames.back()->baLocalKeyFrameId = 0;
        optimizableKeyFrames.back()->baFixedKeyFrameId = p_keyFrame_inout->id;
        fixedKeyFrames.push_back(optimizableKeyFrames.back());
        optimizableKeyFrames.pop_back();
    }

    // Optimizable visual KFs
    const int maximumCovisibleKeyFrameCount = 0;
    for (int neighborIndex = 0, iend = neighborsKeyFrames.size();
         neighborIndex < iend;
         neighborIndex++)
    {
        if (optVisKeyFrames.size() >= maximumCovisibleKeyFrameCount)
            break;

        KeyFrame *p_keyFrame = neighborsKeyFrames[neighborIndex];
        if (p_keyFrame->baLocalKeyFrameId == p_keyFrame_inout->id ||
            p_keyFrame->baFixedKeyFrameId == p_keyFrame_inout->id)
            continue;
        p_keyFrame->baLocalKeyFrameId = p_keyFrame_inout->id;
        bool keyFrameIsBad{};
        if (p_keyFrame->isBad(keyFrameIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Map *p_keyFrameMap = nullptr;
        if ((!keyFrameIsBad) && p_keyFrame->getMap(p_keyFrameMap) !=
                                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!keyFrameIsBad && p_keyFrameMap == p_currentMap)
        {
            optVisKeyFrames.push_back(p_keyFrame);

            std::vector<MapPoint *> mapPoints{};
            if (p_keyFrame->getMapPointMatches(mapPoints) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPointMatches returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (vector<MapPoint *>::iterator vit  = mapPoints.begin(),
                                              vend = mapPoints.end();
                 vit != vend;
                 vit++)
            {
                MapPoint *p_mapPoint = *vit;
                if (p_mapPoint)
                {
                    bool mapPointIsBad2{};
                    if (p_mapPoint->isBad(mapPointIsBad2) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (!mapPointIsBad2)
                    {
                        if (p_mapPoint->baLocalKeyFrameId !=
                            p_keyFrame_inout->id)
                        {
                            localMapPointList.push_back(p_mapPoint);
                            p_mapPoint->baLocalKeyFrameId =
                                p_keyFrame_inout->id;
                        }
                    }
                }
            }
        }
    }

    // Fixed KFs which are not covisible optimizable
    const int maximumFixKeyFrame = 200;

    for (list<MapPoint *>::iterator lit  = localMapPointList.begin(),
                                    lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        std::map<KeyFrame *, std::tuple<int, int>> observations{};
        if ((*lit)->getObservations(observations) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (map<KeyFrame *, tuple<int, int>>::iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            KeyFrame *p_keyFrame = mit->first;

            if (p_keyFrame->baLocalKeyFrameId != p_keyFrame_inout->id &&
                p_keyFrame->baFixedKeyFrameId != p_keyFrame_inout->id)
            {
                p_keyFrame->baFixedKeyFrameId = p_keyFrame_inout->id;
                bool keyFrameIsBad2{};
                if (p_keyFrame->isBad(keyFrameIsBad2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!keyFrameIsBad2)
                {
                    fixedKeyFrames.push_back(p_keyFrame);
                    break;
                }
            }
        }
        if (fixedKeyFrames.size() >= maximumFixKeyFrame)
            break;
    }

    // Setup optimizer
    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver;
    p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(p_linearSolver);

    if (isLargeWindow_in)
    {
        g2o::OptimizationAlgorithmLevenberg *p_solver =
            new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        p_solver->setUserLambdaInit(
            1e-2); // to avoid iterating for finding optimal lambda
        optimizer.setAlgorithm(p_solver);
    }
    else
    {
        g2o::OptimizationAlgorithmLevenberg *p_solver =
            new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        p_solver->setUserLambdaInit(1e0);
        optimizer.setAlgorithm(p_solver);
    }

    // Set Local temporal KeyFrame vertices
    N = optimizableKeyFrames.size();
    for (int neighborIndex = 0; neighborIndex < N; neighborIndex++)
    {
        KeyFrame *p_keyFrame = optimizableKeyFrames[neighborIndex];

        VertexPose *p_poseVertex = new VertexPose(p_keyFrame);
        p_poseVertex->setId(p_keyFrame->id);
        p_poseVertex->setFixed(false);
        optimizer.addVertex(p_poseVertex);

        if (p_keyFrame->isImu)
        {
            VertexVelocity *p_velocityVertex = new VertexVelocity(p_keyFrame);
            p_velocityVertex->setId(maximumKeyFrameId + 3 * (p_keyFrame->id) +
                                    1);
            p_velocityVertex->setFixed(false);
            optimizer.addVertex(p_velocityVertex);
            VertexGyroBias *p_gyroBiasVertex = new VertexGyroBias(p_keyFrame);
            p_gyroBiasVertex->setId(maximumKeyFrameId + 3 * (p_keyFrame->id) +
                                    2);
            p_gyroBiasVertex->setFixed(false);
            optimizer.addVertex(p_gyroBiasVertex);
            VertexAccBias *p_accelerometerBiasVertex =
                new VertexAccBias(p_keyFrame);
            p_accelerometerBiasVertex->setId(maximumKeyFrameId +
                                             3 * (p_keyFrame->id) + 3);
            p_accelerometerBiasVertex->setFixed(false);
            optimizer.addVertex(p_accelerometerBiasVertex);
        }
    }

    // Set Local visual KeyFrame vertices
    for (list<KeyFrame *>::iterator
             optimizedVisualKeyFrameIt = optVisKeyFrames.begin(),
             itEnd                     = optVisKeyFrames.end();
         optimizedVisualKeyFrameIt != itEnd;
         optimizedVisualKeyFrameIt++)
    {
        KeyFrame   *p_keyFrame   = *optimizedVisualKeyFrameIt;
        VertexPose *p_poseVertex = new VertexPose(p_keyFrame);
        p_poseVertex->setId(p_keyFrame->id);
        p_poseVertex->setFixed(false);
        optimizer.addVertex(p_poseVertex);
    }

    // Set Fixed KeyFrame vertices
    for (list<KeyFrame *>::iterator lit  = fixedKeyFrames.begin(),
                                    lend = fixedKeyFrames.end();
         lit != lend;
         lit++)
    {
        KeyFrame   *p_keyFrame   = *lit;
        VertexPose *p_poseVertex = new VertexPose(p_keyFrame);
        p_poseVertex->setId(p_keyFrame->id);
        p_poseVertex->setFixed(true);
        optimizer.addVertex(p_poseVertex);

        if (p_keyFrame->isImu) // This should be done only for keyframe just
                               // before temporal window
        {
            VertexVelocity *p_velocityVertex = new VertexVelocity(p_keyFrame);
            p_velocityVertex->setId(maximumKeyFrameId + 3 * (p_keyFrame->id) +
                                    1);
            p_velocityVertex->setFixed(true);
            optimizer.addVertex(p_velocityVertex);
            VertexGyroBias *p_gyroBiasVertex = new VertexGyroBias(p_keyFrame);
            p_gyroBiasVertex->setId(maximumKeyFrameId + 3 * (p_keyFrame->id) +
                                    2);
            p_gyroBiasVertex->setFixed(true);
            optimizer.addVertex(p_gyroBiasVertex);
            VertexAccBias *p_accelerometerBiasVertex =
                new VertexAccBias(p_keyFrame);
            p_accelerometerBiasVertex->setId(maximumKeyFrameId +
                                             3 * (p_keyFrame->id) + 3);
            p_accelerometerBiasVertex->setFixed(true);
            optimizer.addVertex(p_accelerometerBiasVertex);
        }
    }

    // Create intertial constraints
    vector<EdgeInertial *> vei(N, (EdgeInertial *)nullptr);
    vector<EdgeGyroRW *>   vegr(N, (EdgeGyroRW *)nullptr);
    vector<EdgeAccRW *>    vear(N, (EdgeAccRW *)nullptr);

    for (int neighborIndex = 0; neighborIndex < N; neighborIndex++)
    {
        KeyFrame *p_keyFrame = optimizableKeyFrames[neighborIndex];

        if (!p_keyFrame->p_prevKF)
        {
            cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
            continue;
        }
        if (p_keyFrame->isImu && p_keyFrame->p_prevKF->isImu &&
            p_keyFrame->p_imuPreintegrated)
        {
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
                maximumKeyFrameId + 3 * (p_keyFrame->p_prevKF->id) + 1);
            g2o::HyperGraph::Vertex *p_firstGyroBiasVertex = optimizer.vertex(
                maximumKeyFrameId + 3 * (p_keyFrame->p_prevKF->id) + 2);
            g2o::HyperGraph::Vertex *p_firstAccelerometerBiasVertex =
                optimizer.vertex(maximumKeyFrameId +
                                 3 * (p_keyFrame->p_prevKF->id) + 3);
            g2o::HyperGraph::Vertex *p_secondPoseVertex =
                optimizer.vertex(p_keyFrame->id);
            g2o::HyperGraph::Vertex *p_secondVelocityVertex =
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 1);
            g2o::HyperGraph::Vertex *p_secondGyroBiasVertex =
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 2);
            g2o::HyperGraph::Vertex *p_secondAccelerometerBiasVertex =
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 3);

            if (!p_firstPoseVertex || !p_firstVelocityVertex ||
                !p_firstGyroBiasVertex || !p_firstAccelerometerBiasVertex ||
                !p_secondPoseVertex || !p_secondVelocityVertex ||
                !p_secondGyroBiasVertex || !p_secondAccelerometerBiasVertex)
            {
                cerr << "Error " << p_firstPoseVertex << ", "
                     << p_firstVelocityVertex << ", " << p_firstGyroBiasVertex
                     << ", " << p_firstAccelerometerBiasVertex << ", "
                     << p_secondPoseVertex << ", " << p_secondVelocityVertex
                     << ", " << p_secondGyroBiasVertex << ", "
                     << p_secondAccelerometerBiasVertex << endl;
                continue;
            }

            vei[neighborIndex] =
                new EdgeInertial(p_keyFrame->p_imuPreintegrated);

            vei[neighborIndex]->setVertex(
                0,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                    p_firstPoseVertex));
            vei[neighborIndex]->setVertex(
                1,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                    p_firstVelocityVertex));
            vei[neighborIndex]->setVertex(
                2,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                    p_firstGyroBiasVertex));
            vei[neighborIndex]->setVertex(
                3,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                    p_firstAccelerometerBiasVertex));
            vei[neighborIndex]->setVertex(
                4,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                    p_secondPoseVertex));
            vei[neighborIndex]->setVertex(
                5,
                dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                    p_secondVelocityVertex));

            if (neighborIndex == N - 1 || isRecentlyInitialized_in)
            {
                // All inertial residuals are included without robust cost
                // function, but not that one linking the last optimizable
                // keyframe inside of the local window and the first fixed
                // keyframe out. The information matrix for this measurement is
                // also downweighted. This is done to avoid accumulating error
                // due to fixing variables.
                g2o::RobustKernelHuber *p_rki = new g2o::RobustKernelHuber;
                vei[neighborIndex]->setRobustKernel(p_rki);
                if (neighborIndex == N - 1)
                    vei[neighborIndex]->setInformation(
                        vei[neighborIndex]->information() * 1e-2);
                p_rki->setDelta(sqrt(16.92));
            }
            optimizer.addEdge(vei[neighborIndex]);

            vegr[neighborIndex] = new EdgeGyroRW();
            vegr[neighborIndex]->setVertex(0, p_firstGyroBiasVertex);
            vegr[neighborIndex]->setVertex(1, p_secondGyroBiasVertex);
            Eigen::Matrix3d informationG =
                p_keyFrame->p_imuPreintegrated->C.block<3, 3>(9, 9)
                    .cast<double>()
                    .inverse();
            vegr[neighborIndex]->setInformation(informationG);
            optimizer.addEdge(vegr[neighborIndex]);

            vear[neighborIndex] = new EdgeAccRW();
            vear[neighborIndex]->setVertex(0, p_firstAccelerometerBiasVertex);
            vear[neighborIndex]->setVertex(1, p_secondAccelerometerBiasVertex);
            Eigen::Matrix3d informationA =
                p_keyFrame->p_imuPreintegrated->C.block<3, 3>(12, 12)
                    .cast<double>()
                    .inverse();
            vear[neighborIndex]->setInformation(informationA);

            optimizer.addEdge(vear[neighborIndex]);
        }
        else
            cout << "ERROR building inertial edge" << endl;
    }

    // Set MapPoint vertices
    const int expectedSizeCount =
        (N + fixedKeyFrames.size()) * localMapPointList.size();

    // Mono
    vector<EdgeMono *> edgesMonos;
    edgesMonos.reserve(expectedSizeCount);

    vector<KeyFrame *> edgeKeyFrameMonos;
    edgeKeyFrameMonos.reserve(expectedSizeCount);

    vector<MapPoint *> mapPointEdgeMonos;
    mapPointEdgeMonos.reserve(expectedSizeCount);

    // Stereo
    vector<EdgeStereo *> edgesStereos;
    edgesStereos.reserve(expectedSizeCount);

    vector<KeyFrame *> edgeKeyFrameStereos;
    edgeKeyFrameStereos.reserve(expectedSizeCount);

    vector<MapPoint *> mapPointEdgeStereos;
    mapPointEdgeStereos.reserve(expectedSizeCount);

    const float thresholdHuberMono   = sqrt(5.991);
    const float chi2Mono2            = 5.991;
    const float thresholdHuberStereo = sqrt(7.815);
    const float chi2Stereo2          = 7.815;

    const unsigned long initialMapPointId = maximumKeyFrameId * 5;

    map<int, int> visibleEdgeCounts;
    for (int neighborIndex = 0; neighborIndex < N; neighborIndex++)
    {
        KeyFrame *p_keyFrame              = optimizableKeyFrames[neighborIndex];
        visibleEdgeCounts[p_keyFrame->id] = 0;
    }
    for (list<KeyFrame *>::iterator lit  = fixedKeyFrames.begin(),
                                    lend = fixedKeyFrames.end();
         lit != lend;
         lit++)
    {
        visibleEdgeCounts[(*lit)->id] = 0;
    }

    for (list<MapPoint *>::iterator lit  = localMapPointList.begin(),
                                    lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        MapPoint               *p_mapPoint    = *lit;
        g2o::VertexSBAPointXYZ *p_pointVertex = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f         mapPointWorldPos{};
        if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        p_pointVertex->setEstimate(mapPointWorldPos.cast<double>());

        unsigned long id = p_mapPoint->id + initialMapPointId + 1;
        p_pointVertex->setId(id);
        p_pointVertex->setMarginalized(true);
        optimizer.addVertex(p_pointVertex);
        std::map<KeyFrame *, std::tuple<int, int>> observations{};
        if (p_mapPoint->getObservations(observations) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getObservations returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        // Create visual constraints
        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            KeyFrame *p_keyFrame = mit->first;

            if (p_keyFrame->baLocalKeyFrameId != p_keyFrame_inout->id &&
                p_keyFrame->baFixedKeyFrameId != p_keyFrame_inout->id)
                continue;

            bool keyFrameIsBad3{};
            if (p_keyFrame->isBad(keyFrameIsBad3) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Map *p_keyFrameMap2 = nullptr;
            if ((!keyFrameIsBad3) &&
                p_keyFrame->getMap(p_keyFrameMap2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!keyFrameIsBad3 && p_keyFrameMap2 == p_currentMap)
            {
                const int leftIndex = get<0>(mit->second);

                cv::KeyPoint keyPointUn;

                // Monocular left observation
                if (leftIndex != -1 && p_keyFrame->uRight[leftIndex] < 0)
                {
                    visibleEdgeCounts[p_keyFrame->id]++;

                    keyPointUn = p_keyFrame->keyPointsUndistorted[leftIndex];
                    Eigen::Matrix<double, 2, 1> observation;
                    observation << keyPointUn.pt.x, keyPointUn.pt.y;

                    EdgeMono *e = new EdgeMono(0);

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(p_keyFrame->id)));
                    e->setMeasurement(observation);

                    // Add here uncerteinty
                    const float unc2 =
                        p_keyFrame->p_camera->uncertainty2(observation);

                    const float &invSigma2 =
                        p_keyFrame->invLevelSigmaSquared[keyPointUn.octave] /
                        unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(thresholdHuberMono);

                    optimizer.addEdge(e);
                    edgesMonos.push_back(e);
                    edgeKeyFrameMonos.push_back(p_keyFrame);
                    mapPointEdgeMonos.push_back(p_mapPoint);
                }
                // Stereo-observation
                else if (leftIndex != -1) // Stereo observation
                {
                    keyPointUn = p_keyFrame->keyPointsUndistorted[leftIndex];
                    visibleEdgeCounts[p_keyFrame->id]++;

                    const float rightKeyPointU = p_keyFrame->uRight[leftIndex];
                    Eigen::Matrix<double, 3, 1> observation;
                    observation << keyPointUn.pt.x, keyPointUn.pt.y,
                        rightKeyPointU;

                    EdgeStereo *e = new EdgeStereo(0);

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(p_keyFrame->id)));
                    e->setMeasurement(observation);

                    // Add here uncerteinty
                    const float unc2 =
                        p_keyFrame->p_camera->uncertainty2(observation.head(2));

                    const float &invSigma2 =
                        p_keyFrame->invLevelSigmaSquared[keyPointUn.octave] /
                        unc2;
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(thresholdHuberStereo);

                    optimizer.addEdge(e);
                    edgesStereos.push_back(e);
                    edgeKeyFrameStereos.push_back(p_keyFrame);
                    mapPointEdgeStereos.push_back(p_mapPoint);
                }

                // Monocular right observation
                if (p_keyFrame->p_camera2)
                {
                    int rightIndex = get<1>(mit->second);

                    if (rightIndex != -1 &&
                        rightIndex < (int)p_keyFrame->keyPointsRight.size())
                    {
                        rightIndex -= p_keyFrame->leftKeyPointCount;
                        visibleEdgeCounts[p_keyFrame->id]++;

                        Eigen::Matrix<double, 2, 1> observation;
                        cv::KeyPoint                keyPoint =
                            p_keyFrame->keyPointsRight[rightIndex];
                        observation << keyPoint.pt.x, keyPoint.pt.y;

                        EdgeMono *e = new EdgeMono(1);

                        e->setVertex(
                            0,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(id)));
                        e->setVertex(
                            1,
                            dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                optimizer.vertex(p_keyFrame->id)));
                        e->setMeasurement(observation);

                        // Add here uncerteinty
                        const float unc2 =
                            p_keyFrame->p_camera->uncertainty2(observation);

                        const float &invSigma2 =
                            p_keyFrame
                                ->invLevelSigmaSquared[keyPointUn.octave] /
                            unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() *
                                          invSigma2);

                        g2o::RobustKernelHuber *p_robustKernel =
                            new g2o::RobustKernelHuber;
                        e->setRobustKernel(p_robustKernel);
                        p_robustKernel->setDelta(thresholdHuberMono);

                        optimizer.addEdge(e);
                        edgesMonos.push_back(e);
                        edgeKeyFrameMonos.push_back(p_keyFrame);
                        mapPointEdgeMonos.push_back(p_mapPoint);
                    }
                }
            }
        }
    }

    // cout << "Total map points: " << localMapPointList.size() << endl;
    for (map<int, int>::iterator mit  = visibleEdgeCounts.begin(),
                                 mend = visibleEdgeCounts.end();
         mit != mend;
         mit++)
    {
        assert(mit->second >= 3);
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    float error = optimizer.activeRobustChi2();
    optimizer.optimize(optimizationIterationCount); // Originally to 2
    float errorEnd = optimizer.activeRobustChi2();
    if (p_pbStopFlag_in)
        optimizer.setForceStopFlag(p_pbStopFlag_in);

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(edgesMonos.size() + edgesStereos.size());

    // Check inlier observations
    // Mono
    for (size_t neighborIndex = 0, iend = edgesMonos.size();
         neighborIndex < iend;
         neighborIndex++)
    {
        EdgeMono *e            = edgesMonos[neighborIndex];
        MapPoint *p_mapPoint   = mapPointEdgeMonos[neighborIndex];
        bool      isClosePoint = p_mapPoint->trackDepth < 10.f;

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

        if ((e->chi2() > chi2Mono2 && !isClosePoint) ||
            (e->chi2() > 1.5f * chi2Mono2 && isClosePoint) ||
            !e->isDepthPositive())
        {
            KeyFrame *p_keyFrame = edgeKeyFrameMonos[neighborIndex];
            vToErase.push_back(make_pair(p_keyFrame, p_mapPoint));
        }
    }

    // Stereo
    for (size_t neighborIndex = 0, iend = edgesStereos.size();
         neighborIndex < iend;
         neighborIndex++)
    {
        EdgeStereo *e          = edgesStereos[neighborIndex];
        MapPoint   *p_mapPoint = mapPointEdgeStereos[neighborIndex];

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

        if (e->chi2() > chi2Stereo2)
        {
            KeyFrame *p_keyFrame = edgeKeyFrameStereos[neighborIndex];
            vToErase.push_back(make_pair(p_keyFrame, p_mapPoint));
        }
    }

    // Get Map Mutex and erase outliers
    unique_lock<mutex> lock(p_map_inout->mapUpdateMutex);

    // TODO: Some convergence problems have been detected here
    if ((2 * error < errorEnd || isnan(error) || isnan(errorEnd)) &&
        !isLargeWindow_in) // bGN)
    {
        cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
        return;
    }

    if (!vToErase.empty())
    {
        for (size_t neighborIndex = 0; neighborIndex < vToErase.size();
             neighborIndex++)
        {
            KeyFrame *p_keyFrame        = vToErase[neighborIndex].first;
            MapPoint *p_mapPointToErase = vToErase[neighborIndex].second;
            if (p_keyFrame->eraseMapPointMatch(p_mapPointToErase) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPointMatch returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapPointToErase->eraseObservation(p_keyFrame) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseObservation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (list<KeyFrame *>::iterator lit  = fixedKeyFrames.begin(),
                                    lend = fixedKeyFrames.end();
         lit != lend;
         lit++)
        (*lit)->baFixedKeyFrameId = 0;

    // Recover optimized data
    // Local temporal Keyframes
    N = optimizableKeyFrames.size();
    for (int neighborIndex = 0; neighborIndex < N; neighborIndex++)
    {
        KeyFrame *p_keyFrame = optimizableKeyFrames[neighborIndex];

        VertexPose *p_poseVertex =
            static_cast<VertexPose *>(optimizer.vertex(p_keyFrame->id));
        Sophus::SE3f Tcw(p_poseVertex->estimate().Rcw[0].cast<float>(),
                         p_poseVertex->estimate().tcw[0].cast<float>());
        if (p_keyFrame->setPose(Tcw) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_keyFrame->baLocalKeyFrameId = 0;

        if (p_keyFrame->isImu)
        {
            VertexVelocity *p_velocityVertex = static_cast<VertexVelocity *>(
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 1));
            if (p_keyFrame->setVelocity(
                    p_velocityVertex->estimate().cast<float>()) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            VertexGyroBias *p_gyroBiasVertex = static_cast<VertexGyroBias *>(
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 2));
            VertexAccBias *p_accelerometerBiasVertex =
                static_cast<VertexAccBias *>(optimizer.vertex(
                    maximumKeyFrameId + 3 * (p_keyFrame->id) + 3));
            Vector6d b;
            b << p_gyroBiasVertex->estimate(),
                p_accelerometerBiasVertex->estimate();
            if (p_keyFrame->setNewBias(
                    IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2])) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNewBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    // Local visual KeyFrame
    for (list<KeyFrame *>::iterator
             optimizedVisualKeyFrameIt = optVisKeyFrames.begin(),
             itEnd                     = optVisKeyFrames.end();
         optimizedVisualKeyFrameIt != itEnd;
         optimizedVisualKeyFrameIt++)
    {
        KeyFrame   *p_keyFrame = *optimizedVisualKeyFrameIt;
        VertexPose *p_poseVertex =
            static_cast<VertexPose *>(optimizer.vertex(p_keyFrame->id));
        Sophus::SE3f Tcw(p_poseVertex->estimate().Rcw[0].cast<float>(),
                         p_poseVertex->estimate().tcw[0].cast<float>());
        if (p_keyFrame->setPose(Tcw) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_keyFrame->baLocalKeyFrameId = 0;
    }

    // Points
    for (list<MapPoint *>::iterator lit  = localMapPointList.begin(),
                                    lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        MapPoint               *p_mapPoint = *lit;
        g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<g2o::VertexSBAPointXYZ *>(
                optimizer.vertex(p_mapPoint->id + initialMapPointId + 1));
        if (p_mapPoint->setWorldPos(p_pointVertex->estimate().cast<float>()) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mapPoint->updateNormalAndDepth() !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateNormalAndDepth returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    if (p_map_inout->increaseChangeIndex() != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: increaseChangeIndex returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

} // namespace core
} // namespace vs_graphs
