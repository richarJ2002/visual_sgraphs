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

#include "../private_functions.h"
#include "System.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void Optimizer::mergeInertialBA(KeyFrame *p_currentKeyFrame_inout,
                                KeyFrame *p_mergeKeyFrame_inout,
                                bool     *p_pbStopFlag_in,
                                Map      *p_map_inout,
                                LoopClosing::KeyFrameAndPose &corrPoses_inout)
{
    const int           Nd                = 6;
    const unsigned long maximumKeyFrameId = p_currentKeyFrame_inout->id;

    vector<KeyFrame *> optimizableKeyFrames;
    optimizableKeyFrames.reserve(2 * Nd);

    // For cov KFS, inertial parameters are not optimized
    const int          maximumCovisibleKeyFrameCount = 30;
    vector<KeyFrame *> optimizableCovisibleKeyFrames;
    optimizableCovisibleKeyFrames.reserve(maximumCovisibleKeyFrameCount);

    // Add sliding window for current KF
    optimizableKeyFrames.push_back(p_currentKeyFrame_inout);
    p_currentKeyFrame_inout->baLocalKeyFrameId = p_currentKeyFrame_inout->id;
    for (int i = 1; i < Nd; i++)
    {
        if (optimizableKeyFrames.back()->p_prevKF)
        {
            optimizableKeyFrames.push_back(
                optimizableKeyFrames.back()->p_prevKF);
            optimizableKeyFrames.back()->baLocalKeyFrameId =
                p_currentKeyFrame_inout->id;
        }
        else
            break;
    }

    list<KeyFrame *> fixedKeyFrames;
    if (optimizableKeyFrames.back()->p_prevKF)
    {
        optimizableCovisibleKeyFrames.push_back(
            optimizableKeyFrames.back()->p_prevKF);
        optimizableKeyFrames.back()->p_prevKF->baLocalKeyFrameId =
            p_currentKeyFrame_inout->id;
    }
    else
    {
        optimizableCovisibleKeyFrames.push_back(optimizableKeyFrames.back());
        optimizableKeyFrames.pop_back();
    }

    // Add temporal neighbours to merge KF (previous and next KFs)
    optimizableKeyFrames.push_back(p_mergeKeyFrame_inout);
    p_mergeKeyFrame_inout->baLocalKeyFrameId = p_currentKeyFrame_inout->id;

    // Previous KFs
    for (int i = 1; i < (Nd / 2); i++)
    {
        if (optimizableKeyFrames.back()->p_prevKF)
        {
            optimizableKeyFrames.push_back(
                optimizableKeyFrames.back()->p_prevKF);
            optimizableKeyFrames.back()->baLocalKeyFrameId =
                p_currentKeyFrame_inout->id;
        }
        else
            break;
    }

    // We fix just once the old map
    if (optimizableKeyFrames.back()->p_prevKF)
    {
        fixedKeyFrames.push_back(optimizableKeyFrames.back()->p_prevKF);
        optimizableKeyFrames.back()->p_prevKF->baFixedKeyFrameId =
            p_currentKeyFrame_inout->id;
    }
    else
    {
        optimizableKeyFrames.back()->baLocalKeyFrameId = 0;
        optimizableKeyFrames.back()->baFixedKeyFrameId =
            p_currentKeyFrame_inout->id;
        fixedKeyFrames.push_back(optimizableKeyFrames.back());
        optimizableKeyFrames.pop_back();
    }

    // Next KFs
    if (p_mergeKeyFrame_inout->p_nextKF)
    {
        optimizableKeyFrames.push_back(p_mergeKeyFrame_inout->p_nextKF);
        optimizableKeyFrames.back()->baLocalKeyFrameId =
            p_currentKeyFrame_inout->id;
    }

    while (optimizableKeyFrames.size() < (2 * Nd))
    {
        if (optimizableKeyFrames.back()->p_nextKF)
        {
            optimizableKeyFrames.push_back(
                optimizableKeyFrames.back()->p_nextKF);
            optimizableKeyFrames.back()->baLocalKeyFrameId =
                p_currentKeyFrame_inout->id;
        }
        else
            break;
    }

    int N = optimizableKeyFrames.size();

    // Optimizable points seen by optimizable keyframes
    list<MapPoint *>     localMapPointList;
    map<MapPoint *, int> localObservationCounts;
    for (int i = 0; i < N; i++)
    {
        vector<MapPoint *> mapPoints =
            optimizableKeyFrames[i]->getMapPointMatches();
        for (vector<MapPoint *>::iterator vit  = mapPoints.begin(),
                                          vend = mapPoints.end();
             vit != vend;
             vit++)
        {
            // Using mnBALocalForKF we avoid redundance here, one MP can not be
            // added several times to localMapPointList
            MapPoint *p_mapPoint = *vit;
            if (p_mapPoint)
                if (!p_mapPoint->isBad())
                {
                    if (p_mapPoint->baLocalKeyFrameId !=
                        p_currentKeyFrame_inout->id)
                    {
                        localObservationCounts[p_mapPoint] = 1;
                        localMapPointList.push_back(p_mapPoint);
                        p_mapPoint->baLocalKeyFrameId =
                            p_currentKeyFrame_inout->id;
                    }
                    else
                    {
                        localObservationCounts[p_mapPoint]++;
                    }
                }
        }
    }

    std::vector<std::pair<MapPoint *, int>> pairs;
    pairs.reserve(localObservationCounts.size());
    for (auto itr = localObservationCounts.begin();
         itr != localObservationCounts.end();
         ++itr)
        pairs.push_back(*itr);
    sort(pairs.begin(), pairs.end(), sortByVal);

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not
    // Local Keyframes
    int i = 0;
    for (vector<pair<MapPoint *, int>>::iterator lit  = pairs.begin(),
                                                 lend = pairs.end();
         lit != lend;
         lit++, i++)
    {
        map<KeyFrame *, tuple<int, int>> observations =
            lit->first->getObservations();
        if (i >= maximumCovisibleKeyFrameCount)
            break;
        for (map<KeyFrame *, tuple<int, int>>::iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            KeyFrame *p_keyFrame = mit->first;

            if (p_keyFrame->baLocalKeyFrameId != p_currentKeyFrame_inout->id &&
                p_keyFrame->baFixedKeyFrameId !=
                    p_currentKeyFrame_inout
                        ->id) // If optimizable or already included...
            {
                p_keyFrame->baLocalKeyFrameId = p_currentKeyFrame_inout->id;
                if (!p_keyFrame->isBad())
                {
                    optimizableCovisibleKeyFrames.push_back(p_keyFrame);
                    break;
                }
            }
        }
    }

    g2o::SparseOptimizer                 optimizer;
    g2o::BlockSolverX::LinearSolverType *p_linearSolver;
    p_linearSolver =
        new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(p_linearSolver);

    g2o::OptimizationAlgorithmLevenberg *p_solver =
        new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    p_solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(p_solver);
    optimizer.setVerbose(false);

    // Set Local KeyFrame vertices
    N = optimizableKeyFrames.size();
    for (int i = 0; i < N; i++)
    {
        KeyFrame *p_keyFrame = optimizableKeyFrames[i];

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

    // Set Local cov keyframes vertices
    int optimizableCovisibleCount = optimizableCovisibleKeyFrames.size();
    for (int i = 0; i < optimizableCovisibleCount; i++)
    {
        KeyFrame *p_keyFrame = optimizableCovisibleKeyFrames[i];

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

        if (p_keyFrame->isImu)
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
    for (int i = 0; i < N; i++)
    {
        // cout << "inserting inertial edge " << i << endl;
        KeyFrame *p_keyFrame = optimizableKeyFrames[i];

        if (!p_keyFrame->p_prevKF)
        {
            Verbose::printMess("NOT INERTIAL LINK TO PREVIOUS FRAME!!!!",
                               Verbose::VERBOSITY_NORMAL);
            continue;
        }
        if (p_keyFrame->isImu && p_keyFrame->p_prevKF->isImu &&
            p_keyFrame->p_imuPreintegrated)
        {
            p_keyFrame->p_imuPreintegrated->setNewBias(
                p_keyFrame->p_prevKF->getImuBias());
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

            vei[i] = new EdgeInertial(p_keyFrame->p_imuPreintegrated);

            vei[i]->setVertex(0,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_firstPoseVertex));
            vei[i]->setVertex(1,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_firstVelocityVertex));
            vei[i]->setVertex(2,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_firstGyroBiasVertex));
            vei[i]->setVertex(3,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_firstAccelerometerBiasVertex));
            vei[i]->setVertex(4,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_secondPoseVertex));
            vei[i]->setVertex(5,
                              dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                  p_secondVelocityVertex));

            // TODO Uncomment
            g2o::RobustKernelHuber *p_rki = new g2o::RobustKernelHuber;
            vei[i]->setRobustKernel(p_rki);
            p_rki->setDelta(sqrt(16.92));
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0, p_firstGyroBiasVertex);
            vegr[i]->setVertex(1, p_secondGyroBiasVertex);
            Eigen::Matrix3d informationG =
                p_keyFrame->p_imuPreintegrated->C.block<3, 3>(9, 9)
                    .cast<double>()
                    .inverse();
            vegr[i]->setInformation(informationG);
            optimizer.addEdge(vegr[i]);

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0, p_firstAccelerometerBiasVertex);
            vear[i]->setVertex(1, p_secondAccelerometerBiasVertex);
            Eigen::Matrix3d informationA =
                p_keyFrame->p_imuPreintegrated->C.block<3, 3>(12, 12)
                    .cast<double>()
                    .inverse();
            vear[i]->setInformation(informationA);
            optimizer.addEdge(vear[i]);
        }
        else
            Verbose::printMess("ERROR building inertial edge",
                               Verbose::VERBOSITY_NORMAL);
    }

    Verbose::printMess("end inserting inertial edges",
                       Verbose::VERBOSITY_NORMAL);

    // Set MapPoint vertices
    const int expectedSizeCount =
        (N + optimizableCovisibleCount + fixedKeyFrames.size()) *
        localMapPointList.size();

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

    for (list<MapPoint *>::iterator lit  = localMapPointList.begin(),
                                    lend = localMapPointList.end();
         lit != lend;
         lit++)
    {
        MapPoint *p_mapPoint = *lit;
        if (!p_mapPoint)
            continue;

        g2o::VertexSBAPointXYZ *p_pointVertex = new g2o::VertexSBAPointXYZ();
        p_pointVertex->setEstimate(p_mapPoint->getWorldPos().cast<double>());

        unsigned long id = p_mapPoint->id + initialMapPointId + 1;
        p_pointVertex->setId(id);
        p_pointVertex->setMarginalized(true);
        optimizer.addVertex(p_pointVertex);

        const map<KeyFrame *, tuple<int, int>> observations =
            p_mapPoint->getObservations();

        // Create visual constraints
        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                 mit  = observations.begin(),
                 mend = observations.end();
             mit != mend;
             mit++)
        {
            KeyFrame *p_keyFrame = mit->first;

            if (!p_keyFrame)
                continue;

            if ((p_keyFrame->baLocalKeyFrameId !=
                 p_currentKeyFrame_inout->id) &&
                (p_keyFrame->baFixedKeyFrameId != p_currentKeyFrame_inout->id))
                continue;

            if (p_keyFrame->id > maximumKeyFrameId)
            {
                continue;
            }

            if (optimizer.vertex(id) == nullptr ||
                optimizer.vertex(p_keyFrame->id) == nullptr)
                continue;

            if (!p_keyFrame->isBad())
            {
                const cv::KeyPoint &keyPointUn =
                    p_keyFrame->keyPointsUndistorted[get<0>(mit->second)];

                if (p_keyFrame->uRight[get<0>(mit->second)] <
                    0) // Monocular observation
                {
                    Eigen::Matrix<double, 2, 1> observation;
                    observation << keyPointUn.pt.x, keyPointUn.pt.y;

                    EdgeMono *e = new EdgeMono();
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
                    p_robustKernel->setDelta(thresholdHuberMono);
                    optimizer.addEdge(e);
                    edgesMonos.push_back(e);
                    edgeKeyFrameMonos.push_back(p_keyFrame);
                    mapPointEdgeMonos.push_back(p_mapPoint);
                }
                else // stereo observation
                {
                    const float rightKeyPointU =
                        p_keyFrame->uRight[get<0>(mit->second)];
                    Eigen::Matrix<double, 3, 1> observation;
                    observation << keyPointUn.pt.x, keyPointUn.pt.y,
                        rightKeyPointU;

                    EdgeStereo *e = new EdgeStereo();

                    e->setVertex(0,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(id)));
                    e->setVertex(1,
                                 dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                                     optimizer.vertex(p_keyFrame->id)));
                    e->setMeasurement(observation);
                    const float &invSigma2 =
                        p_keyFrame->invLevelSigmaSquared[keyPointUn.octave];
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
            }
        }
    }

    if (p_pbStopFlag_in)
        optimizer.setForceStopFlag(p_pbStopFlag_in);

    if (p_pbStopFlag_in)
        if (*p_pbStopFlag_in)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(8);

    vector<pair<KeyFrame *, MapPoint *>> vToErase;
    vToErase.reserve(edgesMonos.size() + edgesStereos.size());

    // Check inlier observations
    // Mono
    for (size_t i = 0, iend = edgesMonos.size(); i < iend; i++)
    {
        EdgeMono *e          = edgesMonos[i];
        MapPoint *p_mapPoint = mapPointEdgeMonos[i];

        if (p_mapPoint->isBad())
            continue;

        if (e->chi2() > chi2Mono2)
        {
            KeyFrame *p_keyFrame = edgeKeyFrameMonos[i];
            vToErase.push_back(make_pair(p_keyFrame, p_mapPoint));
        }
    }

    // Stereo
    for (size_t i = 0, iend = edgesStereos.size(); i < iend; i++)
    {
        EdgeStereo *e          = edgesStereos[i];
        MapPoint   *p_mapPoint = mapPointEdgeStereos[i];

        if (p_mapPoint->isBad())
            continue;

        if (e->chi2() > chi2Stereo2)
        {
            KeyFrame *p_keyFrame = edgeKeyFrameStereos[i];
            vToErase.push_back(make_pair(p_keyFrame, p_mapPoint));
        }
    }

    // Get Map Mutex and erase outliers
    unique_lock<mutex> lock(p_map_inout->mapUpdateMutex);
    if (!vToErase.empty())
    {
        for (size_t i = 0; i < vToErase.size(); i++)
        {
            KeyFrame *p_keyFrame        = vToErase[i].first;
            MapPoint *p_mapPointToErase = vToErase[i].second;
            p_keyFrame->eraseMapPointMatch(p_mapPointToErase);
            p_mapPointToErase->eraseObservation(p_keyFrame);
        }
    }

    // Recover optimized data
    // Keyframes
    for (int i = 0; i < N; i++)
    {
        KeyFrame *p_keyFrame = optimizableKeyFrames[i];

        VertexPose *p_poseVertex =
            static_cast<VertexPose *>(optimizer.vertex(p_keyFrame->id));
        Sophus::SE3f Tcw(p_poseVertex->estimate().Rcw[0].cast<float>(),
                         p_poseVertex->estimate().tcw[0].cast<float>());
        p_keyFrame->setPose(Tcw);

        Sophus::SE3d Tiw = p_keyFrame->getPose().cast<double>();
        g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
        corrPoses_inout[p_keyFrame] = g2oSiw;

        if (p_keyFrame->isImu)
        {
            VertexVelocity *p_velocityVertex = static_cast<VertexVelocity *>(
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 1));
            p_keyFrame->setVelocity(p_velocityVertex->estimate().cast<float>());
            VertexGyroBias *p_gyroBiasVertex = static_cast<VertexGyroBias *>(
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 2));
            VertexAccBias *p_accelerometerBiasVertex =
                static_cast<VertexAccBias *>(optimizer.vertex(
                    maximumKeyFrameId + 3 * (p_keyFrame->id) + 3));
            Vector6d b;
            b << p_gyroBiasVertex->estimate(),
                p_accelerometerBiasVertex->estimate();
            p_keyFrame->setNewBias(
                IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
        }
    }

    for (int i = 0; i < optimizableCovisibleCount; i++)
    {
        KeyFrame *p_keyFrame = optimizableCovisibleKeyFrames[i];

        VertexPose *p_poseVertex =
            static_cast<VertexPose *>(optimizer.vertex(p_keyFrame->id));
        Sophus::SE3f Tcw(p_poseVertex->estimate().Rcw[0].cast<float>(),
                         p_poseVertex->estimate().tcw[0].cast<float>());
        p_keyFrame->setPose(Tcw);

        Sophus::SE3d Tiw = p_keyFrame->getPose().cast<double>();
        g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
        corrPoses_inout[p_keyFrame] = g2oSiw;

        if (p_keyFrame->isImu)
        {
            VertexVelocity *p_velocityVertex = static_cast<VertexVelocity *>(
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 1));
            p_keyFrame->setVelocity(p_velocityVertex->estimate().cast<float>());
            VertexGyroBias *p_gyroBiasVertex = static_cast<VertexGyroBias *>(
                optimizer.vertex(maximumKeyFrameId + 3 * (p_keyFrame->id) + 2));
            VertexAccBias *p_accelerometerBiasVertex =
                static_cast<VertexAccBias *>(optimizer.vertex(
                    maximumKeyFrameId + 3 * (p_keyFrame->id) + 3));
            Vector6d b;
            b << p_gyroBiasVertex->estimate(),
                p_accelerometerBiasVertex->estimate();
            p_keyFrame->setNewBias(
                IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
        }
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
        p_mapPoint->setWorldPos(p_pointVertex->estimate().cast<float>());
        p_mapPoint->updateNormalAndDepth();
    }

    p_map_inout->increaseChangeIndex();
}

} // namespace core
} // namespace vs_graphs
