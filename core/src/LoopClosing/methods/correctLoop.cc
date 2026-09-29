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

#include "LoopClosing.h"

#include "LocalMapping.h"
#include "Optimizer.h"
#include "System.h"
#include "Tracking.h"

#include <chrono>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

LoopClosingStatus LoopClosing::correctLoop()
{
    // Avoid new keyframes are inserted while correcting the loop
    if (p_localMapper->requestStop() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: requestStop returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_localMapper->emptyQueue() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: emptyQueue returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Stop and reclaim any global bundle-adjustment worker before mutation. */
    bool wasRunning{};
    if (stopGlobalBundleAdjustment(wasRunning) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: stopGlobalBundleAdjustment returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    // Wait until Local Mapping has effectively stopped
    for (;;)
    {
        bool localMapperIsStopped{};
        if (p_localMapper->isStopped(localMapperIsStopped) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isStopped returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (localMapperIsStopped)
        {
            break;
        }
        usleep(1000);
    }

    // Ensure current keyframe is updated
    if (p_currentKF->updateConnections() !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateConnections returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    // Retrive keyframes connected to the current keyframe and compute corrected
    // Sim3 pose by propagation
    std::vector<KeyFrame *> currentKFVectorCovisibleKeyFrames{};
    if (p_currentKF->getVectorCovisibleKeyFrames(
            currentKFVectorCovisibleKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getVectorCovisibleKeyFrames returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    currentConnectedKFs = currentKFVectorCovisibleKeyFrames;
    currentConnectedKFs.push_back(p_currentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[p_currentKF] = mg2oLoopScw;
    Sophus::SE3f Twc{};
    if (p_currentKF->getPoseInverse(Twc) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPoseInverse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f Tcw{};
    if (p_currentKF->getPose(Tcw) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    g2o::Sim3 g2oScw(Tcw.unit_quaternion().cast<double>(),
                     Tcw.translation().cast<double>(),
                     1.0);
    NonCorrectedSim3[p_currentKF] = g2oScw;

    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3
    // (scale translation)
    Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(),
                              mg2oLoopScw.translation() / mg2oLoopScw.scale());
    if (p_currentKF->setPose(correctedTcw.cast<float>()) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    Map *p_loopMap = nullptr;
    if (p_currentKF->getMap(p_loopMap) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeStartFusion =
        std::chrono::steady_clock::now();
#endif

    {
        // Get Map Mutex
        unique_lock<mutex> lock(p_loopMap->mapUpdateMutex);

        bool isImuInitialized{};
        if (p_loopMap->isImuInitialized(isImuInitialized) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isImuInitialized returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        for (vector<KeyFrame *>::iterator vit  = currentConnectedKFs.begin(),
                                          vend = currentConnectedKFs.end();
             vit != vend;
             vit++)
        {
            KeyFrame *p_keyFrame = *vit;

            if (p_keyFrame != p_currentKF)
            {
                Sophus::SE3f Tiw{};
                if (p_keyFrame->getPose(Tiw) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
                g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oLoopScw;
                // Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[p_keyFrame] = g2oCorrectedSiw;

                // Update keyframe pose with corrected Sim3. First transform
                // Sim3 to SE3 (scale translation)
                Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),
                                          g2oCorrectedSiw.translation() /
                                              g2oCorrectedSiw.scale());
                if (p_keyFrame->setPose(correctedTiw.cast<float>()) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                // Pose without correction
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(),
                                 Tiw.translation().cast<double>(),
                                 1.0);
                NonCorrectedSim3[p_keyFrame] = g2oSiw;
            }
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so
        // that they align with the other side of the loop
        for (KeyFrameAndPose::iterator mit  = CorrectedSim3.begin(),
                                       mend = CorrectedSim3.end();
             mit != mend;
             mit++)
        {
            KeyFrame *p_keyFrame      = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw = NonCorrectedSim3[p_keyFrame];

            // Update keyframe pose with corrected Sim3. First transform Sim3 to
            // SE3 (scale translation)
            /*Sophus::SE3d
            correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation()
            / g2oCorrectedSiw.scale());
            pKFi->setPose(correctedTiw.cast<float>());*/

            std::vector<MapPoint *> mapPoints{};
            if (p_keyFrame->getMapPointMatches(mapPoints) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPointMatches returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (size_t mapPointIndex = 0, endMapPoint = mapPoints.size();
                 mapPointIndex < endMapPoint;
                 mapPointIndex++)
            {
                MapPoint *p_mapPoint = mapPoints[mapPointIndex];
                if (!p_mapPoint)
                    continue;
                bool mapPointIsBad{};
                if (p_mapPoint->isBad(mapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (mapPointIsBad)
                    continue;
                if (p_mapPoint->correctedByKeyFrameId == p_currentKF->id)
                    continue;

                // Project with non-corrected pose and project back with
                // corrected pose
                Eigen::Vector3f mapPointWorldPos{};
                if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWorldPos returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3d P3Dw = mapPointWorldPos.cast<double>();
                Eigen::Vector3d eigCorrectedP3Dw =
                    g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

                if (p_mapPoint->setWorldPos(eigCorrectedP3Dw.cast<float>()) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setWorldPos returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                p_mapPoint->correctedByKeyFrameId        = p_currentKF->id;
                p_mapPoint->correctedReferenceKeyFrameId = p_keyFrame->id;
                if (p_mapPoint->updateNormalAndDepth() !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: updateNormalAndDepth returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }

            // Correct velocity according to orientation correction
            if (isImuInitialized)
            {
                Eigen::Quaternionf Rcor =
                    (g2oCorrectedSiw.rotation().inverse() * g2oSiw.rotation())
                        .cast<float>();
                Eigen::Vector3f keyFrameVelocity{};
                if (p_keyFrame->getVelocity(keyFrameVelocity) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getVelocity returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_keyFrame->setVelocity(Rcor * keyFrameVelocity) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setVelocity returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }

            // Make sure connections are updated
            if (p_keyFrame->updateConnections() !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updateConnections returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        // TODO Check this index increasement
        Map *p_atlasCurrentMap = nullptr;
        if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_atlasCurrentMap->increaseChangeIndex() !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: increaseChangeIndex returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for (size_t loopMatchedMapPointIndex = 0;
             loopMatchedMapPointIndex < loopMatchedMPs.size();
             loopMatchedMapPointIndex++)
        {
            if (loopMatchedMPs[loopMatchedMapPointIndex])
            {
                MapPoint *p_loopMapPoint =
                    loopMatchedMPs[loopMatchedMapPointIndex];
                MapPoint *p_currentMapPoint = nullptr;
                if (p_currentKF->getMapPoint(loopMatchedMapPointIndex,
                                             p_currentMapPoint) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMapPoint returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_currentMapPoint)
                {
                    if (p_currentMapPoint->replace(p_loopMapPoint) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: replace returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                }
                else
                {
                    if (p_currentKF->addMapPoint(p_loopMapPoint,
                                                 loopMatchedMapPointIndex) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addMapPoint returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_loopMapPoint->addObservation(
                            p_currentKF,
                            loopMatchedMapPointIndex) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addObservation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_loopMapPoint->computeDistinctiveDescriptors() !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: computeDistinctiveDescriptors "
                                     "returned a failure status although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                }
            }
        }
        // cout << "LC: end replacing duplicated" << endl;
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    if (searchAndFuse(CorrectedSim3, loopMapPoints) !=
        LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: searchAndFuse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // After the MapPoint fusion, new links in the covisibility graph will
    // appear attaching both sides of the loop
    map<KeyFrame *, set<KeyFrame *>> loopConnections;

    for (vector<KeyFrame *>::iterator vit  = currentConnectedKFs.begin(),
                                      vend = currentConnectedKFs.end();
         vit != vend;
         vit++)
    {
        KeyFrame               *p_keyFrame = *vit;
        std::vector<KeyFrame *> previousNeighbors{};
        if (p_keyFrame->getVectorCovisibleKeyFrames(previousNeighbors) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getVectorCovisibleKeyFrames returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        // Update connections. Detect new links.
        if (p_keyFrame->updateConnections() !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateConnections returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::set<KeyFrame *> keyFrameConnectedKeyFrames{};
        if (p_keyFrame->getConnectedKeyFrames(keyFrameConnectedKeyFrames) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getConnectedKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        loopConnections[p_keyFrame] = keyFrameConnectedKeyFrames;
        for (vector<KeyFrame *>::iterator
                 vitPrevious  = previousNeighbors.begin(),
                 vendPrevious = previousNeighbors.end();
             vitPrevious != vendPrevious;
             vitPrevious++)
        {
            loopConnections[p_keyFrame].erase(*vitPrevious);
        }
        for (vector<KeyFrame *>::iterator vit2  = currentConnectedKFs.begin(),
                                          vend2 = currentConnectedKFs.end();
             vit2 != vend2;
             vit2++)
        {
            loopConnections[p_keyFrame].erase(*vit2);
        }
    }

    // Optimize graph
    bool isFixedScale = isScaleFixed;
    // TODO CHECK; Solo para el monocular inertial
    Map *p_currentKFMap = nullptr;
    if ((p_tracker->sensor == System::IMU_MONOCULAR) &&
        p_currentKF->getMap(p_currentKFMap) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool inertialBA2{};
    if ((p_tracker->sensor == System::IMU_MONOCULAR) &&
        p_currentKFMap->getInertialBA2(inertialBA2) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getInertialBA2 returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_tracker->sensor == System::IMU_MONOCULAR && !inertialBA2)
        isFixedScale = false;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeEndFusion =
        std::chrono::steady_clock::now();

    double timeFusion =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            timeEndFusion - timeStartFusion)
            .count();
    loopFusionTimes_ms.push_back(timeFusion);
#endif
    // cout << "Optimize essential graph" << endl;
    bool loopMapIsInertial{};
    if (p_loopMap->isInertial(loopMapIsInertial) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isInertial returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool loopMapIsImuInitialized{};
    if ((loopMapIsInertial) &&
        p_loopMap->isImuInitialized(loopMapIsImuInitialized) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (loopMapIsInertial && loopMapIsImuInitialized)
    {
        if (Optimizer::optimizeEssentialGraph4DoF(p_loopMap,
                                                  p_loopMatchedKF,
                                                  p_currentKF,
                                                  NonCorrectedSim3,
                                                  CorrectedSim3,
                                                  loopConnections) !=
            OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: optimizeEssentialGraph4DoF returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
    }
    else
    {
        // cout << "Loop -> Scale correction: " << mg2oLoopScw.scale() << endl;
        if (Optimizer::optimizeEssentialGraph(p_loopMap,
                                              p_loopMatchedKF,
                                              p_currentKF,
                                              NonCorrectedSim3,
                                              CorrectedSim3,
                                              loopConnections,
                                              isFixedScale) !=
            OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: optimizeEssentialGraph returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point timeEndOpt =
        std::chrono::steady_clock::now();

    double timeOptEss =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            timeEndOpt - timeEndFusion)
            .count();
    loopEssentialGraphTimes_ms.push_back(timeOptEss);
#endif

    if (p_atlas->informNewBigChange() != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: informNewBigChange returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    // Add loop edge
    if (p_loopMatchedKF->addLoopEdge(p_currentKF) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addLoopEdge returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_currentKF->addLoopEdge(p_loopMatchedKF) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addLoopEdge returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // Launch a new thread to perform Global Bundle Adjustment (Only if few
    // keyframes, if not it would take too much time)
    bool loopMapIsImuInitialized2{};
    if (p_loopMap->isImuInitialized(loopMapIsImuInitialized2) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    unsigned long loopMapKeyFrameCount{};
    if (!(!loopMapIsImuInitialized2) &&
        p_loopMap->getKeyFrameCount(loopMapKeyFrameCount) !=
            MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKeyFrameCount returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    int atlasMaps{};
    if (!(!loopMapIsImuInitialized2) && (loopMapKeyFrameCount < 200) &&
        p_atlas->countMaps(atlasMaps) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: countMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (!loopMapIsImuInitialized2 ||
        (loopMapKeyFrameCount < 200 && atlasMaps == 1))
    {
        std::unique_lock<std::mutex> globalBundleAdjustmentLock(gbaMutex);
        isGbaRunning   = true;
        hasGbaFinished = false;
        correctionGBA  = numCorrection;
        isGlobalBundleAdjustmentStopRequested.store(false,
                                                    std::memory_order_release);

        p_threadGBA = new thread(&LoopClosing::runGlobalBundleAdjustment,
                                 this,
                                 p_loopMap,
                                 p_currentKF->id,
                                 fullBundleAdjustmentIndex);
    }

    // Loop closed. Release Local Mapping.
    if (p_localMapper->release() !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: release returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    lastLoopKeyFrameId =
        p_currentKF->id; // TODO old varible, it is not use in the new algorithm

    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
