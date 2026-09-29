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

#include "Tracking.h"

#include "FrameDrawer.h"
#include "LocalMapping.h"
#include "ResetCause.h"
#include "System.h"

#include <chrono>
#include <iostream>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void Tracking::track()
{
    if (isStepByStepMode)
    {
        std::cout << "Waiting for the next step in Tracking ..." << std::endl;
        while (!isStepRequested && isStepByStepMode)
            usleep(500);
        isStepRequested = false;
    }

    if (p_localMapper->isImuBad)
    {
        cout << "[Tracking] Reseting map because the Local Mapper set the 'Bad "
                "IMU' flag ..."
             << endl;
        p_system->requestResetActiveMapWithCause(
            ResetCause::LOCAL_MAPPER_BAD_IMU);
        return;
    }

    Map *p_currentMap = p_atlas->getCurrentMap();
    if (!p_currentMap)
    {
        cout << "[ERROR] No active maps found in the ATLAS!" << endl;
        return;
    }

    if (state != NO_IMAGES_YET)
    {
        if (lastFrame.timeStamp > currentFrame.timeStamp)
        {
            cerr << "ERROR: Frame with a timestamp older than previous frame "
                    "detected!"
                 << endl;
            unique_lock<mutex> lock(imuQueueMutex);
            queueImuData.clear();
            if (reportResetAttribution(
                    ResetCause::NON_MONOTONIC_SENSOR_TIMESTAMP,
                    ResetAction::CREATE_MAP_EXECUTION) !=
                ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: reportResetAttribution returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            createMapInAtlas();
            return;
        }
        else if (currentFrame.timeStamp > lastFrame.timeStamp + 1.0)
        {
            // cout << mCurrentFrame.timeStamp << ", " << mLastFrame.timeStamp
            // << endl; cout << "id last: " << mLastFrame.id << "    id curr:
            // " << mCurrentFrame.id << endl;
            if (p_atlas->isInertial())
            {

                if (p_atlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. "
                            "Reseting IMU integration..."
                         << endl;
                    bool currentMapInertialBA2{};
                    if (p_currentMap->getInertialBA2(currentMapInertialBA2) !=
                        MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getInertialBA2 returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (!currentMapInertialBA2)
                    {
                        p_system->requestResetActiveMapWithCause(
                            ResetCause::TIMESTAMP_JUMP_BEFORE_SECOND_IMU_BA);
                    }
                    else
                    {
                        if (reportResetAttribution(
                                ResetCause::TIMESTAMP_JUMP_AFTER_SECOND_IMU_BA,
                                ResetAction::CREATE_MAP_EXECUTION) !=
                            ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
                        {
                            // reportResetAttribution cannot fail; continue as
                            // before.
                        }
                        createMapInAtlas();
                    }
                }
                else
                {
                    cout << "Timestamp jump detected, before IMU "
                            "initialization. Reseting..."
                         << endl;
                    p_system->requestResetActiveMapWithCause(
                        ResetCause::TIMESTAMP_JUMP_BEFORE_IMU_INITIALIZATION);
                }
                return;
            }
        }
    }

    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        p_lastKeyFrame)
    {
        IMU::Bias lastKeyFrameImuBias{};
        if (p_lastKeyFrame->getImuBias(lastKeyFrameImuBias) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (currentFrame.setNewBias(lastKeyFrameImuBias) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setNewBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    if (state == NO_IMAGES_YET)
        state = NOT_INITIALIZED;

    lastProcessedState = state;

    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        !hasCreatedMap)
    {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point timeStartPreImu =
            std::chrono::steady_clock::now();
#endif
        preintegrateIMU();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point timeEndPreImu =
            std::chrono::steady_clock::now();

        double timePreImu = std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli>>(
                                timeEndPreImu - timeStartPreImu)
                                .count();
        imuIntegrationTimes_ms.push_back(timePreImu);
#endif
    }
    hasCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(p_currentMap->mapUpdateMutex);

    isMapUpdated = false;

    int currentMapChangeIndexCount{};
    if (p_currentMap->getMapChangeIndex(currentMapChangeIndexCount) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapChangeIndex returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    int mapChangeIndexCount{};
    if (p_currentMap->getLastMapChange(mapChangeIndexCount) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLastMapChange returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (currentMapChangeIndexCount > mapChangeIndexCount)
    {
        if (p_currentMap->setLastMapChange(currentMapChangeIndexCount) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setLastMapChange returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        isMapUpdated = true;
    }

    if (state == NOT_INITIALIZED)
    {
        if (sensor == System::STEREO || sensor == System::RGBD ||
            sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
        {
            stereoInitialization();
        }
        else
        {
            monocularInitialization();
        }

        // If initialization succesful, save frame pose
        if (state != OK)
        {
            lastFrame = Frame(currentFrame);
            return;
        }

        if (p_atlas->getAllMaps().size() == 1)
            firstFrameId = currentFrame.id;
    }
    else
    {
        // System is initialized. Track Frame.
        bool isOk = false;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point timeStartPosePred =
            std::chrono::steady_clock::now();
#endif

        // Initial camera pose estimation using motion model or relocalization
        // (if tracking is lost)
        if (!isTrackingOnlyMode)
        {

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if (state == OK)
            {

                // Local Mapping might have changed some MapPoints tracked in
                // last frame
                checkReplacedInLastFrame();

                bool currentMapIsImuInitialized{};
                if ((!isVelocityAvailable) && p_currentMap->isImuInitialized(
                                                  currentMapIsImuInitialized) !=
                                                  MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isImuInitialized returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if ((!isVelocityAvailable && !currentMapIsImuInitialized) ||
                    currentFrame.id < lastRelocFrameId + 2)
                {
                    Verbose::printMess(
                        "TRACK: Track with respect to the reference KF ",
                        Verbose::VERBOSITY_DEBUG);
                    isOk = trackReferenceKeyFrame();
                }
                else
                {
                    Verbose::printMess("TRACK: Track with motion model",
                                       Verbose::VERBOSITY_DEBUG);
                    isOk = trackWithMotionModel();
                    if (!isOk)
                    {
                        isOk = trackReferenceKeyFrame();
                    }
                }

                if (!isOk)
                {
                    if (currentFrame.id <=
                            (lastRelocFrameId + framesToResetIMU) &&
                        (sensor == System::IMU_MONOCULAR ||
                         sensor == System::IMU_STEREO ||
                         sensor == System::IMU_RGBD))
                    {
                        state = LOST;
                    }
                    else
                    {
                        unsigned long currentMapKeyFrameCount{};
                        if (p_currentMap->getKeyFrameCount(
                                currentMapKeyFrameCount) !=
                            MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getKeyFrameCount returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        if (currentMapKeyFrameCount > 10)
                        {
                            // cout << "KF in map: " <<
                            // pCurrentMap->KeyFramesInMap() << endl;
                            state         = RECENTLY_LOST;
                            timeStampLost = currentFrame.timeStamp;
                        }
                        else
                        {
                            state = LOST;
                        }
                    }
                }
            }
            else
            {

                if (state == RECENTLY_LOST)
                {
                    Verbose::printMess("Lost for a short time",
                                       Verbose::VERBOSITY_NORMAL);

                    isOk = true;
                    if ((sensor == System::IMU_MONOCULAR ||
                         sensor == System::IMU_STEREO ||
                         sensor == System::IMU_RGBD))
                    {
                        bool currentMapIsImuInitialized2{};
                        if (p_currentMap->isImuInitialized(
                                currentMapIsImuInitialized2) !=
                            MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: isImuInitialized returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        if (currentMapIsImuInitialized2)
                        {
                            predictStateIMU();
                        }
                        else
                        {
                            isOk = false;
                        }

                        if (currentFrame.timeStamp - timeStampLost >
                            time_recently_lost)
                        {
                            state = LOST;
                            Verbose::printMess("Track Lost...",
                                               Verbose::VERBOSITY_NORMAL);
                            isOk = false;
                        }
                    }
                    else
                    {
                        // Relocalization
                        isOk = relocalization();
                        // std::cout << "mCurrentFrame.timeStamp:" <<
                        // to_string(mCurrentFrame.timeStamp) << std::endl;
                        // std::cout << "mTimeStampLost:" <<
                        // to_string(mTimeStampLost) << std::endl;
                        if (currentFrame.timeStamp - timeStampLost > 3.0f &&
                            !isOk)
                        {
                            state = LOST;
                            Verbose::printMess("Track Lost...",
                                               Verbose::VERBOSITY_NORMAL);
                            isOk = false;
                        }
                    }
                }
                else if (state == LOST)
                {

                    Verbose::printMess("A new map is started...",
                                       Verbose::VERBOSITY_NORMAL);

                    unsigned long currentMapKeyFrameCount2{};
                    if (p_currentMap->getKeyFrameCount(
                            currentMapKeyFrameCount2) !=
                        MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getKeyFrameCount returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (currentMapKeyFrameCount2 < 10)
                    {
                        p_system->requestResetActiveMapWithCause(
                            ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP);
                        Verbose::printMess("Reseting current map...",
                                           Verbose::VERBOSITY_NORMAL);
                    }
                    else
                    {
                        if (reportResetAttribution(
                                ResetCause::VISUAL_TRACKING_LOST_NEW_MAP,
                                ResetAction::CREATE_MAP_EXECUTION) !=
                            ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
                        {
                            // reportResetAttribution cannot fail; continue as
                            // before.
                        }
                        createMapInAtlas();
                    }

                    if (p_lastKeyFrame)
                        p_lastKeyFrame = static_cast<KeyFrame *>(nullptr);

                    Verbose::printMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated (TODO Not
            // available in inertial mode)
            if (state == LOST)
            {
                if (sensor == System::IMU_MONOCULAR ||
                    sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
                {
                    Verbose::printMess("IMU. State LOST",
                                       Verbose::VERBOSITY_NORMAL);
                }
                isOk = relocalization();
            }
            else
            {
                if (!isVisualOdometry)
                {
                    // In last frame we tracked enough MapPoints in the map
                    if (isVelocityAvailable)
                    {
                        isOk = trackWithMotionModel();
                    }
                    else
                    {
                        isOk = trackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and
                    // one doing relocalization. If relocalization is sucessfull
                    // we choose that solution, otherwise we retain the "visual
                    // odometry" solution.

                    bool               bOKMM     = false;
                    bool               isOkReloc = false;
                    vector<MapPoint *> mapPointsMMs;
                    vector<bool>       outMmFlags;
                    Sophus::SE3f       TcwMM;
                    if (isVelocityAvailable)
                    {
                        bOKMM        = trackWithMotionModel();
                        mapPointsMMs = currentFrame.mapPoints;
                        outMmFlags   = currentFrame.outlierFlags;
                        Sophus::SE3<float> currentFrameGetPose{};
                        if (currentFrame.getPose(currentFrameGetPose) !=
                            FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getPose returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        TcwMM = currentFrameGetPose;
                    }
                    isOkReloc = relocalization();

                    if (bOKMM && !isOkReloc)
                    {
                        if (currentFrame.setPose(TcwMM) !=
                            FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: setPose returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        currentFrame.mapPoints    = mapPointsMMs;
                        currentFrame.outlierFlags = outMmFlags;

                        if (isVisualOdometry)
                        {
                            for (int keyPointIndex = 0;
                                 keyPointIndex < currentFrame.keyPointCount;
                                 keyPointIndex++)
                            {
                                if (currentFrame.mapPoints[keyPointIndex] &&
                                    !currentFrame.outlierFlags[keyPointIndex])
                                {
                                    if (currentFrame.mapPoints[keyPointIndex]
                                            ->increaseFound() !=
                                        MapPointStatus::
                                            MAP_POINT_STATUS_SUCCESS)
                                    {
                                        RCLCPP_ERROR(
                                            rclcpp::get_logger("vs_graphs"),
                                            "%s: increaseFound returned a "
                                            "failure status although it cannot "
                                            "fail; continuing as before.",
                                            __func__);
                                    }
                                }
                            }
                        }
                    }
                    else if (isOkReloc)
                    {
                        isVisualOdometry = false;
                    }

                    isOk = isOkReloc || bOKMM;
                }
            }
        }

        if (!currentFrame.p_referenceKeyFrame)
            currentFrame.p_referenceKeyFrame = p_referenceKF;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point timeEndPosePred =
            std::chrono::steady_clock::now();

        double timePosePred = std::chrono::duration_cast<
                                  std::chrono::duration<double, std::milli>>(
                                  timeEndPosePred - timeStartPosePred)
                                  .count();
        posePredictionTimes_ms.push_back(timePosePred);
#endif

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point timeStartLmTrack =
            std::chrono::steady_clock::now();
#endif
        // If we have an initial estimation of the camera pose and matching.
        // Track the local map.
        if (!isTrackingOnlyMode)
        {
            if (isOk)
            {
                isOk = trackLocalMap();
            }
            else
            {
                std::cout << "[Tracking] Failed to track the features ..."
                          << std::endl;
            }
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the
            // map. We cannot retrieve a local map and therefore we do not
            // perform TrackLocalMap(). Once the system relocalizes the camera
            // we will use the local map again.
            if (isOk && !isVisualOdometry)
            {
                isOk = trackLocalMap();
            }
        }

        if (isOk)
            state = OK;
        else if (state == OK)
        {
            if (sensor == System::IMU_MONOCULAR ||
                sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
            {
                Verbose::printMess("Visual tracking lost; entering bounded "
                                   "inertial recovery...",
                                   Verbose::VERBOSITY_NORMAL);
                /* Do not destroy a newly initialized inertial map after one
                 * failed visual update. RECENTLY_LOST already propagates the
                 * state with the IMU for a bounded recovery window and the
                 * LOST branch performs the appropriate reset or atlas-map
                 * transition if recovery actually fails. */
                state = RECENTLY_LOST;
            }
            else
                state = RECENTLY_LOST; // visual to lost

            /*if(mCurrentFrame.id>mnLastRelocFrameId+mMaxFrames)
            {*/
            timeStampLost = currentFrame.timeStamp;
            //}
        }

        bool currentMapIsImuInitialized3{};
        if (p_currentMap->isImuInitialized(currentMapIsImuInitialized3) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isImuInitialized returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (currentMapIsImuInitialized3)
        {
            if (isOk)
            {
                if (currentFrame.id == (lastRelocFrameId + framesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                    resetFrameIMU();
                }
                else if (currentFrame.id > (lastRelocFrameId + 30))
                    lastBias = currentFrame.imuBias;
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point timeEndLmTrack =
            std::chrono::steady_clock::now();

        double timeLmTrack = std::chrono::duration_cast<
                                 std::chrono::duration<double, std::milli>>(
                                 timeEndLmTrack - timeStartLmTrack)
                                 .count();
        localMapTrackTimes_ms.push_back(timeLmTrack);
#endif

        // Update drawer
        p_frameDrawer->update(this);
        bool currentFrameIsSet{};
        if (currentFrame.isSet(currentFrameIsSet) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isSet returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (currentFrameIsSet)
        {
            Sophus::SE3<float> currentFrameGetPose2{};
            if (currentFrame.getPose(currentFrameGetPose2) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            p_mapDrawer->setCurrentCameraPose(currentFrameGetPose2);
        }

        if (isOk || state == RECENTLY_LOST)
        {
            // Update motion model
            bool lastFrameIsSet{};
            if (lastFrame.isSet(lastFrameIsSet) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isSet returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            bool currentFrameIsSet2{};
            if ((lastFrameIsSet) && currentFrame.isSet(currentFrameIsSet2) !=
                                        FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isSet returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (lastFrameIsSet && currentFrameIsSet2)
            {
                Sophus::SE3<float> lastFrameGetPose{};
                if (lastFrame.getPose(lastFrameGetPose) !=
                    FrameStatus::FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Sophus::SE3f       LastTwc = lastFrameGetPose.inverse();
                Sophus::SE3<float> currentFrameGetPose3{};
                if (currentFrame.getPose(currentFrameGetPose3) !=
                    FrameStatus::FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                velocity            = currentFrameGetPose3 * LastTwc;
                isVelocityAvailable = true;
            }
            else
            {
                isVelocityAvailable = false;
            }

            if (sensor == System::IMU_MONOCULAR ||
                sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
            {
                Sophus::SE3<float> currentFrameGetPose4{};
                if (currentFrame.getPose(currentFrameGetPose4) !=
                    FrameStatus::FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                p_mapDrawer->setCurrentCameraPose(currentFrameGetPose4);
            }

            // Clean VO matches
            for (int keyPointIndex = 0;
                 keyPointIndex < currentFrame.keyPointCount;
                 keyPointIndex++)
            {
                MapPoint *p_mapPoint = currentFrame.mapPoints[keyPointIndex];
                if (p_mapPoint)
                {
                    int mapPointObservationCount{};
                    if (p_mapPoint->getObservationCount(
                            mapPointObservationCount) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getObservationCount returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (mapPointObservationCount < 1)
                    {
                        currentFrame.outlierFlags[keyPointIndex] = false;
                        currentFrame.mapPoints[keyPointIndex] =
                            static_cast<MapPoint *>(nullptr);
                    }
                }
            }

            // Delete temporal MapPoints
            for (list<MapPoint *>::iterator lit  = temporalMapPoints.begin(),
                                            lend = temporalMapPoints.end();
                 lit != lend;
                 lit++)
            {
                MapPoint *p_mapPoint = *lit;
                delete p_mapPoint;
            }
            temporalMapPoints.clear();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point timeStartNewKeyFrame =
                std::chrono::steady_clock::now();
#endif
            bool isNeedKeyFrame = needNewKeyFrame();

            // Check if we need to insert a new keyframe
            if (isNeedKeyFrame && (isOk || (shouldInsertKeyFramesWhenLost &&
                                            state == RECENTLY_LOST &&
                                            (sensor == System::IMU_MONOCULAR ||
                                             sensor == System::IMU_STEREO ||
                                             sensor == System::IMU_RGBD))))
            {
                // Create a new KeyFrame
                createNewKeyFrame();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point timeEndNewKeyFrame =
                std::chrono::steady_clock::now();

            double timeNewKeyFrame =
                std::chrono::duration_cast<
                    std::chrono::duration<double, std::milli>>(
                    timeEndNewKeyFrame - timeStartNewKeyFrame)
                    .count();
            newKeyFrameTimes_ms.push_back(timeNewKeyFrame);
#endif

            // We allow points with high innovation (considererd outliers by the
            // Huber Function) pass to the new keyframe, so that bundle
            // adjustment will finally decide if they are outliers or not. We
            // don't want next frame to estimate its position with those points
            // so we discard them in the frame. Only has effect if lastframe is
            // tracked
            for (int keyPointIndex = 0;
                 keyPointIndex < currentFrame.keyPointCount;
                 keyPointIndex++)
            {
                if (currentFrame.mapPoints[keyPointIndex] &&
                    currentFrame.outlierFlags[keyPointIndex])
                    currentFrame.mapPoints[keyPointIndex] =
                        static_cast<MapPoint *>(nullptr);
            }
        }

        // Reset if the camera get lost soon after initialization
        if (state == LOST)
        {
            unsigned long currentMapKeyFrameCount3{};
            if (p_currentMap->getKeyFrameCount(currentMapKeyFrameCount3) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getKeyFrameCount returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (currentMapKeyFrameCount3 <= 10)
            {
                p_system->requestResetActiveMapWithCause(
                    ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP);
                return;
            }
            if (sensor == System::IMU_MONOCULAR ||
                sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
            {
                bool currentMapIsImuInitialized4{};
                if (p_currentMap->isImuInitialized(
                        currentMapIsImuInitialized4) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isImuInitialized returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (!currentMapIsImuInitialized4)
                {
                    Verbose::printMess(
                        "Track lost before IMU initialisation, reseting...",
                        Verbose::VERBOSITY_QUIET);
                    p_system->requestResetActiveMapWithCause(
                        ResetCause::
                            VISUAL_TRACKING_LOST_BEFORE_IMU_INITIALIZATION);
                    return;
                }
            }

            if (reportResetAttribution(ResetCause::VISUAL_TRACKING_LOST_NEW_MAP,
                                       ResetAction::CREATE_MAP_EXECUTION) !=
                ResetCauseStatus::RESET_CAUSE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: reportResetAttribution returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            createMapInAtlas();

            return;
        }

        if (!currentFrame.p_referenceKeyFrame)
            currentFrame.p_referenceKeyFrame = p_referenceKF;

        lastFrame = Frame(currentFrame);
    }

    if (state == OK || state == RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera
        // trajectory afterwards.
        bool currentFrameIsSet3{};
        if (currentFrame.isSet(currentFrameIsSet3) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isSet returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (currentFrameIsSet3)
        {
            Sophus::SE3<float> currentFrameGetPose5{};
            if (currentFrame.getPose(currentFrameGetPose5) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            Sophus::SE3f poseInverse{};
            if (currentFrame.p_referenceKeyFrame->getPoseInverse(poseInverse) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Sophus::SE3f Tcr_ = currentFrameGetPose5 * poseInverse;
            relativeFramePoses.push_back(Tcr_);
            referenceKeyFrames.push_back(currentFrame.p_referenceKeyFrame);
            frameTimes.push_back(currentFrame.timeStamp);
            lostFlags.push_back(state == LOST);
        }
        else
        {
            // The current frame carries no pose (e.g. tracking was lost):
            // append the last stored entry to keep the trajectory aligned,
            // when one exists.
            if (!relativeFramePoses.empty() && !referenceKeyFrames.empty() &&
                !frameTimes.empty())
            {
                relativeFramePoses.push_back(relativeFramePoses.back());
                referenceKeyFrames.push_back(referenceKeyFrames.back());
                frameTimes.push_back(frameTimes.back());
                lostFlags.push_back(state == LOST);
            }
        }
    }

#ifdef REGISTER_LOOP
    if (stop())
    {

        // Safe area to stop
        for (;;)
        {
            if (!isStopped())
            {
                break;
            }
            usleep(3000);
        }
    }
#endif
}

} // namespace core
} // namespace vs_graphs
