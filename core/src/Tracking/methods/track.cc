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

TrackingStatus Tracking::track()
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
        if (p_system->requestResetActiveMapWithCause(
                ResetCause::LOCAL_MAPPER_BAD_IMU) !=
            SystemStatus::SYSTEM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: requestResetActiveMapWithCause returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    Map *p_currentMap = nullptr;
    if (p_atlas->getCurrentMap(p_currentMap) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (!p_currentMap)
    {
        cout << "[ERROR] No active maps found in the ATLAS!" << endl;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
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
            if (createMapInAtlas() != TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: createMapInAtlas returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }
        else if (currentFrame.timeStamp > lastFrame.timeStamp + 1.0)
        {
            // cout << mCurrentFrame.timeStamp << ", " << mLastFrame.timeStamp
            // << endl; cout << "id last: " << mLastFrame.id << "    id curr:
            // " << mCurrentFrame.id << endl;
            bool atlasIsInertial{};
            if (p_atlas->isInertial(atlasIsInertial) !=
                AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isInertial returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (atlasIsInertial)
            {

                bool atlasIsImuInitialized{};
                if (p_atlas->isImuInitialized(atlasIsImuInitialized) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isImuInitialized returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (atlasIsImuInitialized)
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
                        if (p_system->requestResetActiveMapWithCause(
                                ResetCause::
                                    TIMESTAMP_JUMP_BEFORE_SECOND_IMU_BA) !=
                            SystemStatus::SYSTEM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: requestResetActiveMapWithCause returned a "
                                "failure status although it cannot fail; "
                                "continuing as before.",
                                __func__);
                        }
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
                        if (createMapInAtlas() !=
                            TrackingStatus::TRACKING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: createMapInAtlas returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                    }
                }
                else
                {
                    cout << "Timestamp jump detected, before IMU "
                            "initialization. Reseting..."
                         << endl;
                    if (p_system->requestResetActiveMapWithCause(
                            ResetCause::
                                TIMESTAMP_JUMP_BEFORE_IMU_INITIALIZATION) !=
                        SystemStatus::SYSTEM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: requestResetActiveMapWithCause "
                                     "returned a failure status although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                }
                return TrackingStatus::TRACKING_STATUS_SUCCESS;
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
        if (preintegrateIMU() != TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: preintegrateIMU returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
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
            if (stereoInitialization() !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: stereoInitialization returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
        else
        {
            if (monocularInitialization() !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: monocularInitialization returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        // If initialization succesful, save frame pose
        if (state != OK)
        {
            lastFrame = Frame(currentFrame);
            return TrackingStatus::TRACKING_STATUS_SUCCESS;
        }

        std::vector<Map *> atlasAllMaps{};
        if (p_atlas->getAllMaps(atlasAllMaps) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllMaps returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (atlasAllMaps.size() == 1)
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
                if (checkReplacedInLastFrame() !=
                    TrackingStatus::TRACKING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: checkReplacedInLastFrame returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }

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
                    if (Verbose::printMess(
                            "TRACK: Track with respect to the reference KF ",
                            Verbose::VERBOSITY_DEBUG) !=
                        VerboseStatus::VERBOSE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: printMess returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    bool isTracked{};
                    if (trackReferenceKeyFrame(isTracked) !=
                        TrackingStatus::TRACKING_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: trackReferenceKeyFrame returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    isOk = isTracked;
                }
                else
                {
                    if (Verbose::printMess("TRACK: Track with motion model",
                                           Verbose::VERBOSITY_DEBUG) !=
                        VerboseStatus::VERBOSE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: printMess returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    bool isTracked2{};
                    if (trackWithMotionModel(isTracked2) !=
                        TrackingStatus::TRACKING_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: trackWithMotionModel returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    isOk = isTracked2;
                    if (!isOk)
                    {
                        bool isTracked3{};
                        if (trackReferenceKeyFrame(isTracked3) !=
                            TrackingStatus::TRACKING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: trackReferenceKeyFrame returned "
                                         "a failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        isOk = isTracked3;
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
                    if (Verbose::printMess("Lost for a short time",
                                           Verbose::VERBOSITY_NORMAL) !=
                        VerboseStatus::VERBOSE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: printMess returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }

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
                            bool isPredicted{};
                            if (predictStateIMU(isPredicted) !=
                                TrackingStatus::TRACKING_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: predictStateIMU returned a failure "
                                    "status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                        }
                        else
                        {
                            isOk = false;
                        }

                        if (currentFrame.timeStamp - timeStampLost >
                            time_recently_lost)
                        {
                            state = LOST;
                            if (Verbose::printMess("Track Lost...",
                                                   Verbose::VERBOSITY_NORMAL) !=
                                VerboseStatus::VERBOSE_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: printMess returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            isOk = false;
                        }
                    }
                    else
                    {
                        // Relocalization
                        bool isRelocalized{};
                        if (relocalization(isRelocalized) !=
                            TrackingStatus::TRACKING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: relocalization returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        isOk = isRelocalized;
                        // std::cout << "mCurrentFrame.timeStamp:" <<
                        // to_string(mCurrentFrame.timeStamp) << std::endl;
                        // std::cout << "mTimeStampLost:" <<
                        // to_string(mTimeStampLost) << std::endl;
                        if (currentFrame.timeStamp - timeStampLost > 3.0f &&
                            !isOk)
                        {
                            state = LOST;
                            if (Verbose::printMess("Track Lost...",
                                                   Verbose::VERBOSITY_NORMAL) !=
                                VerboseStatus::VERBOSE_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: printMess returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            isOk = false;
                        }
                    }
                }
                else if (state == LOST)
                {

                    if (Verbose::printMess("A new map is started...",
                                           Verbose::VERBOSITY_NORMAL) !=
                        VerboseStatus::VERBOSE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: printMess returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }

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
                        if (p_system->requestResetActiveMapWithCause(
                                ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP) !=
                            SystemStatus::SYSTEM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: requestResetActiveMapWithCause returned a "
                                "failure status although it cannot fail; "
                                "continuing as before.",
                                __func__);
                        }
                        if (Verbose::printMess("Reseting current map...",
                                               Verbose::VERBOSITY_NORMAL) !=
                            VerboseStatus::VERBOSE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: printMess returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
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
                        if (createMapInAtlas() !=
                            TrackingStatus::TRACKING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: createMapInAtlas returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                    }

                    if (p_lastKeyFrame)
                        p_lastKeyFrame = static_cast<KeyFrame *>(nullptr);

                    if (Verbose::printMess("done", Verbose::VERBOSITY_NORMAL) !=
                        VerboseStatus::VERBOSE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: printMess returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }

                    return TrackingStatus::TRACKING_STATUS_SUCCESS;
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
                    if (Verbose::printMess("IMU. State LOST",
                                           Verbose::VERBOSITY_NORMAL) !=
                        VerboseStatus::VERBOSE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: printMess returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                }
                bool isRelocalized2{};
                if (relocalization(isRelocalized2) !=
                    TrackingStatus::TRACKING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: relocalization returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                isOk = isRelocalized2;
            }
            else
            {
                if (!isVisualOdometry)
                {
                    // In last frame we tracked enough MapPoints in the map
                    if (isVelocityAvailable)
                    {
                        bool isTracked4{};
                        if (trackWithMotionModel(isTracked4) !=
                            TrackingStatus::TRACKING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: trackWithMotionModel returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        isOk = isTracked4;
                    }
                    else
                    {
                        bool isTracked5{};
                        if (trackReferenceKeyFrame(isTracked5) !=
                            TrackingStatus::TRACKING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: trackReferenceKeyFrame returned "
                                         "a failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        isOk = isTracked5;
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
                        bool isTracked6{};
                        if (trackWithMotionModel(isTracked6) !=
                            TrackingStatus::TRACKING_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: trackWithMotionModel returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        bOKMM        = isTracked6;
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
                    bool isRelocalized3{};
                    if (relocalization(isRelocalized3) !=
                        TrackingStatus::TRACKING_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: relocalization returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    isOkReloc = isRelocalized3;

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
                bool isTracked7{};
                if (trackLocalMap(isTracked7) !=
                    TrackingStatus::TRACKING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: trackLocalMap returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                isOk = isTracked7;
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
                bool isTracked8{};
                if (trackLocalMap(isTracked8) !=
                    TrackingStatus::TRACKING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: trackLocalMap returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                isOk = isTracked8;
            }
        }

        if (isOk)
            state = OK;
        else if (state == OK)
        {
            if (sensor == System::IMU_MONOCULAR ||
                sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
            {
                if (Verbose::printMess("Visual tracking lost; entering bounded "
                                       "inertial recovery...",
                                       Verbose::VERBOSITY_NORMAL) !=
                    VerboseStatus::VERBOSE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: printMess returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
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
                    if (resetFrameIMU() !=
                        TrackingStatus::TRACKING_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: resetFrameIMU returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
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
        if (p_frameDrawer->update(this) !=
            FrameDrawerStatus::FRAME_DRAWER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: update returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
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
            if (p_mapDrawer->setCurrentCameraPose(currentFrameGetPose2) !=
                MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setCurrentCameraPose returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
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
                if (p_mapDrawer->setCurrentCameraPose(currentFrameGetPose4) !=
                    MapDrawerStatus::MAP_DRAWER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setCurrentCameraPose returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
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
            bool isNeedKeyFrame{};
            if (needNewKeyFrame(isNeedKeyFrame) !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: needNewKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            // Check if we need to insert a new keyframe
            if (isNeedKeyFrame && (isOk || (shouldInsertKeyFramesWhenLost &&
                                            state == RECENTLY_LOST &&
                                            (sensor == System::IMU_MONOCULAR ||
                                             sensor == System::IMU_STEREO ||
                                             sensor == System::IMU_RGBD))))
            {
                // Create a new KeyFrame
                if (createNewKeyFrame() !=
                    TrackingStatus::TRACKING_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: createNewKeyFrame returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
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
                if (p_system->requestResetActiveMapWithCause(
                        ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP) !=
                    SystemStatus::SYSTEM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: requestResetActiveMapWithCause returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                return TrackingStatus::TRACKING_STATUS_SUCCESS;
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
                    if (Verbose::printMess(
                            "Track lost before IMU initialisation, reseting...",
                            Verbose::VERBOSITY_QUIET) !=
                        VerboseStatus::VERBOSE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: printMess returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_system->requestResetActiveMapWithCause(
                            ResetCause::
                                VISUAL_TRACKING_LOST_BEFORE_IMU_INITIALIZATION) !=
                        SystemStatus::SYSTEM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: requestResetActiveMapWithCause "
                                     "returned a failure status although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                    return TrackingStatus::TRACKING_STATUS_SUCCESS;
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
            if (createMapInAtlas() != TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: createMapInAtlas returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            return TrackingStatus::TRACKING_STATUS_SUCCESS;
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
    bool isStopped2{};
    if (stop(isStopped2) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: stop returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (isStopped2)
    {

        // Safe area to stop
        for (;;)
        {
            bool isStopped3{};
            if (isStopped(isStopped3) !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isStopped returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (!isStopped3)
            {
                break;
            }
            usleep(3000);
        }
    }
#endif

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
