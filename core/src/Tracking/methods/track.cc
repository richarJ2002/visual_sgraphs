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

#include "ResetCause.h"

#include <chrono>
#include <iostream>
#include <mutex>

namespace vs_graphs
{
namespace core
{

void Tracking::track()
{
    if (stepByStep)
    {
        std::cout << "Waiting for the next step in Tracking ..." << std::endl;
        while (!step && stepByStep)
            usleep(500);
        step = false;
    }

    if (p_localMapper->badImu)
    {
        cout << "[Tracking] Reseting map because the Local Mapper set the 'Bad "
                "IMU' flag ..."
             << endl;
        p_system->requestResetActiveMapWithCause(
            ResetCause::LOCAL_MAPPER_BAD_IMU);
        return;
    }

    Map *pCurrentMap = p_atlas->getCurrentMap();
    if (!pCurrentMap)
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
            unique_lock<mutex> lock(mMutexImuQueue);
            queueImuData.clear();
            reportResetAttribution(ResetCause::NON_MONOTONIC_SENSOR_TIMESTAMP,
                                   ResetAction::CREATE_MAP_EXECUTION);
            createMapInAtlas();
            return;
        }
        else if (currentFrame.timeStamp > lastFrame.timeStamp + 1.0)
        {
            // cout << mCurrentFrame.timeStamp << ", " << mLastFrame.timeStamp
            // << endl; cout << "id last: " << mLastFrame.mnId << "    id curr:
            // " << mCurrentFrame.mnId << endl;
            if (p_atlas->isInertial())
            {

                if (p_atlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. "
                            "Reseting IMU integration..."
                         << endl;
                    if (!pCurrentMap->getInertialBA2())
                    {
                        p_system->requestResetActiveMapWithCause(
                            ResetCause::TIMESTAMP_JUMP_BEFORE_SECOND_IMU_BA);
                    }
                    else
                    {
                        reportResetAttribution(
                            ResetCause::TIMESTAMP_JUMP_AFTER_SECOND_IMU_BA,
                            ResetAction::CREATE_MAP_EXECUTION);
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
        currentFrame.setNewBias(p_lastKeyFrame->getImuBias());

    if (state == NO_IMAGES_YET)
        state = NOT_INITIALIZED;

    lastProcessedState = state;

    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        !createdMap)
    {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPreIMU =
            std::chrono::steady_clock::now();
#endif
        preintegrateIMU();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPreIMU =
            std::chrono::steady_clock::now();

        double timePreImu = std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli>>(
                                time_EndPreIMU - time_StartPreIMU)
                                .count();
        vdIMUInteg_ms.push_back(timePreImu);
#endif
    }
    createdMap = false;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->getMapChangeIndex();
    int nMapChangeIndex    = pCurrentMap->getLastMapChange();
    if (nCurMapChangeIndex > nMapChangeIndex)
    {
        pCurrentMap->setLastMapChange(nCurMapChangeIndex);
        mapUpdated = true;
    }

    if (state == NOT_INITIALIZED)
    {
        if (sensor == System::STEREO || sensor == System::RGBD ||
            sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
            stereoInitialization();
        else
            monocularInitialization();

        // If initialization succesful, save frame pose
        if (state != OK)
        {
            lastFrame = Frame(currentFrame);
            return;
        }

        if (p_atlas->getAllMaps().size() == 1)
            firstFrameId = currentFrame.mnId;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK = false;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPosePred =
            std::chrono::steady_clock::now();
#endif

        // Initial camera pose estimation using motion model or relocalization
        // (if tracking is lost)
        if (!onlyTracking)
        {

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if (state == OK)
            {

                // Local Mapping might have changed some MapPoints tracked in
                // last frame
                checkReplacedInLastFrame();

                if ((!velocityAvailable && !pCurrentMap->isImuInitialized()) ||
                    currentFrame.mnId < lastRelocFrameId + 2)
                {
                    Verbose::printMess(
                        "TRACK: Track with respect to the reference KF ",
                        Verbose::VERBOSITY_DEBUG);
                    bOK = trackReferenceKeyFrame();
                }
                else
                {
                    Verbose::printMess("TRACK: Track with motion model",
                                       Verbose::VERBOSITY_DEBUG);
                    bOK = trackWithMotionModel();
                    if (!bOK)
                        bOK = trackReferenceKeyFrame();
                }

                if (!bOK)
                {
                    if (currentFrame.mnId <=
                            (lastRelocFrameId + framesToResetIMU) &&
                        (sensor == System::IMU_MONOCULAR ||
                         sensor == System::IMU_STEREO ||
                         sensor == System::IMU_RGBD))
                    {
                        state = LOST;
                    }
                    else if (pCurrentMap->getKeyFrameCount() > 10)
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
            else
            {

                if (state == RECENTLY_LOST)
                {
                    Verbose::printMess("Lost for a short time",
                                       Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    if ((sensor == System::IMU_MONOCULAR ||
                         sensor == System::IMU_STEREO ||
                         sensor == System::IMU_RGBD))
                    {
                        if (pCurrentMap->isImuInitialized())
                            predictStateIMU();
                        else
                            bOK = false;

                        if (currentFrame.timeStamp - timeStampLost >
                            time_recently_lost)
                        {
                            state = LOST;
                            Verbose::printMess("Track Lost...",
                                               Verbose::VERBOSITY_NORMAL);
                            bOK = false;
                        }
                    }
                    else
                    {
                        // Relocalization
                        bOK = relocalization();
                        // std::cout << "mCurrentFrame.timeStamp:" <<
                        // to_string(mCurrentFrame.timeStamp) << std::endl;
                        // std::cout << "mTimeStampLost:" <<
                        // to_string(mTimeStampLost) << std::endl;
                        if (currentFrame.timeStamp - timeStampLost > 3.0f &&
                            !bOK)
                        {
                            state = LOST;
                            Verbose::printMess("Track Lost...",
                                               Verbose::VERBOSITY_NORMAL);
                            bOK = false;
                        }
                    }
                }
                else if (state == LOST)
                {

                    Verbose::printMess("A new map is started...",
                                       Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->getKeyFrameCount() < 10)
                    {
                        p_system->requestResetActiveMapWithCause(
                            ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP);
                        Verbose::printMess("Reseting current map...",
                                           Verbose::VERBOSITY_NORMAL);
                    }
                    else
                    {
                        reportResetAttribution(
                            ResetCause::VISUAL_TRACKING_LOST_NEW_MAP,
                            ResetAction::CREATE_MAP_EXECUTION);
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
                    Verbose::printMess("IMU. State LOST",
                                       Verbose::VERBOSITY_NORMAL);
                bOK = relocalization();
            }
            else
            {
                if (!visualOdometry)
                {
                    // In last frame we tracked enough MapPoints in the map
                    if (velocityAvailable)
                    {
                        bOK = trackWithMotionModel();
                    }
                    else
                    {
                        bOK = trackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and
                    // one doing relocalization. If relocalization is sucessfull
                    // we choose that solution, otherwise we retain the "visual
                    // odometry" solution.

                    bool               bOKMM    = false;
                    bool               bOKReloc = false;
                    vector<MapPoint *> vpMPsMM;
                    vector<bool>       vbOutMM;
                    Sophus::SE3f       TcwMM;
                    if (velocityAvailable)
                    {
                        bOKMM   = trackWithMotionModel();
                        vpMPsMM = currentFrame.mapPoints;
                        vbOutMM = currentFrame.outlierFlags;
                        TcwMM   = currentFrame.getPose();
                    }
                    bOKReloc = relocalization();

                    if (bOKMM && !bOKReloc)
                    {
                        currentFrame.setPose(TcwMM);
                        currentFrame.mapPoints    = vpMPsMM;
                        currentFrame.outlierFlags = vbOutMM;

                        if (visualOdometry)
                        {
                            for (int i = 0; i < currentFrame.N; i++)
                            {
                                if (currentFrame.mapPoints[i] &&
                                    !currentFrame.outlierFlags[i])
                                {
                                    currentFrame.mapPoints[i]->increaseFound();
                                }
                            }
                        }
                    }
                    else if (bOKReloc)
                    {
                        visualOdometry = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        if (!currentFrame.p_referenceKeyFrame)
            currentFrame.p_referenceKeyFrame = p_referenceKF;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPosePred =
            std::chrono::steady_clock::now();

        double timePosePred = std::chrono::duration_cast<
                                  std::chrono::duration<double, std::milli>>(
                                  time_EndPosePred - time_StartPosePred)
                                  .count();
        vdPosePred_ms.push_back(timePosePred);
#endif

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartLMTrack =
            std::chrono::steady_clock::now();
#endif
        // If we have an initial estimation of the camera pose and matching.
        // Track the local map.
        if (!onlyTracking)
        {
            if (bOK)
                bOK = trackLocalMap();
            else
                std::cout << "[Tracking] Failed to track the features ..."
                          << std::endl;
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the
            // map. We cannot retrieve a local map and therefore we do not
            // perform TrackLocalMap(). Once the system relocalizes the camera
            // we will use the local map again.
            if (bOK && !visualOdometry)
                bOK = trackLocalMap();
        }

        if (bOK)
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

            /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {*/
            timeStampLost = currentFrame.timeStamp;
            //}
        }

        if (pCurrentMap->isImuInitialized())
        {
            if (bOK)
            {
                if (currentFrame.mnId == (lastRelocFrameId + framesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                    resetFrameIMU();
                }
                else if (currentFrame.mnId > (lastRelocFrameId + 30))
                    lastBias = currentFrame.imuBias;
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndLMTrack =
            std::chrono::steady_clock::now();

        double timeLMTrack = std::chrono::duration_cast<
                                 std::chrono::duration<double, std::milli>>(
                                 time_EndLMTrack - time_StartLMTrack)
                                 .count();
        vdLMTrack_ms.push_back(timeLMTrack);
#endif

        // Update drawer
        p_frameDrawer->update(this);
        if (currentFrame.isSet())
            p_mapDrawer->setCurrentCameraPose(currentFrame.getPose());

        if (bOK || state == RECENTLY_LOST)
        {
            // Update motion model
            if (lastFrame.isSet() && currentFrame.isSet())
            {
                Sophus::SE3f LastTwc = lastFrame.getPose().inverse();
                velocity             = currentFrame.getPose() * LastTwc;
                velocityAvailable    = true;
            }
            else
            {
                velocityAvailable = false;
            }

            if (sensor == System::IMU_MONOCULAR ||
                sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
                p_mapDrawer->setCurrentCameraPose(currentFrame.getPose());

            // Clean VO matches
            for (int i = 0; i < currentFrame.N; i++)
            {
                MapPoint *pMP = currentFrame.mapPoints[i];
                if (pMP)
                    if (pMP->getObservationCount() < 1)
                    {
                        currentFrame.outlierFlags[i] = false;
                        currentFrame.mapPoints[i] =
                            static_cast<MapPoint *>(nullptr);
                    }
            }

            // Delete temporal MapPoints
            for (list<MapPoint *>::iterator lit  = mlpTemporalPoints.begin(),
                                            lend = mlpTemporalPoints.end();
                 lit != lend;
                 lit++)
            {
                MapPoint *pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartNewKF =
                std::chrono::steady_clock::now();
#endif
            bool bNeedKF = needNewKeyFrame();

            // Check if we need to insert a new keyframe
            if (bNeedKF && (bOK || (insertKFsLost && state == RECENTLY_LOST &&
                                    (sensor == System::IMU_MONOCULAR ||
                                     sensor == System::IMU_STEREO ||
                                     sensor == System::IMU_RGBD))))
            {
                // Create a new KeyFrame
                createNewKeyFrame();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndNewKF =
                std::chrono::steady_clock::now();

            double timeNewKF = std::chrono::duration_cast<
                                   std::chrono::duration<double, std::milli>>(
                                   time_EndNewKF - time_StartNewKF)
                                   .count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

            // We allow points with high innovation (considererd outliers by the
            // Huber Function) pass to the new keyframe, so that bundle
            // adjustment will finally decide if they are outliers or not. We
            // don't want next frame to estimate its position with those points
            // so we discard them in the frame. Only has effect if lastframe is
            // tracked
            for (int i = 0; i < currentFrame.N; i++)
            {
                if (currentFrame.mapPoints[i] && currentFrame.outlierFlags[i])
                    currentFrame.mapPoints[i] =
                        static_cast<MapPoint *>(nullptr);
            }
        }

        // Reset if the camera get lost soon after initialization
        if (state == LOST)
        {
            if (pCurrentMap->getKeyFrameCount() <= 10)
            {
                p_system->requestResetActiveMapWithCause(
                    ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP);
                return;
            }
            if (sensor == System::IMU_MONOCULAR ||
                sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
                if (!pCurrentMap->isImuInitialized())
                {
                    Verbose::printMess(
                        "Track lost before IMU initialisation, reseting...",
                        Verbose::VERBOSITY_QUIET);
                    p_system->requestResetActiveMapWithCause(
                        ResetCause::
                            VISUAL_TRACKING_LOST_BEFORE_IMU_INITIALIZATION);
                    return;
                }

            reportResetAttribution(ResetCause::VISUAL_TRACKING_LOST_NEW_MAP,
                                   ResetAction::CREATE_MAP_EXECUTION);
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
        if (currentFrame.isSet())
        {
            Sophus::SE3f Tcr_ =
                currentFrame.getPose() *
                currentFrame.p_referenceKeyFrame->getPoseInverse();
            relativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(currentFrame.p_referenceKeyFrame);
            frameTimes.push_back(currentFrame.timeStamp);
            mlbLost.push_back(state == LOST);
        }
        else
        {
            // The current frame carries no pose (e.g. tracking was lost):
            // append the last stored entry to keep the trajectory aligned,
            // when one exists.
            if (!relativeFramePoses.empty() && !mlpReferences.empty() &&
                !frameTimes.empty())
            {
                relativeFramePoses.push_back(relativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                frameTimes.push_back(frameTimes.back());
                mlbLost.push_back(state == LOST);
            }
        }
    }

#ifdef REGISTER_LOOP
    if (stop())
    {

        // Safe area to stop
        while (isStopped())
        {
            usleep(3000);
        }
    }
#endif
}

} // namespace core
} // namespace vs_graphs
