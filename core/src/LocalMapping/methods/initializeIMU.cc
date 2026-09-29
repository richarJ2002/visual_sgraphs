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

#include "LocalMapping.h"

#include "Optimizer.h"
#include "System.h"
#include "Tracking.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void LocalMapping::initializeIMU(float gyroPriorWeight_in,
                                 float accelPriorWeight_in,
                                 bool  shouldRunFullInertialBa_in)
{
    if (isResetRequested)
        return;

    float       minInitializationTime;
    std::size_t minKeyFrameCount;
    if (isMonocular)
    {
        minInitializationTime = 2.0;
        minKeyFrameCount      = 10;
    }
    else
    {
        minInitializationTime = 1.0;
        minKeyFrameCount      = 10;
    }

    if (p_atlas->getKeyFrameCount() < minKeyFrameCount)
        return;

    // Retrieve all keyframe in temporal order
    list<KeyFrame *> temporalKeyFrames;
    KeyFrame        *p_walkKeyFrame = p_currentKeyFrame;
    while (p_walkKeyFrame->p_prevKF)
    {
        temporalKeyFrames.push_front(p_walkKeyFrame);
        p_walkKeyFrame = p_walkKeyFrame->p_prevKF;
    }
    temporalKeyFrames.push_front(p_walkKeyFrame);
    vector<KeyFrame *> orderedKeyFrames(temporalKeyFrames.begin(),
                                        temporalKeyFrames.end());

    if (orderedKeyFrames.size() < minKeyFrameCount)
        return;

    firstTimestamp = orderedKeyFrames.front()->timeStamp;
    if (p_currentKeyFrame->timeStamp - firstTimestamp < minInitializationTime)
        return;

    double accumulatedTranslation_m = 0.0;
    for (std::size_t keyFrameIndex = 1; keyFrameIndex < orderedKeyFrames.size();
         ++keyFrameIndex)
    {
        Eigen::Vector3f cameraCenter{};
        if (orderedKeyFrames[keyFrameIndex]->getCameraCenter(cameraCenter) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f cameraCenter2{};
        if (orderedKeyFrames[keyFrameIndex - 1]->getCameraCenter(
                cameraCenter2) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        accumulatedTranslation_m += (cameraCenter - cameraCenter2).norm();
    }

    constexpr double minimumInitializationTranslation_m = 0.05;
    if (accumulatedTranslation_m < minimumInitializationTranslation_m)
        return;

    isInitializationInProgress = true;

    while (checkNewKeyFrames())
    {
        processNewKeyFrame();
        orderedKeyFrames.push_back(p_currentKeyFrame);
        temporalKeyFrames.push_back(p_currentKeyFrame);
    }

    const int orderedKeyFrameCount = orderedKeyFrames.size();
    IMU::Bias zeroImuBias(0, 0, 0, 0, 0, 0);

    // Compute and KF velocities mRwg estimation
    Map *p_currentKeyFrameMap = nullptr;
    if (p_currentKeyFrame->getMap(p_currentKeyFrameMap) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool isImuInitialized2{};
    if (p_currentKeyFrameMap->isImuInitialized(isImuInitialized2) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isImuInitialized2)
    {
        Eigen::Matrix3f Rwg;
        Eigen::Vector3f gravityDirection;
        gravityDirection.setZero();
        for (vector<KeyFrame *>::iterator orderedKeyFrameIt =
                 orderedKeyFrames.begin();
             orderedKeyFrameIt != orderedKeyFrames.end();
             orderedKeyFrameIt++)
        {
            if (!(*orderedKeyFrameIt)->p_imuPreintegrated)
                continue;
            if (!(*orderedKeyFrameIt)->p_prevKF)
                continue;

            Eigen::Matrix3f imuRotation{};
            if ((*orderedKeyFrameIt)->p_prevKF->getImuRotation(imuRotation) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getImuRotation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            gravityDirection -=
                imuRotation *
                (*orderedKeyFrameIt)
                    ->p_imuPreintegrated->getUpdatedDeltaVelocity();
            Eigen::Vector3f imuPosition{};
            if ((*orderedKeyFrameIt)->getImuPosition(imuPosition) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getImuPosition returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3f imuPosition2{};
            if ((*orderedKeyFrameIt)->p_prevKF->getImuPosition(imuPosition2) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getImuPosition returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3f keyFrameVelocity =
                (imuPosition - imuPosition2) /
                (*orderedKeyFrameIt)->p_imuPreintegrated->dT;
            if ((*orderedKeyFrameIt)->setVelocity(keyFrameVelocity) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if ((*orderedKeyFrameIt)->p_prevKF->setVelocity(keyFrameVelocity) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        gravityDirection = gravityDirection / gravityDirection.norm();
        Eigen::Vector3f referenceGravityDirection(0.0f, 0.0f, -1.0f);
        Eigen::Vector3f rotationAxis =
            referenceGravityDirection.cross(gravityDirection);
        const float rotationAxisNorm = rotationAxis.norm();
        const float gravityCosine =
            referenceGravityDirection.dot(gravityDirection);
        const float     rotationAngle = acos(gravityCosine);
        Eigen::Vector3f rotationVector(0.0f, 0.0f, 0.0f); // = v*ang/nv;
        if (rotationAxisNorm != 0 && !isnan(gravityCosine) &&
            !isnan(rotationAngle))
            rotationVector = rotationAxis * rotationAngle / rotationAxisNorm;
        Rwg                     = Sophus::SO3f::exp(rotationVector).matrix();
        mRwg                    = Rwg.cast<double>();
        initializationStartTime = p_currentKeyFrame->timeStamp - firstTimestamp;
    }
    else
    {
        mRwg = Eigen::Matrix3d::Identity();
        Eigen::Vector3f currentKeyFrameGyroBias{};
        if (p_currentKeyFrame->getGyroBias(currentKeyFrameGyroBias) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGyroBias returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        mbg = currentKeyFrameGyroBias.cast<double>();
        Eigen::Vector3f currentKeyFrameAccBias{};
        if (p_currentKeyFrame->getAccBias(currentKeyFrameAccBias) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAccBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        mba = currentKeyFrameAccBias.cast<double>();
    }

    scale = 1.0;

    initTime =
        p_tracker->lastFrame.timeStamp - orderedKeyFrames.front()->timeStamp;

    Optimizer::inertialOptimization(p_atlas->getCurrentMap(),
                                    mRwg,
                                    scale,
                                    mbg,
                                    mba,
                                    isMonocular,
                                    infoInertial,
                                    false,
                                    false,
                                    gyroPriorWeight_in,
                                    accelPriorWeight_in);

    if (scale < 1e-1)
    {
        cout << "scale too small" << endl;
        isInitializationInProgress = false;
        return;
    }

    // Before this line we are not changing the map
    {
        std::unique_lock<std::mutex> semanticUpdateLock =
            p_atlas->acquireSemanticUpdateLock();
        Map *p_activeMap = p_atlas->getCurrentMap();

        if (p_activeMap == nullptr)
        {
            isInitializationInProgress = false;
            return;
        }

        const bool         imuWasInitialized = p_atlas->isImuInitialized();
        unique_lock<mutex> mapUpdateLock(p_activeMap->mapUpdateMutex);
        if ((fabs(scale - 1.f) > 0.00001) || !isMonocular)
        {
            Sophus::SE3f Twg(mRwg.cast<float>().transpose(),
                             Eigen::Vector3f::Zero());
            if (p_activeMap->applyScaledRotation(Twg, scale, true) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: applyScaledRotation returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            IMU::Bias imuBias{};
            if (orderedKeyFrames[0]->getImuBias(imuBias) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getImuBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            p_tracker->updateFrameIMU(scale, imuBias, p_currentKeyFrame);
        }

        // Check if initialization OK
        if (!imuWasInitialized)
            for (int elementIndex = 0; elementIndex < orderedKeyFrameCount;
                 elementIndex++)
            {
                KeyFrame *p_orderedKeyFrame = orderedKeyFrames[elementIndex];
                p_orderedKeyFrame->isImu    = true;
            }
    }

    IMU::Bias imuBias2{};
    if (orderedKeyFrames[0]->getImuBias(imuBias2) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getImuBias returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    p_tracker->updateFrameIMU(1.0, imuBias2, p_currentKeyFrame);
    if (!p_atlas->isImuInitialized())
    {
        p_atlas->setImuInitialized();
        p_tracker->t0IMU         = p_tracker->currentFrame.timeStamp;
        p_currentKeyFrame->isImu = true;
    }

    if (shouldRunFullInertialBa_in)
    {
        if (accelPriorWeight_in != 0.f)
            Optimizer::fullInertialBA(p_atlas->getCurrentMap(),
                                      100,
                                      false,
                                      p_currentKeyFrame->id,
                                      nullptr,
                                      true,
                                      gyroPriorWeight_in,
                                      accelPriorWeight_in);
        else
            Optimizer::fullInertialBA(p_atlas->getCurrentMap(),
                                      100,
                                      false,
                                      p_currentKeyFrame->id,
                                      nullptr,
                                      false);
    }

    Verbose::printMess("Global Bundle Adjustment finished\nUpdating map ...",
                       Verbose::VERBOSITY_NORMAL);

    /* Keep semantic observations valid while corrected KFs are retired. */
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();

    // Get Map Mutex
    unique_lock<mutex> mapUpdateLock(p_atlas->getCurrentMap()->mapUpdateMutex);

    unsigned long globalBaId = p_currentKeyFrame->id;

    // Process keyframes in the queue
    while (checkNewKeyFrames())
    {
        processNewKeyFrame();
        orderedKeyFrames.push_back(p_currentKeyFrame);
        temporalKeyFrames.push_back(p_currentKeyFrame);
    }

    // Correct keyframes starting at map first keyframe
    list<KeyFrame *> keyFramesToCorrect(
        p_atlas->getCurrentMap()->keyFrameOrigins.begin(),
        p_atlas->getCurrentMap()->keyFrameOrigins.end());

    while (!keyFramesToCorrect.empty())
    {
        KeyFrame            *p_walkKeyFrame = keyFramesToCorrect.front();
        std::set<KeyFrame *> childKeyFrames{};
        if (p_walkKeyFrame->getChilds(childKeyFrames) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getChilds returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Sophus::SE3f Twc{};
        if (p_walkKeyFrame->getPoseInverse(Twc) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (set<KeyFrame *>::const_iterator childKeyFrameIt =
                 childKeyFrames.begin();
             childKeyFrameIt != childKeyFrames.end();
             childKeyFrameIt++)
        {
            KeyFrame *p_childKeyFrame = *childKeyFrameIt;
            bool      childKeyFrameIsBad{};
            if (!(!p_childKeyFrame) &&
                p_childKeyFrame->isBad(childKeyFrameIsBad) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!p_childKeyFrame || childKeyFrameIsBad)
                continue;

            if (p_childKeyFrame->baGlobalKeyFrameId != globalBaId)
            {
                Sophus::SE3f childKeyFramePose{};
                if (p_childKeyFrame->getPose(childKeyFramePose) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Sophus::SE3f Tchildc    = childKeyFramePose * Twc;
                p_childKeyFrame->tcwGBA = Tchildc * p_walkKeyFrame->tcwGBA;

                Sophus::SE3f childKeyFramePose2{};
                if (p_childKeyFrame->getPose(childKeyFramePose2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Sophus::SO3f Rcor = p_childKeyFrame->tcwGBA.so3().inverse() *
                                    childKeyFramePose2.so3();
                bool childKeyFrameIsVelocitySet{};
                if (p_childKeyFrame->isVelocitySet(
                        childKeyFrameIsVelocitySet) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: isVelocitySet returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (childKeyFrameIsVelocitySet)
                {
                    Eigen::Vector3f childKeyFrameVelocity{};
                    if (p_childKeyFrame->getVelocity(childKeyFrameVelocity) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getVelocity returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    p_childKeyFrame->vwbGBA = Rcor * childKeyFrameVelocity;
                }
                else
                {
                    Verbose::printMess("Child velocity empty!! ",
                                       Verbose::VERBOSITY_NORMAL);
                }

                IMU::Bias childKeyFrameImuBias{};
                if (p_childKeyFrame->getImuBias(childKeyFrameImuBias) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getImuBias returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                p_childKeyFrame->biasGBA            = childKeyFrameImuBias;
                p_childKeyFrame->baGlobalKeyFrameId = globalBaId;
            }
            keyFramesToCorrect.push_back(p_childKeyFrame);
        }

        Sophus::SE3f walkKeyFramePose{};
        if (p_walkKeyFrame->getPose(walkKeyFramePose) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        p_walkKeyFrame->tcwBefGBA = walkKeyFramePose;
        if (p_walkKeyFrame->setPose(p_walkKeyFrame->tcwGBA) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        if (p_walkKeyFrame->isImu)
        {
            Eigen::Vector3f walkKeyFrameVelocity{};
            if (p_walkKeyFrame->getVelocity(walkKeyFrameVelocity) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            p_walkKeyFrame->vwbBefGBA = walkKeyFrameVelocity;
            if (p_walkKeyFrame->setVelocity(p_walkKeyFrame->vwbGBA) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_walkKeyFrame->setNewBias(p_walkKeyFrame->biasGBA) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setNewBias returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {
            cout << "KF " << p_walkKeyFrame->id << " not set to inertial!! \n";
        }

        keyFramesToCorrect.pop_front();
    }

    // Correct MapPoints
    std::vector<MapPoint *> allMapPoints{};
    if (p_atlas->getCurrentMap()->getAllMapPoints(allMapPoints) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    for (size_t elementIndex = 0; elementIndex < allMapPoints.size();
         elementIndex++)
    {
        MapPoint *p_mapPoint = allMapPoints[elementIndex];

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

        if (p_mapPoint->baGlobalKeyFrameId == globalBaId)
        {
            // If optimized by Global BA, just update
            if (p_mapPoint->setWorldPos(p_mapPoint->posGBA) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {
            // Update according to the correction of its reference keyframe
            KeyFrame *p_referenceKeyFrame = nullptr;
            if (p_mapPoint->getReferenceKeyFrame(p_referenceKeyFrame) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getReferenceKeyFrame returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (p_referenceKeyFrame->baGlobalKeyFrameId != globalBaId)
                continue;

            // Map to non-corrected camera
            Eigen::Vector3f mapPointWorldPos{};
            if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3f Xc =
                p_referenceKeyFrame->tcwBefGBA * mapPointWorldPos;

            // Backproject using corrected camera
            Sophus::SE3f referenceKeyFramePoseInverse{};
            if (p_referenceKeyFrame->getPoseInverse(
                    referenceKeyFramePoseInverse) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapPoint->setWorldPos(referenceKeyFramePoseInverse * Xc) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setWorldPos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    Verbose::printMess("Map updated!", Verbose::VERBOSITY_NORMAL);

    keyFrameCount = orderedKeyFrames.size();
    initIndex++;

    for (list<KeyFrame *>::iterator newKeyFrameIt  = newKeyFrames.begin(),
                                    newKeyFrameEnd = newKeyFrames.end();
         newKeyFrameIt != newKeyFrameEnd;
         newKeyFrameIt++)
    {
        if ((*newKeyFrameIt)->setBadFlag() !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setBadFlag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        delete *newKeyFrameIt;
    }
    newKeyFrames.clear();

    p_tracker->state           = Tracking::OK;
    isInitializationInProgress = false;

    Map *p_currentKeyFrameMap2 = nullptr;
    if (p_currentKeyFrame->getMap(p_currentKeyFrameMap2) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_currentKeyFrameMap2->increaseChangeIndex() !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: increaseChangeIndex returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return;
}

} // namespace core
} // namespace vs_graphs
