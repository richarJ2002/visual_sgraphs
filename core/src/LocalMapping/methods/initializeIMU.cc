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

#include <mutex>

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
        accumulatedTranslation_m +=
            (orderedKeyFrames[keyFrameIndex]->getCameraCenter() -
             orderedKeyFrames[keyFrameIndex - 1]->getCameraCenter())
                .norm();
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
    if (!p_currentKeyFrame->getMap()->isImuInitialized())
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

            gravityDirection -=
                (*orderedKeyFrameIt)->p_prevKF->getImuRotation() *
                (*orderedKeyFrameIt)
                    ->p_imuPreintegrated->getUpdatedDeltaVelocity();
            Eigen::Vector3f keyFrameVelocity =
                ((*orderedKeyFrameIt)->getImuPosition() -
                 (*orderedKeyFrameIt)->p_prevKF->getImuPosition()) /
                (*orderedKeyFrameIt)->p_imuPreintegrated->dT;
            (*orderedKeyFrameIt)->setVelocity(keyFrameVelocity);
            (*orderedKeyFrameIt)->p_prevKF->setVelocity(keyFrameVelocity);
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
        mbg  = p_currentKeyFrame->getGyroBias().cast<double>();
        mba  = p_currentKeyFrame->getAccBias().cast<double>();
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
            p_activeMap->applyScaledRotation(Twg, scale, true);
            p_tracker->updateFrameIMU(scale,
                                      orderedKeyFrames[0]->getImuBias(),
                                      p_currentKeyFrame);
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

    p_tracker->updateFrameIMU(1.0,
                              orderedKeyFrames[0]->getImuBias(),
                              p_currentKeyFrame);
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
        KeyFrame             *p_walkKeyFrame = keyFramesToCorrect.front();
        const set<KeyFrame *> childKeyFrames = p_walkKeyFrame->getChilds();
        Sophus::SE3f          Twc            = p_walkKeyFrame->getPoseInverse();
        for (set<KeyFrame *>::const_iterator childKeyFrameIt =
                 childKeyFrames.begin();
             childKeyFrameIt != childKeyFrames.end();
             childKeyFrameIt++)
        {
            KeyFrame *p_childKeyFrame = *childKeyFrameIt;
            if (!p_childKeyFrame || p_childKeyFrame->isBad())
                continue;

            if (p_childKeyFrame->baGlobalKeyFrameId != globalBaId)
            {
                Sophus::SE3f Tchildc    = p_childKeyFrame->getPose() * Twc;
                p_childKeyFrame->tcwGBA = Tchildc * p_walkKeyFrame->tcwGBA;

                Sophus::SO3f Rcor = p_childKeyFrame->tcwGBA.so3().inverse() *
                                    p_childKeyFrame->getPose().so3();
                if (p_childKeyFrame->isVelocitySet())
                {
                    p_childKeyFrame->vwbGBA =
                        Rcor * p_childKeyFrame->getVelocity();
                }
                else
                {
                    Verbose::printMess("Child velocity empty!! ",
                                       Verbose::VERBOSITY_NORMAL);
                }

                p_childKeyFrame->biasGBA = p_childKeyFrame->getImuBias();
                p_childKeyFrame->baGlobalKeyFrameId = globalBaId;
            }
            keyFramesToCorrect.push_back(p_childKeyFrame);
        }

        p_walkKeyFrame->tcwBefGBA = p_walkKeyFrame->getPose();
        p_walkKeyFrame->setPose(p_walkKeyFrame->tcwGBA);

        if (p_walkKeyFrame->isImu)
        {
            p_walkKeyFrame->vwbBefGBA = p_walkKeyFrame->getVelocity();
            p_walkKeyFrame->setVelocity(p_walkKeyFrame->vwbGBA);
            p_walkKeyFrame->setNewBias(p_walkKeyFrame->biasGBA);
        }
        else
        {
            cout << "KF " << p_walkKeyFrame->id << " not set to inertial!! \n";
        }

        keyFramesToCorrect.pop_front();
    }

    // Correct MapPoints
    const vector<MapPoint *> allMapPoints =
        p_atlas->getCurrentMap()->getAllMapPoints();

    for (size_t elementIndex = 0; elementIndex < allMapPoints.size();
         elementIndex++)
    {
        MapPoint *p_mapPoint = allMapPoints[elementIndex];

        if (p_mapPoint->isBad())
            continue;

        if (p_mapPoint->baGlobalKeyFrameId == globalBaId)
        {
            // If optimized by Global BA, just update
            p_mapPoint->setWorldPos(p_mapPoint->posGBA);
        }
        else
        {
            // Update according to the correction of its reference keyframe
            KeyFrame *p_referenceKeyFrame = p_mapPoint->getReferenceKeyFrame();

            if (p_referenceKeyFrame->baGlobalKeyFrameId != globalBaId)
                continue;

            // Map to non-corrected camera
            Eigen::Vector3f Xc =
                p_referenceKeyFrame->tcwBefGBA * p_mapPoint->getWorldPos();

            // Backproject using corrected camera
            p_mapPoint->setWorldPos(p_referenceKeyFrame->getPoseInverse() * Xc);
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
        (*newKeyFrameIt)->setBadFlag();
        delete *newKeyFrameIt;
    }
    newKeyFrames.clear();

    p_tracker->state           = Tracking::OK;
    isInitializationInProgress = false;

    p_currentKeyFrame->getMap()->increaseChangeIndex();

    return;
}

} // namespace core
} // namespace vs_graphs
