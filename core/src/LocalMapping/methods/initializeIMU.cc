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

void LocalMapping::initializeIMU(float priorG, float priorA, bool bFIBA)
{
    if (resetRequested)
        return;

    float       minTime;
    std::size_t nMinKF;
    if (monocular)
    {
        minTime = 2.0;
        nMinKF  = 10;
    }
    else
    {
        minTime = 1.0;
        nMinKF  = 10;
    }

    if (p_atlas->getKeyFrameCount() < nMinKF)
        return;

    // Retrieve all keyframe in temporal order
    list<KeyFrame *> lpKF;
    KeyFrame        *pKF = p_currentKeyFrame;
    while (pKF->p_prevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->p_prevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFrame *> vpKF(lpKF.begin(), lpKF.end());

    if (vpKF.size() < nMinKF)
        return;

    firstTimestamp = vpKF.front()->timeStamp;
    if (p_currentKeyFrame->timeStamp - firstTimestamp < minTime)
        return;

    double accumulatedTranslation_m = 0.0;
    for (std::size_t keyFrameIndex = 1; keyFrameIndex < vpKF.size();
         ++keyFrameIndex)
    {
        accumulatedTranslation_m += (vpKF[keyFrameIndex]->getCameraCenter() -
                                     vpKF[keyFrameIndex - 1]->getCameraCenter())
                                        .norm();
    }

    constexpr double minimumInitializationTranslation_m = 0.05;
    if (accumulatedTranslation_m < minimumInitializationTranslation_m)
        return;

    bInitializing = true;

    while (checkNewKeyFrames())
    {
        processNewKeyFrame();
        vpKF.push_back(p_currentKeyFrame);
        lpKF.push_back(p_currentKeyFrame);
    }

    const int N = vpKF.size();
    IMU::Bias b(0, 0, 0, 0, 0, 0);

    // Compute and KF velocities mRwg estimation
    if (!p_currentKeyFrame->getMap()->isImuInitialized())
    {
        Eigen::Matrix3f Rwg;
        Eigen::Vector3f dirG;
        dirG.setZero();
        for (vector<KeyFrame *>::iterator itKF = vpKF.begin();
             itKF != vpKF.end();
             itKF++)
        {
            if (!(*itKF)->p_imuPreintegrated)
                continue;
            if (!(*itKF)->p_prevKF)
                continue;

            dirG -= (*itKF)->p_prevKF->getImuRotation() *
                    (*itKF)->p_imuPreintegrated->getUpdatedDeltaVelocity();
            Eigen::Vector3f _vel = ((*itKF)->getImuPosition() -
                                    (*itKF)->p_prevKF->getImuPosition()) /
                                   (*itKF)->p_imuPreintegrated->dT;
            (*itKF)->setVelocity(_vel);
            (*itKF)->p_prevKF->setVelocity(_vel);
        }

        dirG = dirG / dirG.norm();
        Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
        Eigen::Vector3f v    = gI.cross(dirG);
        const float     nv   = v.norm();
        const float     cosg = gI.dot(dirG);
        const float     ang  = acos(cosg);
        Eigen::Vector3f vzg(0.0f, 0.0f, 0.0f); // = v*ang/nv;
        if (nv != 0 && !isnan(cosg) && !isnan(ang))
            vzg = v * ang / nv;
        Rwg                     = Sophus::SO3f::exp(vzg).matrix();
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

    initTime = p_tracker->lastFrame.timeStamp - vpKF.front()->timeStamp;

    Optimizer::inertialOptimization(p_atlas->getCurrentMap(),
                                    mRwg,
                                    scale,
                                    mbg,
                                    mba,
                                    monocular,
                                    infoInertial,
                                    false,
                                    false,
                                    priorG,
                                    priorA);

    if (scale < 1e-1)
    {
        cout << "scale too small" << endl;
        bInitializing = false;
        return;
    }

    // Before this line we are not changing the map
    {
        std::unique_lock<std::mutex> semanticUpdateLock =
            p_atlas->acquireSemanticUpdateLock();
        Map *p_activeMap = p_atlas->getCurrentMap();

        if (p_activeMap == nullptr)
        {
            bInitializing = false;
            return;
        }

        const bool         imuWasInitialized = p_atlas->isImuInitialized();
        unique_lock<mutex> lock(p_activeMap->mMutexMapUpdate);
        if ((fabs(scale - 1.f) > 0.00001) || !monocular)
        {
            Sophus::SE3f Twg(mRwg.cast<float>().transpose(),
                             Eigen::Vector3f::Zero());
            p_activeMap->applyScaledRotation(Twg, scale, true);
            p_tracker->updateFrameIMU(scale,
                                      vpKF[0]->getImuBias(),
                                      p_currentKeyFrame);
        }

        // Check if initialization OK
        if (!imuWasInitialized)
            for (int i = 0; i < N; i++)
            {
                KeyFrame *pKF2 = vpKF[i];
                pKF2->isImu    = true;
            }
    }

    p_tracker->updateFrameIMU(1.0, vpKF[0]->getImuBias(), p_currentKeyFrame);
    if (!p_atlas->isImuInitialized())
    {
        p_atlas->setImuInitialized();
        p_tracker->t0IMU         = p_tracker->currentFrame.timeStamp;
        p_currentKeyFrame->isImu = true;
    }

    if (bFIBA)
    {
        if (priorA != 0.f)
            Optimizer::fullInertialBA(p_atlas->getCurrentMap(),
                                      100,
                                      false,
                                      p_currentKeyFrame->mnId,
                                      nullptr,
                                      true,
                                      priorG,
                                      priorA);
        else
            Optimizer::fullInertialBA(p_atlas->getCurrentMap(),
                                      100,
                                      false,
                                      p_currentKeyFrame->mnId,
                                      nullptr,
                                      false);
    }

    Verbose::printMess("Global Bundle Adjustment finished\nUpdating map ...",
                       Verbose::VERBOSITY_NORMAL);

    /* Keep semantic observations valid while corrected KFs are retired. */
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();

    // Get Map Mutex
    unique_lock<mutex> lock(p_atlas->getCurrentMap()->mMutexMapUpdate);

    unsigned long GBAid = p_currentKeyFrame->mnId;

    // Process keyframes in the queue
    while (checkNewKeyFrames())
    {
        processNewKeyFrame();
        vpKF.push_back(p_currentKeyFrame);
        lpKF.push_back(p_currentKeyFrame);
    }

    // Correct keyframes starting at map first keyframe
    list<KeyFrame *> lpKFtoCheck(
        p_atlas->getCurrentMap()->keyFrameOrigins.begin(),
        p_atlas->getCurrentMap()->keyFrameOrigins.end());

    while (!lpKFtoCheck.empty())
    {
        KeyFrame             *pKF     = lpKFtoCheck.front();
        const set<KeyFrame *> sChilds = pKF->getChilds();
        Sophus::SE3f          Twc     = pKF->getPoseInverse();
        for (set<KeyFrame *>::const_iterator sit = sChilds.begin();
             sit != sChilds.end();
             sit++)
        {
            KeyFrame *pChild = *sit;
            if (!pChild || pChild->isBad())
                continue;

            if (pChild->baGlobalKeyFrameId != GBAid)
            {
                Sophus::SE3f Tchildc = pChild->getPose() * Twc;
                pChild->tcwGBA       = Tchildc * pKF->tcwGBA;

                Sophus::SO3f Rcor =
                    pChild->tcwGBA.so3().inverse() * pChild->getPose().so3();
                if (pChild->isVelocitySet())
                {
                    pChild->vwbGBA = Rcor * pChild->getVelocity();
                }
                else
                {
                    Verbose::printMess("Child velocity empty!! ",
                                       Verbose::VERBOSITY_NORMAL);
                }

                pChild->biasGBA            = pChild->getImuBias();
                pChild->baGlobalKeyFrameId = GBAid;
            }
            lpKFtoCheck.push_back(pChild);
        }

        pKF->tcwBefGBA = pKF->getPose();
        pKF->setPose(pKF->tcwGBA);

        if (pKF->isImu)
        {
            pKF->vwbBefGBA = pKF->getVelocity();
            pKF->setVelocity(pKF->vwbGBA);
            pKF->setNewBias(pKF->biasGBA);
        }
        else
        {
            cout << "KF " << pKF->mnId << " not set to inertial!! \n";
        }

        lpKFtoCheck.pop_front();
    }

    // Correct MapPoints
    const vector<MapPoint *> vpMPs =
        p_atlas->getCurrentMap()->getAllMapPoints();

    for (size_t i = 0; i < vpMPs.size(); i++)
    {
        MapPoint *pMP = vpMPs[i];

        if (pMP->isBad())
            continue;

        if (pMP->baGlobalKeyFrameId == GBAid)
        {
            // If optimized by Global BA, just update
            pMP->setWorldPos(pMP->posGBA);
        }
        else
        {
            // Update according to the correction of its reference keyframe
            KeyFrame *pRefKF = pMP->getReferenceKeyFrame();

            if (pRefKF->baGlobalKeyFrameId != GBAid)
                continue;

            // Map to non-corrected camera
            Eigen::Vector3f Xc = pRefKF->tcwBefGBA * pMP->getWorldPos();

            // Backproject using corrected camera
            pMP->setWorldPos(pRefKF->getPoseInverse() * Xc);
        }
    }

    Verbose::printMess("Map updated!", Verbose::VERBOSITY_NORMAL);

    keyFrameCount = vpKF.size();
    initIndex++;

    for (list<KeyFrame *>::iterator lit  = newKeyFrames.begin(),
                                    lend = newKeyFrames.end();
         lit != lend;
         lit++)
    {
        (*lit)->setBadFlag();
        delete *lit;
    }
    newKeyFrames.clear();

    p_tracker->state = Tracking::OK;
    bInitializing    = false;

    p_currentKeyFrame->getMap()->increaseChangeIndex();

    return;
}

} // namespace core
} // namespace vs_graphs
