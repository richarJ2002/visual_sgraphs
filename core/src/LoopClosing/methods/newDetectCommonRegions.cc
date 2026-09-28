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

#include <chrono>
#include <mutex>

namespace vs_graphs
{
namespace core
{

bool LoopClosing::newDetectCommonRegions()
{
    // To deactivate placerecognition. No loopclosing nor merging will be
    // performed
    if (!isLoopClosingActive)
    {
        return false;
    }

    {
        unique_lock<mutex> lock(loopQueueMutex);
        p_currentKF = loopKeyFrameQueue.front();
        loopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this
        // thread
        p_currentKF->setNotErase();
        p_currentKF->isInCurrentPlaceRecognition = true;

        p_lastMap = p_currentKF->getMap();
    }

    if (p_lastMap->isInertial() && !p_lastMap->getInertialBA2())
    {
        p_keyFrameDatabase->add(p_currentKF);
        p_currentKF->setErase();
        return false;
    }

    if (p_tracker->sensor == System::STEREO &&
        p_lastMap->getAllKeyFrames().size() < 5) // 12
    {
        p_keyFrameDatabase->add(p_currentKF);
        p_currentKF->setErase();
        return false;
    }

    if (p_lastMap->getAllKeyFrames().size() < 12)
    {
        p_keyFrameDatabase->add(p_currentKF);
        p_currentKF->setErase();
        return false;
    }

    // Check the last candidates with geometric validation
    //  Loop candidates
    bool isLoopDetectedInKeyFrame = false;
    bool shouldCheckSpatial       = false;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartEstSim3_1 =
        std::chrono::steady_clock::now();
#endif
    if (loopNumCoincidences > 0)
    {
        shouldCheckSpatial = true;
        // Find from the last KF candidates
        Sophus::SE3d mTcl =
            (p_currentKF->getPose() * p_loopLastCurrentKF->getPoseInverse())
                .cast<double>();
        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw                 = gScl * mg2oLoopSlw;
        int       projectionMatchCount = 0;
        vector<MapPoint *> matchedMapPoints;
        bool               isCommonRegionFound =
            detectAndReffineSim3FromLastKF(p_currentKF,
                                           p_loopMatchedKF,
                                           gScw,
                                           projectionMatchCount,
                                           loopMPs,
                                           matchedMapPoints);
        if (isCommonRegionFound)
        {

            isLoopDetectedInKeyFrame = true;

            loopNumCoincidences++;
            p_loopLastCurrentKF->setErase();
            p_loopLastCurrentKF = p_currentKF;
            mg2oLoopSlw         = gScw;
            loopMatchedMPs      = matchedMapPoints;

            isLoopDetected  = loopNumCoincidences >= 3;
            loopNumNotFound = 0;
        }
        else
        {
            isLoopDetectedInKeyFrame = false;

            loopNumNotFound++;
            if (loopNumNotFound >= 2)
            {
                recordLoopCorrectionEvent(false, "geometric_validation");
                p_loopLastCurrentKF->setErase();
                p_loopMatchedKF->setErase();
                loopNumCoincidences = 0;
                loopMatchedMPs.clear();
                loopMPs.clear();
                loopNumNotFound = 0;
            }
        }
    }

    // Merge candidates
    bool isMergeDetectedInKeyFrame = false;
    if (mergeNumCoincidences > 0)
    {
        // Find from the last KF candidates
        Sophus::SE3d mTcl =
            (p_currentKF->getPose() * p_mergeLastCurrentKF->getPoseInverse())
                .cast<double>();

        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw                 = gScl * mg2oMergeSlw;
        int       projectionMatchCount = 0;
        vector<MapPoint *> matchedMapPoints;
        bool               isCommonRegionFound =
            detectAndReffineSim3FromLastKF(p_currentKF,
                                           p_mergeMatchedKF,
                                           gScw,
                                           projectionMatchCount,
                                           mergeMPs,
                                           matchedMapPoints);
        if (isCommonRegionFound)
        {
            isMergeDetectedInKeyFrame = true;

            mergeNumCoincidences++;
            p_mergeLastCurrentKF->setErase();
            p_mergeLastCurrentKF = p_currentKF;
            mg2oMergeSlw         = gScw;
            mergeMatchedMPs      = matchedMapPoints;

            isMergeDetected = mergeNumCoincidences >= 3;
        }
        else
        {
            isMergeDetected           = false;
            isMergeDetectedInKeyFrame = false;

            mergeNumNotFound++;
            if (mergeNumNotFound >= 2)
            {
                p_mergeLastCurrentKF->setErase();
                p_mergeMatchedKF->setErase();
                mergeNumCoincidences = 0;
                mergeMatchedMPs.clear();
                mergeMPs.clear();
                mergeNumNotFound = 0;
            }
        }
    }
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndEstSim3_1 =
        std::chrono::steady_clock::now();

    double timeEstSim3 =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndEstSim3_1 - time_StartEstSim3_1)
            .count();
#endif

    if (isMergeDetected || isLoopDetected)
    {
#ifdef REGISTER_TIMES
        sim3EstimationTimes_ms.push_back(timeEstSim3);
#endif
        p_keyFrameDatabase->add(p_currentKF);
        return true;
    }

    // TODO: This is only necessary if we use a minimun score for pick the best
    // candidates
    const vector<KeyFrame *> connectedKeyFrames =
        p_currentKF->getVectorCovisibleKeyFrames();

    // Extract candidates from the bag of words
    vector<KeyFrame *> mergeBowCandidates, loopBowCandidates;
    if (!isMergeDetectedInKeyFrame || !isLoopDetectedInKeyFrame)
    {
        // Search in BoW
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point timeStartQuery =
            std::chrono::steady_clock::now();
#endif
        p_keyFrameDatabase->detectNBestCandidates(p_currentKF,
                                                  loopBowCandidates,
                                                  mergeBowCandidates,
                                                  3);
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point timeEndQuery =
            std::chrono::steady_clock::now();

        double timeDataQuery = std::chrono::duration_cast<
                                   std::chrono::duration<double, std::milli>>(
                                   timeEndQuery - timeStartQuery)
                                   .count();
        dataQueryTimes_ms.push_back(timeDataQuery);
#endif
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartEstSim3_2 =
        std::chrono::steady_clock::now();
#endif
    // Check the BoW candidates if the geometric candidate list is empty
    // Loop candidates
    if (!isLoopDetectedInKeyFrame && !loopBowCandidates.empty())
    {
        isLoopDetected = detectCommonRegionsFromBoW(loopBowCandidates,
                                                    p_loopMatchedKF,
                                                    p_loopLastCurrentKF,
                                                    mg2oLoopSlw,
                                                    loopNumCoincidences,
                                                    loopMPs,
                                                    loopMatchedMPs);
    }
    // Merge candidates
    if (!isMergeDetectedInKeyFrame && !mergeBowCandidates.empty())
    {
        isMergeDetected = detectCommonRegionsFromBoW(mergeBowCandidates,
                                                     p_mergeMatchedKF,
                                                     p_mergeLastCurrentKF,
                                                     mg2oMergeSlw,
                                                     mergeNumCoincidences,
                                                     mergeMPs,
                                                     mergeMatchedMPs);
    }

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndEstSim3_2 =
        std::chrono::steady_clock::now();

    timeEstSim3 +=
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndEstSim3_2 - time_StartEstSim3_2)
            .count();
    sim3EstimationTimes_ms.push_back(timeEstSim3);
#endif

    p_keyFrameDatabase->add(p_currentKF);

    if (isMergeDetected || isLoopDetected)
    {
        return true;
    }

    p_currentKF->setErase();
    p_currentKF->isInCurrentPlaceRecognition = false;

    return false;
}

} // namespace core
} // namespace vs_graphs
