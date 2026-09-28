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

namespace vs_graphs
{
namespace core
{

void LocalMapping::keyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees,
    // are seen in at least other 3 keyframes (in the same or finer scale) We
    // only consider close stereo points
    const int temporalWindowSize = 21;
    p_currentKeyFrame->updateBestCovisibles();
    vector<KeyFrame *> neighborKeyFrames =
        p_currentKeyFrame->getVectorCovisibleKeyFrames();

    float redundancyThreshold;
    if (!isInertial)
        redundancyThreshold = 0.9;
    else if (isMonocular)
        redundancyThreshold = 0.9;
    else
        redundancyThreshold = 0.5;

    const bool isImuInitialized       = p_atlas->isImuInitialized();
    int        processedKeyFrameCount = 0;

    // Compute the oldest keyframe in the optimizable inertial window.
    unsigned long lastOptimizableKeyFrameId = p_currentKeyFrame->id;
    if (isInertial)
    {
        int       temporalKeyFrameCount = 0;
        KeyFrame *p_oldestKeyFrame      = p_currentKeyFrame;
        while (temporalKeyFrameCount < temporalWindowSize &&
               p_oldestKeyFrame->p_prevKF)
        {
            p_oldestKeyFrame = p_oldestKeyFrame->p_prevKF;
            temporalKeyFrameCount++;
        }
        lastOptimizableKeyFrameId = p_oldestKeyFrame->id;
    }

    for (vector<KeyFrame *>::iterator
             neighborKeyFrameIt  = neighborKeyFrames.begin(),
             neighborKeyFrameEnd = neighborKeyFrames.end();
         neighborKeyFrameIt != neighborKeyFrameEnd;
         neighborKeyFrameIt++)
    {
        processedKeyFrameCount++;
        KeyFrame *p_neighborKeyFrame = *neighborKeyFrameIt;

        if ((p_neighborKeyFrame->id ==
             p_neighborKeyFrame->getMap()->getInitKeyFrameId()) ||
            p_neighborKeyFrame->isBad())
            continue;
        const vector<MapPoint *> neighborMapPoints =
            p_neighborKeyFrame->getMapPointMatches();

        int       observationCount          = 3;
        const int observationThreshold      = observationCount;
        int       redundantObservationCount = 0;
        int       validMapPointCount        = 0;
        for (size_t mapPointIndex = 0, mapPointCount = neighborMapPoints.size();
             mapPointIndex < mapPointCount;
             mapPointIndex++)
        {
            MapPoint *p_mapPoint = neighborMapPoints[mapPointIndex];
            if (p_mapPoint)
            {
                if (!p_mapPoint->isBad())
                {
                    if (!isMonocular)
                    {
                        if (p_neighborKeyFrame->depths[mapPointIndex] >
                                p_neighborKeyFrame->depthThreshold ||
                            p_neighborKeyFrame->depths[mapPointIndex] < 0)
                            continue;
                    }

                    validMapPointCount++;
                    if (p_mapPoint->getObservationCount() >
                        observationThreshold)
                    {
                        // Reached only when Nleft != -1, i.e. the fisheye
                        // stereo case, where Nleft is a keypoint count >= 0.
                        const int &scaleLevel =
                            (p_neighborKeyFrame->leftKeyPointCount == -1)
                                ? p_neighborKeyFrame
                                      ->keyPointsUndistorted[mapPointIndex]
                                      .octave
                            : (mapPointIndex <
                               static_cast<std::size_t>(
                                   p_neighborKeyFrame->leftKeyPointCount))
                                ? p_neighborKeyFrame->keyPoints[mapPointIndex]
                                      .octave
                                : p_neighborKeyFrame
                                      ->keyPointsRight[mapPointIndex]
                                      .octave;
                        const map<KeyFrame *, tuple<int, int>>
                            pointObservations = p_mapPoint->getObservations();
                        int observationCount  = 0;
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                                 observationIt  = pointObservations.begin(),
                                 observationEnd = pointObservations.end();
                             observationIt != observationEnd;
                             observationIt++)
                        {
                            KeyFrame *p_observingKeyFrame =
                                observationIt->first;
                            if (p_observingKeyFrame == p_neighborKeyFrame)
                                continue;
                            tuple<int, int> observationIndices =
                                observationIt->second;
                            int leftIndex          = get<0>(observationIndices),
                                rightIndex         = get<1>(observationIndices);
                            int observerScaleLevel = -1;
                            if (p_observingKeyFrame->leftKeyPointCount == -1)
                                observerScaleLevel =
                                    p_observingKeyFrame
                                        ->keyPointsUndistorted[leftIndex]
                                        .octave;
                            else
                            {
                                if (leftIndex != -1)
                                {
                                    observerScaleLevel =
                                        p_observingKeyFrame
                                            ->keyPoints[leftIndex]
                                            .octave;
                                }
                                if (rightIndex != -1)
                                {
                                    int rightLevel =
                                        p_observingKeyFrame
                                            ->keyPointsRight
                                                [rightIndex -
                                                 p_observingKeyFrame
                                                     ->leftKeyPointCount]
                                            .octave;
                                    observerScaleLevel =
                                        (observerScaleLevel == -1 ||
                                         observerScaleLevel > rightLevel)
                                            ? rightLevel
                                            : observerScaleLevel;
                                }
                            }

                            if (observerScaleLevel <= scaleLevel + 1)
                            {
                                observationCount++;
                                if (observationCount > observationThreshold)
                                    break;
                            }
                        }
                        if (observationCount > observationThreshold)
                        {
                            redundantObservationCount++;
                        }
                    }
                }
            }
        }

        if (redundantObservationCount >
            redundancyThreshold * validMapPointCount)
        {
            if (isInertial)
            {
                if (p_atlas->getKeyFrameCount() <= temporalWindowSize)
                    continue;

                if (p_neighborKeyFrame->id > (p_currentKeyFrame->id - 2))
                    continue;

                if (p_neighborKeyFrame->p_prevKF &&
                    p_neighborKeyFrame->p_nextKF)
                {
                    const float timeGap =
                        p_neighborKeyFrame->p_nextKF->timeStamp -
                        p_neighborKeyFrame->p_prevKF->timeStamp;

                    if ((isImuInitialized &&
                         (p_neighborKeyFrame->id < lastOptimizableKeyFrameId) &&
                         timeGap < 3.) ||
                        (timeGap < 0.5))
                    {
                        p_neighborKeyFrame->p_nextKF->p_imuPreintegrated
                            ->mergePrevious(
                                p_neighborKeyFrame->p_imuPreintegrated);
                        p_neighborKeyFrame->p_nextKF->p_prevKF =
                            p_neighborKeyFrame->p_prevKF;
                        p_neighborKeyFrame->p_prevKF->p_nextKF =
                            p_neighborKeyFrame->p_nextKF;
                        p_neighborKeyFrame->p_nextKF = nullptr;
                        p_neighborKeyFrame->p_prevKF = nullptr;
                        p_neighborKeyFrame->setBadFlag();
                    }
                    else if (!p_currentKeyFrame->getMap()->getInertialBA2() &&
                             ((p_neighborKeyFrame->getImuPosition() -
                               p_neighborKeyFrame->p_prevKF->getImuPosition())
                                  .norm() < 0.02) &&
                             (timeGap < 3))
                    {
                        p_neighborKeyFrame->p_nextKF->p_imuPreintegrated
                            ->mergePrevious(
                                p_neighborKeyFrame->p_imuPreintegrated);
                        p_neighborKeyFrame->p_nextKF->p_prevKF =
                            p_neighborKeyFrame->p_prevKF;
                        p_neighborKeyFrame->p_prevKF->p_nextKF =
                            p_neighborKeyFrame->p_nextKF;
                        p_neighborKeyFrame->p_nextKF = nullptr;
                        p_neighborKeyFrame->p_prevKF = nullptr;
                        p_neighborKeyFrame->setBadFlag();
                    }
                }
            }
            else
            {
                p_neighborKeyFrame->setBadFlag();
            }
        }
        if ((processedKeyFrameCount > 20 && shouldAbortBa) ||
            processedKeyFrameCount > 100)
        {
            break;
        }
    }
}

} // namespace core
} // namespace vs_graphs
