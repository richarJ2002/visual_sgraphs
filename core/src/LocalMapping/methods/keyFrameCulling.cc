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
    const int Nd = 21;
    p_currentKeyFrame->updateBestCovisibles();
    vector<KeyFrame *> vpLocalKeyFrames =
        p_currentKeyFrame->getVectorCovisibleKeyFrames();

    float redundant_th;
    if (!inertial)
        redundant_th = 0.9;
    else if (monocular)
        redundant_th = 0.9;
    else
        redundant_th = 0.5;

    const bool bInitImu = p_atlas->isImuInitialized();
    int        count    = 0;

    // Compute the oldest keyframe in the optimizable inertial window.
    unsigned long lastOptimizableKeyFrameId = p_currentKeyFrame->mnId;
    if (inertial)
    {
        int       temporalKeyFrameCount = 0;
        KeyFrame *p_oldestKeyFrame      = p_currentKeyFrame;
        while (temporalKeyFrameCount < Nd && p_oldestKeyFrame->p_prevKF)
        {
            p_oldestKeyFrame = p_oldestKeyFrame->p_prevKF;
            temporalKeyFrameCount++;
        }
        lastOptimizableKeyFrameId = p_oldestKeyFrame->mnId;
    }

    for (vector<KeyFrame *>::iterator vit  = vpLocalKeyFrames.begin(),
                                      vend = vpLocalKeyFrames.end();
         vit != vend;
         vit++)
    {
        count++;
        KeyFrame *pKF = *vit;

        if ((pKF->mnId == pKF->getMap()->getInitKeyFrameId()) || pKF->isBad())
            continue;
        const vector<MapPoint *> vpMapPoints = pKF->getMapPointMatches();

        int       nObs                   = 3;
        const int thObs                  = nObs;
        int       nRedundantObservations = 0;
        int       nMPs                   = 0;
        for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++)
        {
            MapPoint *pMP = vpMapPoints[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    if (!monocular)
                    {
                        if (pKF->depths[i] > pKF->depthThreshold ||
                            pKF->depths[i] < 0)
                            continue;
                    }

                    nMPs++;
                    if (pMP->getObservationCount() > thObs)
                    {
                        // Reached only when Nleft != -1, i.e. the fisheye
                        // stereo case, where Nleft is a keypoint count >= 0.
                        const int &scaleLevel =
                            (pKF->Nleft == -1)
                                ? pKF->keyPointsUndistorted[i].octave
                            : (i < static_cast<std::size_t>(pKF->Nleft))
                                ? pKF->keyPoints[i].octave
                                : pKF->keyPointsRight[i].octave;
                        const map<KeyFrame *, tuple<int, int>> observations =
                            pMP->getObservations();
                        int nObs = 0;
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator
                                 mit  = observations.begin(),
                                 mend = observations.end();
                             mit != mend;
                             mit++)
                        {
                            KeyFrame *pKFi = mit->first;
                            if (pKFi == pKF)
                                continue;
                            tuple<int, int> indexes   = mit->second;
                            int             leftIndex = get<0>(indexes),
                                rightIndex            = get<1>(indexes);
                            int scaleLeveli           = -1;
                            if (pKFi->Nleft == -1)
                                scaleLeveli =
                                    pKFi->keyPointsUndistorted[leftIndex]
                                        .octave;
                            else
                            {
                                if (leftIndex != -1)
                                {
                                    scaleLeveli =
                                        pKFi->keyPoints[leftIndex].octave;
                                }
                                if (rightIndex != -1)
                                {
                                    int rightLevel =
                                        pKFi->keyPointsRight[rightIndex -
                                                             pKFi->Nleft]
                                            .octave;
                                    scaleLeveli = (scaleLeveli == -1 ||
                                                   scaleLeveli > rightLevel)
                                                      ? rightLevel
                                                      : scaleLeveli;
                                }
                            }

                            if (scaleLeveli <= scaleLevel + 1)
                            {
                                nObs++;
                                if (nObs > thObs)
                                    break;
                            }
                        }
                        if (nObs > thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if (nRedundantObservations > redundant_th * nMPs)
        {
            if (inertial)
            {
                if (p_atlas->getKeyFrameCount() <= Nd)
                    continue;

                if (pKF->mnId > (p_currentKeyFrame->mnId - 2))
                    continue;

                if (pKF->p_prevKF && pKF->p_nextKF)
                {
                    const float t =
                        pKF->p_nextKF->timeStamp - pKF->p_prevKF->timeStamp;

                    if ((bInitImu && (pKF->mnId < lastOptimizableKeyFrameId) &&
                         t < 3.) ||
                        (t < 0.5))
                    {
                        pKF->p_nextKF->p_imuPreintegrated->mergePrevious(
                            pKF->p_imuPreintegrated);
                        pKF->p_nextKF->p_prevKF = pKF->p_prevKF;
                        pKF->p_prevKF->p_nextKF = pKF->p_nextKF;
                        pKF->p_nextKF           = nullptr;
                        pKF->p_prevKF           = nullptr;
                        pKF->setBadFlag();
                    }
                    else if (!p_currentKeyFrame->getMap()->getInertialBA2() &&
                             ((pKF->getImuPosition() -
                               pKF->p_prevKF->getImuPosition())
                                  .norm() < 0.02) &&
                             (t < 3))
                    {
                        pKF->p_nextKF->p_imuPreintegrated->mergePrevious(
                            pKF->p_imuPreintegrated);
                        pKF->p_nextKF->p_prevKF = pKF->p_prevKF;
                        pKF->p_prevKF->p_nextKF = pKF->p_nextKF;
                        pKF->p_nextKF           = nullptr;
                        pKF->p_prevKF           = nullptr;
                        pKF->setBadFlag();
                    }
                }
            }
            else
            {
                pKF->setBadFlag();
            }
        }
        if ((count > 20 && abortBA) || count > 100)
        {
            break;
        }
    }
}

} // namespace core
} // namespace vs_graphs
