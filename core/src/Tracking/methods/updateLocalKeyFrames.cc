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

namespace vs_graphs
{
namespace core
{

void Tracking::updateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame *, int> keyframeCounter;
    if (!p_atlas->isImuInitialized() ||
        (currentFrame.mnId < lastRelocFrameId + 2))
    {
        for (int i = 0; i < currentFrame.N; i++)
        {
            MapPoint *pMP = currentFrame.mapPoints[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    const map<KeyFrame *, tuple<int, int>> observations =
                        pMP->getObservations();
                    for (map<KeyFrame *, tuple<int, int>>::const_iterator
                             it    = observations.begin(),
                             itend = observations.end();
                         it != itend;
                         it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    currentFrame.mapPoints[i] = nullptr;
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < lastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if (lastFrame.mapPoints[i])
            {
                MapPoint *pMP = lastFrame.mapPoints[i];
                if (!pMP)
                    continue;
                if (!pMP->isBad())
                {
                    const map<KeyFrame *, tuple<int, int>> observations =
                        pMP->getObservations();
                    for (map<KeyFrame *, tuple<int, int>>::const_iterator
                             it    = observations.begin(),
                             itend = observations.end();
                         it != itend;
                         it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    lastFrame.mapPoints[i] = nullptr;
                }
            }
        }
    }

    int       max    = 0;
    KeyFrame *pKFmax = static_cast<KeyFrame *>(nullptr);

    localKeyFrames.clear();
    localKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map.
    // Also check which keyframe shares most points
    for (map<KeyFrame *, int>::const_iterator it    = keyframeCounter.begin(),
                                              itEnd = keyframeCounter.end();
         it != itEnd;
         it++)
    {
        KeyFrame *pKF = it->first;

        if (pKF->isBad())
            continue;

        if (it->second > max)
        {
            max    = it->second;
            pKFmax = pKF;
        }

        localKeyFrames.push_back(pKF);
        pKF->trackReferenceFrameId = currentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to
    // already-included keyframes
    for (vector<KeyFrame *>::const_iterator itKF    = localKeyFrames.begin(),
                                            itEndKF = localKeyFrames.end();
         itKF != itEndKF;
         itKF++)
    {
        // Limit the number of keyframes - use configurable max (200 for
        // corridors)
        if (localKeyFrames.size() > static_cast<size_t>(maxKFsInLocalMap))
        {
            break;
        }

        KeyFrame *pKF = *itKF;

        const vector<KeyFrame *> vNeighs =
            pKF->getBestCovisibilityKeyFrames(10);

        for (vector<KeyFrame *>::const_iterator itNeighKF    = vNeighs.begin(),
                                                itEndNeighKF = vNeighs.end();
             itNeighKF != itEndNeighKF;
             itNeighKF++)
        {
            KeyFrame *pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad())
            {
                if (pNeighKF->trackReferenceFrameId != currentFrame.mnId)
                {
                    localKeyFrames.push_back(pNeighKF);
                    pNeighKF->trackReferenceFrameId = currentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame *> spChilds = pKF->getChilds();
        for (set<KeyFrame *>::const_iterator sit  = spChilds.begin(),
                                             send = spChilds.end();
             sit != send;
             sit++)
        {
            KeyFrame *pChildKF = *sit;
            if (!pChildKF->isBad())
            {
                if (pChildKF->trackReferenceFrameId != currentFrame.mnId)
                {
                    localKeyFrames.push_back(pChildKF);
                    pChildKF->trackReferenceFrameId = currentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame *pParent = pKF->getParent();
        if (pParent)
        {
            if (pParent->trackReferenceFrameId != currentFrame.mnId)
            {
                localKeyFrames.push_back(pParent);
                pParent->trackReferenceFrameId = currentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        localKeyFrames.size() < 80)
    {
        KeyFrame *tempKeyFrame = currentFrame.p_lastKeyFrame;

        const int Nd = 20;
        for (int i = 0; i < Nd; i++)
        {
            if (!tempKeyFrame)
                break;
            if (tempKeyFrame->trackReferenceFrameId != currentFrame.mnId)
            {
                localKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->trackReferenceFrameId = currentFrame.mnId;
                tempKeyFrame                        = tempKeyFrame->p_prevKF;
            }
        }
    }

    if (pKFmax)
    {
        p_referenceKF                    = pKFmax;
        currentFrame.p_referenceKeyFrame = p_referenceKF;
    }
}

} // namespace core
} // namespace vs_graphs
