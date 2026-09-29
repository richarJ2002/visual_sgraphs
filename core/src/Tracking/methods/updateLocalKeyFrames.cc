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

#include "System.h"
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
        (currentFrame.id < lastRelocFrameId + 2))
    {
        for (int keyPointIndex = 0; keyPointIndex < currentFrame.keyPointCount;
             keyPointIndex++)
        {
            MapPoint *p_mapPoint = currentFrame.mapPoints[keyPointIndex];
            if (p_mapPoint)
            {
                if (!p_mapPoint->isBad())
                {
                    const map<KeyFrame *, tuple<int, int>> observations =
                        p_mapPoint->getObservations();
                    for (map<KeyFrame *, tuple<int, int>>::const_iterator
                             keyFrameCounterIt = observations.begin(),
                             itend             = observations.end();
                         keyFrameCounterIt != itend;
                         keyFrameCounterIt++)
                        keyframeCounter[keyFrameCounterIt->first]++;
                }
                else
                {
                    currentFrame.mapPoints[keyPointIndex] = nullptr;
                }
            }
        }
    }
    else
    {
        for (int keyPointIndex = 0; keyPointIndex < lastFrame.keyPointCount;
             keyPointIndex++)
        {
            // Using lastframe since current frame has not matches yet
            if (lastFrame.mapPoints[keyPointIndex])
            {
                MapPoint *p_mapPoint = lastFrame.mapPoints[keyPointIndex];
                if (!p_mapPoint)
                    continue;
                if (!p_mapPoint->isBad())
                {
                    const map<KeyFrame *, tuple<int, int>> observations =
                        p_mapPoint->getObservations();
                    for (map<KeyFrame *, tuple<int, int>>::const_iterator
                             keyFrameCounterIt = observations.begin(),
                             itend             = observations.end();
                         keyFrameCounterIt != itend;
                         keyFrameCounterIt++)
                        keyframeCounter[keyFrameCounterIt->first]++;
                }
                else
                {
                    // MODIFICATION
                    lastFrame.mapPoints[keyPointIndex] = nullptr;
                }
            }
        }
    }

    int       maximum           = 0;
    KeyFrame *p_keyFrameMaximum = static_cast<KeyFrame *>(nullptr);

    localKeyFrames.clear();
    localKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map.
    // Also check which keyframe shares most points
    for (map<KeyFrame *, int>::const_iterator
             keyFrameCounterIt = keyframeCounter.begin(),
             itEnd             = keyframeCounter.end();
         keyFrameCounterIt != itEnd;
         keyFrameCounterIt++)
    {
        KeyFrame *p_keyFrame = keyFrameCounterIt->first;

        if (p_keyFrame->isBad())
            continue;

        if (keyFrameCounterIt->second > maximum)
        {
            maximum           = keyFrameCounterIt->second;
            p_keyFrameMaximum = p_keyFrame;
        }

        localKeyFrames.push_back(p_keyFrame);
        p_keyFrame->trackReferenceFrameId = currentFrame.id;
    }

    // Include also some not-already-included keyframes that are neighbors to
    // already-included keyframes
    for (vector<KeyFrame *>::const_iterator
             itKeyFrame    = localKeyFrames.begin(),
             itEndKeyFrame = localKeyFrames.end();
         itKeyFrame != itEndKeyFrame;
         itKeyFrame++)
    {
        // Limit the number of keyframes - use configurable max (200 for
        // corridors)
        if (localKeyFrames.size() > static_cast<size_t>(maxKFsInLocalMap))
        {
            break;
        }

        KeyFrame *p_keyFrame = *itKeyFrame;

        const vector<KeyFrame *> neighbors =
            p_keyFrame->getBestCovisibilityKeyFrames(10);

        for (vector<KeyFrame *>::const_iterator
                 itNeighborKeyFrame    = neighbors.begin(),
                 itEndNeighborKeyFrame = neighbors.end();
             itNeighborKeyFrame != itEndNeighborKeyFrame;
             itNeighborKeyFrame++)
        {
            KeyFrame *p_neighborKeyFrame = *itNeighborKeyFrame;
            if (!p_neighborKeyFrame->isBad())
            {
                if (p_neighborKeyFrame->trackReferenceFrameId !=
                    currentFrame.id)
                {
                    localKeyFrames.push_back(p_neighborKeyFrame);
                    p_neighborKeyFrame->trackReferenceFrameId = currentFrame.id;
                    break;
                }
            }
        }

        const set<KeyFrame *> childs = p_keyFrame->getChilds();
        for (set<KeyFrame *>::const_iterator sit  = childs.begin(),
                                             send = childs.end();
             sit != send;
             sit++)
        {
            KeyFrame *p_childKeyFrame = *sit;
            if (!p_childKeyFrame->isBad())
            {
                if (p_childKeyFrame->trackReferenceFrameId != currentFrame.id)
                {
                    localKeyFrames.push_back(p_childKeyFrame);
                    p_childKeyFrame->trackReferenceFrameId = currentFrame.id;
                    break;
                }
            }
        }

        KeyFrame *p_parent = p_keyFrame->getParent();
        if (p_parent)
        {
            if (p_parent->trackReferenceFrameId != currentFrame.id)
            {
                localKeyFrames.push_back(p_parent);
                p_parent->trackReferenceFrameId = currentFrame.id;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        localKeyFrames.size() < 80)
    {
        KeyFrame *p_tempKeyFrame = currentFrame.p_lastKeyFrame;

        const int Nd = 20;
        for (int keyPointIndex = 0; keyPointIndex < Nd; keyPointIndex++)
        {
            if (!p_tempKeyFrame)
                break;
            if (p_tempKeyFrame->trackReferenceFrameId != currentFrame.id)
            {
                localKeyFrames.push_back(p_tempKeyFrame);
                p_tempKeyFrame->trackReferenceFrameId = currentFrame.id;
                p_tempKeyFrame = p_tempKeyFrame->p_prevKF;
            }
        }
    }

    if (p_keyFrameMaximum)
    {
        p_referenceKF                    = p_keyFrameMaximum;
        currentFrame.p_referenceKeyFrame = p_referenceKF;
    }
}

} // namespace core
} // namespace vs_graphs
