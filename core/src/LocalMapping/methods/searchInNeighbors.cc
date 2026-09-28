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

#include "ORBmatcher.h"

namespace vs_graphs
{
namespace core
{

void LocalMapping::searchInNeighbors()
{
    // Retrieve neighbor keyframes
    int neighborKeyFrameCount = 10;
    if (isMonocular)
        neighborKeyFrameCount = 30;
    const vector<KeyFrame *> neighborKeyFrames =
        p_currentKeyFrame->getBestCovisibilityKeyFrames(neighborKeyFrameCount);
    vector<KeyFrame *> targetKeyFrames;
    for (vector<KeyFrame *>::const_iterator
             targetKeyFrameIt  = neighborKeyFrames.begin(),
             targetKeyFrameEnd = neighborKeyFrames.end();
         targetKeyFrameIt != targetKeyFrameEnd;
         targetKeyFrameIt++)
    {
        KeyFrame *p_targetKeyFrame = *targetKeyFrameIt;
        if (p_targetKeyFrame->isBad() ||
            p_targetKeyFrame->fuseTargetKeyFrameId == p_currentKeyFrame->id)
            continue;
        targetKeyFrames.push_back(p_targetKeyFrame);
        p_targetKeyFrame->fuseTargetKeyFrameId = p_currentKeyFrame->id;
    }

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    for (int elementIndex = 0, targetKeyFrameCount = targetKeyFrames.size();
         elementIndex < targetKeyFrameCount;
         elementIndex++)
    {
        const vector<KeyFrame *> secondNeighborKeyFrames =
            targetKeyFrames[elementIndex]->getBestCovisibilityKeyFrames(20);
        for (vector<KeyFrame *>::const_iterator
                 secondNeighborKeyFrameIt  = secondNeighborKeyFrames.begin(),
                 secondNeighborKeyFrameEnd = secondNeighborKeyFrames.end();
             secondNeighborKeyFrameIt != secondNeighborKeyFrameEnd;
             secondNeighborKeyFrameIt++)
        {
            KeyFrame *p_secondNeighborKeyFrame = *secondNeighborKeyFrameIt;
            if (p_secondNeighborKeyFrame->isBad() ||
                p_secondNeighborKeyFrame->fuseTargetKeyFrameId ==
                    p_currentKeyFrame->id ||
                p_secondNeighborKeyFrame->id == p_currentKeyFrame->id)
                continue;
            targetKeyFrames.push_back(p_secondNeighborKeyFrame);
            p_secondNeighborKeyFrame->fuseTargetKeyFrameId =
                p_currentKeyFrame->id;
        }
        if (shouldAbortBa)
            break;
    }

    // Extend to temporal neighbors
    if (isInertial)
    {
        KeyFrame *p_targetKeyFrame = p_currentKeyFrame->p_prevKF;
        while (targetKeyFrames.size() < 20 && p_targetKeyFrame)
        {
            if (p_targetKeyFrame->isBad() ||
                p_targetKeyFrame->fuseTargetKeyFrameId == p_currentKeyFrame->id)
            {
                p_targetKeyFrame = p_targetKeyFrame->p_prevKF;
                continue;
            }
            targetKeyFrames.push_back(p_targetKeyFrame);
            p_targetKeyFrame->fuseTargetKeyFrameId = p_currentKeyFrame->id;
            p_targetKeyFrame                       = p_targetKeyFrame->p_prevKF;
        }
    }

    // Search matches by projection from current KF in target KFs
    ORBmatcher         matcher;
    vector<MapPoint *> currentMapPointMatches =
        p_currentKeyFrame->getMapPointMatches();
    for (vector<KeyFrame *>::iterator
             targetKeyFrameIt  = targetKeyFrames.begin(),
             targetKeyFrameEnd = targetKeyFrames.end();
         targetKeyFrameIt != targetKeyFrameEnd;
         targetKeyFrameIt++)
    {
        KeyFrame *p_targetKeyFrame = *targetKeyFrameIt;

        matcher.fuse(p_targetKeyFrame, currentMapPointMatches);
        if (p_targetKeyFrame->leftKeyPointCount != -1)
            matcher.fuse(p_targetKeyFrame, currentMapPointMatches, true);
    }

    if (shouldAbortBa)
        return;

    // Search matches by projection from target KFs in current KF
    vector<MapPoint *> fuseCandidateMapPoints;
    fuseCandidateMapPoints.reserve(targetKeyFrames.size() *
                                   currentMapPointMatches.size());

    for (vector<KeyFrame *>::iterator
             targetKeyFrameIt2  = targetKeyFrames.begin(),
             targetKeyFrameEnd2 = targetKeyFrames.end();
         targetKeyFrameIt2 != targetKeyFrameEnd2;
         targetKeyFrameIt2++)
    {
        KeyFrame *p_targetKeyFrame = *targetKeyFrameIt2;

        vector<MapPoint *> targetMapPoints =
            p_targetKeyFrame->getMapPointMatches();

        for (vector<MapPoint *>::iterator
                 targetMapPointIt  = targetMapPoints.begin(),
                 targetMapPointEnd = targetMapPoints.end();
             targetMapPointIt != targetMapPointEnd;
             targetMapPointIt++)
        {
            MapPoint *p_mapPoint = *targetMapPointIt;
            if (!p_mapPoint)
                continue;
            if (p_mapPoint->isBad() ||
                p_mapPoint->fuseCandidateKeyFrameId == p_currentKeyFrame->id)
                continue;
            p_mapPoint->fuseCandidateKeyFrameId = p_currentKeyFrame->id;
            fuseCandidateMapPoints.push_back(p_mapPoint);
        }
    }

    matcher.fuse(p_currentKeyFrame, fuseCandidateMapPoints);
    if (p_currentKeyFrame->leftKeyPointCount != -1)
        matcher.fuse(p_currentKeyFrame, fuseCandidateMapPoints, true);

    // Update points
    currentMapPointMatches = p_currentKeyFrame->getMapPointMatches();
    for (size_t elementIndex = 0, mapPointCount = currentMapPointMatches.size();
         elementIndex < mapPointCount;
         elementIndex++)
    {
        MapPoint *p_mapPoint = currentMapPointMatches[elementIndex];
        if (p_mapPoint)
        {
            if (!p_mapPoint->isBad())
            {
                p_mapPoint->computeDistinctiveDescriptors();
                p_mapPoint->updateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    p_currentKeyFrame->updateConnections();
}

} // namespace core
} // namespace vs_graphs
