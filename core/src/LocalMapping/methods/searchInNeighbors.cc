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
    int nn = 10;
    if (monocular)
        nn = 30;
    const vector<KeyFrame *> vpNeighKFs =
        p_currentKeyFrame->getBestCovisibilityKeyFrames(nn);
    vector<KeyFrame *> vpTargetKFs;
    for (vector<KeyFrame *>::const_iterator vit  = vpNeighKFs.begin(),
                                            vend = vpNeighKFs.end();
         vit != vend;
         vit++)
    {
        KeyFrame *pKFi = *vit;
        if (pKFi->isBad() ||
            pKFi->fuseTargetKeyFrameId == p_currentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->fuseTargetKeyFrameId = p_currentKeyFrame->mnId;
    }

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    for (int i = 0, imax = vpTargetKFs.size(); i < imax; i++)
    {
        const vector<KeyFrame *> vpSecondNeighKFs =
            vpTargetKFs[i]->getBestCovisibilityKeyFrames(20);
        for (vector<KeyFrame *>::const_iterator vit2 = vpSecondNeighKFs.begin(),
                                                vend2 = vpSecondNeighKFs.end();
             vit2 != vend2;
             vit2++)
        {
            KeyFrame *pKFi2 = *vit2;
            if (pKFi2->isBad() ||
                pKFi2->fuseTargetKeyFrameId == p_currentKeyFrame->mnId ||
                pKFi2->mnId == p_currentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
            pKFi2->fuseTargetKeyFrameId = p_currentKeyFrame->mnId;
        }
        if (abortBA)
            break;
    }

    // Extend to temporal neighbors
    if (inertial)
    {
        KeyFrame *pKFi = p_currentKeyFrame->p_prevKF;
        while (vpTargetKFs.size() < 20 && pKFi)
        {
            if (pKFi->isBad() ||
                pKFi->fuseTargetKeyFrameId == p_currentKeyFrame->mnId)
            {
                pKFi = pKFi->p_prevKF;
                continue;
            }
            vpTargetKFs.push_back(pKFi);
            pKFi->fuseTargetKeyFrameId = p_currentKeyFrame->mnId;
            pKFi                       = pKFi->p_prevKF;
        }
    }

    // Search matches by projection from current KF in target KFs
    ORBmatcher         matcher;
    vector<MapPoint *> vpMapPointMatches =
        p_currentKeyFrame->getMapPointMatches();
    for (vector<KeyFrame *>::iterator vit  = vpTargetKFs.begin(),
                                      vend = vpTargetKFs.end();
         vit != vend;
         vit++)
    {
        KeyFrame *pKFi = *vit;

        matcher.fuse(pKFi, vpMapPointMatches);
        if (pKFi->Nleft != -1)
            matcher.fuse(pKFi, vpMapPointMatches, true);
    }

    if (abortBA)
        return;

    // Search matches by projection from target KFs in current KF
    vector<MapPoint *> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

    for (vector<KeyFrame *>::iterator vitKF  = vpTargetKFs.begin(),
                                      vendKF = vpTargetKFs.end();
         vitKF != vendKF;
         vitKF++)
    {
        KeyFrame *pKFi = *vitKF;

        vector<MapPoint *> vpMapPointsKFi = pKFi->getMapPointMatches();

        for (vector<MapPoint *>::iterator vitMP  = vpMapPointsKFi.begin(),
                                          vendMP = vpMapPointsKFi.end();
             vitMP != vendMP;
             vitMP++)
        {
            MapPoint *pMP = *vitMP;
            if (!pMP)
                continue;
            if (pMP->isBad() ||
                pMP->fuseCandidateKeyFrameId == p_currentKeyFrame->mnId)
                continue;
            pMP->fuseCandidateKeyFrameId = p_currentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.fuse(p_currentKeyFrame, vpFuseCandidates);
    if (p_currentKeyFrame->Nleft != -1)
        matcher.fuse(p_currentKeyFrame, vpFuseCandidates, true);

    // Update points
    vpMapPointMatches = p_currentKeyFrame->getMapPointMatches();
    for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++)
    {
        MapPoint *pMP = vpMapPointMatches[i];
        if (pMP)
        {
            if (!pMP->isBad())
            {
                pMP->computeDistinctiveDescriptors();
                pMP->updateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    p_currentKeyFrame->updateConnections();
}

} // namespace core
} // namespace vs_graphs
