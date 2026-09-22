/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

#include "ORBmatcher.h"

#include <limits.h>

#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <stdint-gcc.h>

namespace vs_graphs
{
namespace core
{

int ORBmatcher::searchByProjection(Frame                    &F,
                                   const vector<MapPoint *> &vpMapPoints,
                                   const float               th,
                                   const bool                bFarPoints,
                                   const float               thFarPoints)
{
    int nmatches = 0, left = 0, right = 0;

    const bool bFactor = th != 1.0;

    for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++)
    {
        MapPoint *pMP = vpMapPoints[iMP];
        if (!pMP->trackInView && !pMP->trackInViewR)
            continue;

        if (bFarPoints && pMP->trackDepth > thFarPoints)
            continue;

        if (pMP->isBad())
            continue;

        if (pMP->trackInView)
        {
            const int &nPredictedLevel = pMP->trackScaleLevel;

            // The size of the window will depend on the viewing direction
            float r = radiusByViewingCos(pMP->trackViewCos);

            if (bFactor)
                r *= th;

            const vector<size_t> vIndices =
                F.getFeaturesInArea(pMP->trackProjX,
                                    pMP->trackProjY,
                                    r * F.scaleFactors[nPredictedLevel],
                                    nPredictedLevel - 1,
                                    nPredictedLevel);

            if (!vIndices.empty())
            {
                const cv::Mat MPdescriptor = pMP->getDescriptor();

                int bestDist   = 256;
                int bestLevel  = -1;
                int bestDist2  = 256;
                int bestLevel2 = -1;
                int bestIdx    = -1;

                // Get best and second matches with near keypoints
                for (vector<size_t>::const_iterator vit  = vIndices.begin(),
                                                    vend = vIndices.end();
                     vit != vend;
                     vit++)
                {
                    const size_t idx = *vit;

                    if (F.mapPoints[idx])
                        if (F.mapPoints[idx]->getObservationCount() > 0)
                            continue;

                    if (F.Nleft == -1 && F.uRight[idx] > 0)
                    {
                        const float er = fabs(pMP->trackProjXR - F.uRight[idx]);
                        if (er > r * F.scaleFactors[nPredictedLevel])
                            continue;
                    }

                    const cv::Mat &d = F.descriptors.row(idx);

                    const int dist = computeDescriptorDistance(MPdescriptor, d);

                    if (dist < bestDist)
                    {
                        bestDist2  = bestDist;
                        bestDist   = dist;
                        bestLevel2 = bestLevel;
                        bestLevel =
                            (F.Nleft == -1) ? F.keyPointsUndistorted[idx].octave
                            : (idx < static_cast<size_t>(F.Nleft))
                                ? F.keyPoints[idx].octave
                                : F.keyPointsRight[idx - F.Nleft].octave;
                        bestIdx = idx;
                    }
                    else if (dist < bestDist2)
                    {
                        bestLevel2 =
                            (F.Nleft == -1) ? F.keyPointsUndistorted[idx].octave
                            : (idx < static_cast<size_t>(F.Nleft))
                                ? F.keyPoints[idx].octave
                                : F.keyPointsRight[idx - F.Nleft].octave;
                        bestDist2 = dist;
                    }
                }

                // Apply ratio to second match (only if best and second are in
                // the same scale level)
                if (bestDist <= TH_HIGH)
                {
                    if (bestLevel == bestLevel2 &&
                        bestDist > mfNNratio * bestDist2)
                        continue;

                    if (bestLevel != bestLevel2 ||
                        bestDist <= mfNNratio * bestDist2)
                    {
                        F.mapPoints[bestIdx] = pMP;

                        if (F.Nleft != -1 &&
                            F.leftToRightMatches[bestIdx] != -1)
                        { // Also match with the stereo observation at right
                          // camera
                            F.mapPoints[F.leftToRightMatches[bestIdx] +
                                        F.Nleft] = pMP;
                            nmatches++;
                            right++;
                        }

                        nmatches++;
                        left++;
                    }
                }
            }
        }

        if (F.Nleft != -1 && pMP->trackInViewR)
        {
            const int &nPredictedLevel = pMP->trackScaleLevelR;
            if (nPredictedLevel != -1)
            {
                float r = radiusByViewingCos(pMP->trackViewCosR);

                const vector<size_t> vIndices =
                    F.getFeaturesInArea(pMP->trackProjXR,
                                        pMP->trackProjYR,
                                        r * F.scaleFactors[nPredictedLevel],
                                        nPredictedLevel - 1,
                                        nPredictedLevel,
                                        true);

                if (vIndices.empty())
                    continue;

                const cv::Mat MPdescriptor = pMP->getDescriptor();

                int bestDist   = 256;
                int bestLevel  = -1;
                int bestDist2  = 256;
                int bestLevel2 = -1;
                int bestIdx    = -1;

                // Get best and second matches with near keypoints
                for (vector<size_t>::const_iterator vit  = vIndices.begin(),
                                                    vend = vIndices.end();
                     vit != vend;
                     vit++)
                {
                    const size_t idx = *vit;

                    if (F.mapPoints[idx + F.Nleft])
                        if (F.mapPoints[idx + F.Nleft]->getObservationCount() >
                            0)
                            continue;

                    const cv::Mat &d = F.descriptors.row(idx + F.Nleft);

                    const int dist = computeDescriptorDistance(MPdescriptor, d);

                    if (dist < bestDist)
                    {
                        bestDist2  = bestDist;
                        bestDist   = dist;
                        bestLevel2 = bestLevel;
                        bestLevel  = F.keyPointsRight[idx].octave;
                        bestIdx    = idx;
                    }
                    else if (dist < bestDist2)
                    {
                        bestLevel2 = F.keyPointsRight[idx].octave;
                        bestDist2  = dist;
                    }
                }

                // Apply ratio to second match (only if best and second are in
                // the same scale level)
                if (bestDist <= TH_HIGH)
                {
                    if (bestLevel == bestLevel2 &&
                        bestDist > mfNNratio * bestDist2)
                        continue;

                    if (F.Nleft != -1 && F.rightToLeftMatches[bestIdx] != -1)
                    { // Also match with the stereo observation at right camera
                        F.mapPoints[F.rightToLeftMatches[bestIdx]] = pMP;
                        nmatches++;
                        left++;
                    }

                    F.mapPoints[bestIdx + F.Nleft] = pMP;
                    nmatches++;
                    right++;
                }
            }
        }
    }
    return nmatches;
}

} // namespace core
} // namespace vs_graphs
