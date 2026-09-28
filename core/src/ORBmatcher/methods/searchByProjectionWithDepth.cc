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

int ORBmatcher::searchByProjectionWithDepth(
    Frame                    &F,
    const vector<MapPoint *> &vpMapPoints,
    const float               th,
    const bool                bFarPoints,
    const float               thFarPoints,
    const float               thDepth)
{
    int nmatches = 0, left = 0, right = 0;

    const bool isThresholdScaled = th != 1.0;

    for (size_t mapPointIndex = 0; mapPointIndex < vpMapPoints.size();
         mapPointIndex++)
    {
        MapPoint *p_mapPoint = vpMapPoints[mapPointIndex];
        if (!p_mapPoint->isTrackedInView && !p_mapPoint->isTrackedInRightView)
            continue;

        if (bFarPoints && p_mapPoint->trackDepth > thFarPoints)
            continue;

        if (p_mapPoint->isBad())
            continue;

        if (p_mapPoint->isTrackedInView)
        {
            const int &predictedLevelCount = p_mapPoint->trackScaleLevel;

            // The size of the window will depend on the viewing direction
            float r = radiusByViewingCos(p_mapPoint->trackViewCos);

            if (isThresholdScaled)
                r *= th;

            // DEPTH-GUIDED SEARCH: If the map point has a tracked depth,
            // reduce the search radius based on depth
            // For close points (depth < thDepth), use tighter search window
            // This helps in repetitive corridors where visual ambiguity is high
            if (p_mapPoint->trackDepth > 0)
            {
                float z = p_mapPoint->trackDepth;
                if (z < thDepth)
                {
                    // Close point: reduce search radius by 30% for tighter
                    // matching
                    r *= 0.7f;
                }
                else
                {
                    // Far point: slightly increase search radius
                    r *= 1.2f;
                }
            }

            const vector<size_t> indices =
                F.getFeaturesInArea(p_mapPoint->trackProjX,
                                    p_mapPoint->trackProjY,
                                    r * F.scaleFactors[predictedLevelCount],
                                    predictedLevelCount - 1,
                                    predictedLevelCount);

            if (!indices.empty())
            {
                const cv::Mat mapPointDescriptor = p_mapPoint->getDescriptor();

                int bestDistance  = 256;
                int bestLevel     = -1;
                int bestDistance2 = 256;
                int bestLevel2    = -1;
                int bestIndex     = -1;

                // Get best and second matches with near keypoints
                for (vector<size_t>::const_iterator vit  = indices.begin(),
                                                    vend = indices.end();
                     vit != vend;
                     vit++)
                {
                    const size_t featureIndex = *vit;

                    if (F.mapPoints[featureIndex])
                        if (F.mapPoints[featureIndex]->getObservationCount() >
                            0)
                            continue;

                    if (F.leftKeyPointCount == -1 && F.uRight[featureIndex] > 0)
                    {
                        const float er = fabs(p_mapPoint->trackProjXR -
                                              F.uRight[featureIndex]);
                        if (er > r * F.scaleFactors[predictedLevelCount])
                            continue;
                    }

                    const cv::Mat &d = F.descriptors.row(featureIndex);

                    const int distance =
                        computeDescriptorDistance(mapPointDescriptor, d);

                    if (distance < bestDistance)
                    {
                        bestDistance2 = bestDistance;
                        bestDistance  = distance;
                        bestLevel2    = bestLevel;
                        bestLevel =
                            (F.leftKeyPointCount == -1)
                                ? F.keyPointsUndistorted[featureIndex].octave
                            : (featureIndex <
                               static_cast<size_t>(F.leftKeyPointCount))
                                ? F.keyPoints[featureIndex].octave
                                : F.keyPointsRight[featureIndex -
                                                   F.leftKeyPointCount]
                                      .octave;
                        bestIndex = featureIndex;
                    }
                    else if (distance < bestDistance2)
                    {
                        bestLevel2 =
                            (F.leftKeyPointCount == -1)
                                ? F.keyPointsUndistorted[featureIndex].octave
                            : (featureIndex <
                               static_cast<size_t>(F.leftKeyPointCount))
                                ? F.keyPoints[featureIndex].octave
                                : F.keyPointsRight[featureIndex -
                                                   F.leftKeyPointCount]
                                      .octave;
                        bestDistance2 = distance;
                    }
                }

                // Apply ratio to second match (only if best and second are in
                // the same scale level)
                if (bestDistance <= TH_HIGH)
                {
                    if (bestLevel == bestLevel2 &&
                        bestDistance > nearestNeighborRatio * bestDistance2)
                        continue;

                    if (bestLevel != bestLevel2 ||
                        bestDistance <= nearestNeighborRatio * bestDistance2)
                    {
                        F.mapPoints[bestIndex] = p_mapPoint;

                        if (F.leftKeyPointCount != -1 &&
                            F.leftToRightMatches[bestIndex] != -1)
                        { // Also match with the stereo observation at right
                          // camera
                            F.mapPoints[F.leftToRightMatches[bestIndex] +
                                        F.leftKeyPointCount] = p_mapPoint;
                            nmatches++;
                            right++;
                        }

                        nmatches++;
                        left++;
                    }
                }
            }
        }

        if (F.leftKeyPointCount != -1 && p_mapPoint->isTrackedInRightView)
        {
            const int &predictedLevelCount = p_mapPoint->trackScaleLevelR;
            if (predictedLevelCount != -1)
            {
                float r = radiusByViewingCos(p_mapPoint->trackViewCosR);

                // Apply same depth-guided radius adjustment for right camera
                if (p_mapPoint->trackDepthR > 0)
                {
                    float z = p_mapPoint->trackDepthR;
                    if (z < thDepth)
                        r *= 0.7f;
                    else
                        r *= 1.2f;
                }

                const vector<size_t> indices =
                    F.getFeaturesInArea(p_mapPoint->trackProjXR,
                                        p_mapPoint->trackProjYR,
                                        r * F.scaleFactors[predictedLevelCount],
                                        predictedLevelCount - 1,
                                        predictedLevelCount,
                                        true);

                if (indices.empty())
                    continue;

                const cv::Mat mapPointDescriptor = p_mapPoint->getDescriptor();

                int bestDistance  = 256;
                int bestLevel     = -1;
                int bestDistance2 = 256;
                int bestLevel2    = -1;
                int bestIndex     = -1;

                // Get best and second matches with near keypoints
                for (vector<size_t>::const_iterator vit  = indices.begin(),
                                                    vend = indices.end();
                     vit != vend;
                     vit++)
                {
                    const size_t featureIndex = *vit;

                    if (F.mapPoints[featureIndex + F.leftKeyPointCount])
                        if (F.mapPoints[featureIndex + F.leftKeyPointCount]
                                ->getObservationCount() > 0)
                            continue;

                    const cv::Mat &d =
                        F.descriptors.row(featureIndex + F.leftKeyPointCount);

                    const int distance =
                        computeDescriptorDistance(mapPointDescriptor, d);

                    if (distance < bestDistance)
                    {
                        bestDistance2 = bestDistance;
                        bestDistance  = distance;
                        bestLevel2    = bestLevel;
                        bestLevel     = F.keyPointsRight[featureIndex].octave;
                        bestIndex     = featureIndex;
                    }
                    else if (distance < bestDistance2)
                    {
                        bestLevel2    = F.keyPointsRight[featureIndex].octave;
                        bestDistance2 = distance;
                    }
                }

                // Apply ratio to second match (only if best and second are in
                // the same scale level)
                if (bestDistance <= TH_HIGH)
                {
                    if (bestLevel == bestLevel2 &&
                        bestDistance > nearestNeighborRatio * bestDistance2)
                        continue;

                    if (F.leftKeyPointCount != -1 &&
                        F.rightToLeftMatches[bestIndex] != -1)
                    { // Also match with the stereo observation at right camera
                        F.mapPoints[F.rightToLeftMatches[bestIndex]] =
                            p_mapPoint;
                        nmatches++;
                        left++;
                    }

                    F.mapPoints[bestIndex + F.leftKeyPointCount] = p_mapPoint;
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
