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

int ORBmatcher::searchBySim3(KeyFrame                *pKF1,
                             KeyFrame                *pKF2,
                             std::vector<MapPoint *> &matches12_inout,
                             const Sophus::Sim3f     &S12,
                             const float              th)
{
    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    // Camera 1 & 2 from world
    Sophus::SE3f T1w = pKF1->getPose();
    Sophus::SE3f T2w = pKF2->getPose();

    // Transformation between cameras
    Sophus::Sim3f S21 = S12.inverse();

    const vector<MapPoint *> mapPoints1 = pKF1->getMapPointMatches();
    const int                N1         = mapPoints1.size();

    const vector<MapPoint *> mapPoints2 = pKF2->getMapPointMatches();
    const int                N2         = mapPoints2.size();

    vector<bool> alreadyMatched1Flags(N1, false);
    vector<bool> alreadyMatched2Flags(N2, false);

    for (int keyPointIndex1 = 0; keyPointIndex1 < N1; keyPointIndex1++)
    {
        MapPoint *p_mapPoint = matches12_inout[keyPointIndex1];
        if (p_mapPoint)
        {
            alreadyMatched1Flags[keyPointIndex1] = true;
            int index2 = get<0>(p_mapPoint->getIndexInKeyFrame(pKF2));
            if (index2 >= 0 && index2 < N2)
                alreadyMatched2Flags[index2] = true;
        }
    }

    vector<int> matchIndices1(N1, -1);
    vector<int> matchIndices2(N2, -1);

    // Transform from KF1 to KF2 and search
    for (int i1 = 0; i1 < N1; i1++)
    {
        MapPoint *p_mapPoint = mapPoints1[i1];

        if (!p_mapPoint || alreadyMatched1Flags[i1])
            continue;

        if (p_mapPoint->isBad())
            continue;

        Eigen::Vector3f p3Dw  = p_mapPoint->getWorldPos();
        Eigen::Vector3f p3Dc1 = T1w * p3Dw;
        Eigen::Vector3f p3Dc2 = S21 * p3Dc1;

        // Depth must be positive
        if (p3Dc2(2) < 0.0)
            continue;

        const float invz = 1.0 / p3Dc2(2);
        const float x    = p3Dc2(0) * invz;
        const float y    = p3Dc2(1) * invz;

        const float u = fx * x + cx;
        const float v = fy * y + cy;

        // Point must be inside the image
        if (!pKF2->isInImage(u, v))
            continue;

        const float maximumDistance = p_mapPoint->getMaxDistanceInvariance();
        const float minimumDistance = p_mapPoint->getMinDistanceInvariance();
        const float distance3d      = p3Dc2.norm();

        // Depth must be inside the scale invariance region
        if (distance3d < minimumDistance || distance3d > maximumDistance)
            continue;

        // Compute predicted octave
        const int predictedLevelCount =
            p_mapPoint->predictScale(distance3d, pKF2);

        // Search in a radius
        const float radius = th * pKF2->scaleFactors[predictedLevelCount];

        const vector<size_t> indices = pKF2->getFeaturesInArea(u, v, radius);

        if (indices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat mapPointDescriptor = p_mapPoint->getDescriptor();

        int bestDistance = INT_MAX;
        int bestIndex    = -1;
        for (vector<size_t>::const_iterator vit  = indices.begin(),
                                            vend = indices.end();
             vit != vend;
             vit++)
        {
            const size_t featureIndex = *vit;

            const cv::KeyPoint &keyPoint =
                pKF2->keyPointsUndistorted[featureIndex];

            if (keyPoint.octave < predictedLevelCount - 1 ||
                keyPoint.octave > predictedLevelCount)
                continue;

            const cv::Mat &keyFrameDescriptor =
                pKF2->descriptors.row(featureIndex);

            const int distance = computeDescriptorDistance(mapPointDescriptor,
                                                           keyFrameDescriptor);

            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex    = featureIndex;
            }
        }

        if (bestDistance <= TH_HIGH)
        {
            matchIndices1[i1] = bestIndex;
        }
    }

    // Transform from KF2 to KF2 and search
    for (int i2 = 0; i2 < N2; i2++)
    {
        MapPoint *p_mapPoint = mapPoints2[i2];

        if (!p_mapPoint || alreadyMatched2Flags[i2])
            continue;

        if (p_mapPoint->isBad())
            continue;

        Eigen::Vector3f p3Dw  = p_mapPoint->getWorldPos();
        Eigen::Vector3f p3Dc2 = T2w * p3Dw;
        Eigen::Vector3f p3Dc1 = S12 * p3Dc2;

        // Depth must be positive
        if (p3Dc1(2) < 0.0)
            continue;

        const float invz = 1.0 / p3Dc1(2);
        const float x    = p3Dc1(0) * invz;
        const float y    = p3Dc1(1) * invz;

        const float u = fx * x + cx;
        const float v = fy * y + cy;

        // Point must be inside the image
        if (!pKF1->isInImage(u, v))
            continue;

        const float maximumDistance = p_mapPoint->getMaxDistanceInvariance();
        const float minimumDistance = p_mapPoint->getMinDistanceInvariance();
        const float distance3d      = p3Dc1.norm();

        // Depth must be inside the scale pyramid of the image
        if (distance3d < minimumDistance || distance3d > maximumDistance)
            continue;

        // Compute predicted octave
        const int predictedLevelCount =
            p_mapPoint->predictScale(distance3d, pKF1);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th * pKF1->scaleFactors[predictedLevelCount];

        const vector<size_t> indices = pKF1->getFeaturesInArea(u, v, radius);

        if (indices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat mapPointDescriptor = p_mapPoint->getDescriptor();

        int bestDistance = INT_MAX;
        int bestIndex    = -1;
        for (vector<size_t>::const_iterator vit  = indices.begin(),
                                            vend = indices.end();
             vit != vend;
             vit++)
        {
            const size_t featureIndex = *vit;

            const cv::KeyPoint &keyPoint =
                pKF1->keyPointsUndistorted[featureIndex];

            if (keyPoint.octave < predictedLevelCount - 1 ||
                keyPoint.octave > predictedLevelCount)
                continue;

            const cv::Mat &keyFrameDescriptor =
                pKF1->descriptors.row(featureIndex);

            const int distance = computeDescriptorDistance(mapPointDescriptor,
                                                           keyFrameDescriptor);

            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex    = featureIndex;
            }
        }

        if (bestDistance <= TH_HIGH)
        {
            matchIndices2[i2] = bestIndex;
        }
    }

    // Check agreement
    int foundCount = 0;

    for (int i1 = 0; i1 < N1; i1++)
    {
        int index2 = matchIndices1[i1];

        if (index2 >= 0)
        {
            int index1 = matchIndices2[index2];
            if (index1 == i1)
            {
                matches12_inout[i1] = mapPoints2[index2];
                foundCount++;
            }
        }
    }

    return foundCount;
}

} // namespace core
} // namespace vs_graphs
