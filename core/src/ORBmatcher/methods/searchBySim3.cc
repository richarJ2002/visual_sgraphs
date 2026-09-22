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
                             std::vector<MapPoint *> &vpMatches12,
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

    const vector<MapPoint *> vpMapPoints1 = pKF1->getMapPointMatches();
    const int                N1           = vpMapPoints1.size();

    const vector<MapPoint *> vpMapPoints2 = pKF2->getMapPointMatches();
    const int                N2           = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1, false);
    vector<bool> vbAlreadyMatched2(N2, false);

    for (int i = 0; i < N1; i++)
    {
        MapPoint *pMP = vpMatches12[i];
        if (pMP)
        {
            vbAlreadyMatched1[i] = true;
            int idx2             = get<0>(pMP->getIndexInKeyFrame(pKF2));
            if (idx2 >= 0 && idx2 < N2)
                vbAlreadyMatched2[idx2] = true;
        }
    }

    vector<int> vnMatch1(N1, -1);
    vector<int> vnMatch2(N2, -1);

    // Transform from KF1 to KF2 and search
    for (int i1 = 0; i1 < N1; i1++)
    {
        MapPoint *pMP = vpMapPoints1[i1];

        if (!pMP || vbAlreadyMatched1[i1])
            continue;

        if (pMP->isBad())
            continue;

        Eigen::Vector3f p3Dw  = pMP->getWorldPos();
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

        const float maxDistance = pMP->getMaxDistanceInvariance();
        const float minDistance = pMP->getMinDistanceInvariance();
        const float dist3D      = p3Dc2.norm();

        // Depth must be inside the scale invariance region
        if (dist3D < minDistance || dist3D > maxDistance)
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->predictScale(dist3D, pKF2);

        // Search in a radius
        const float radius = th * pKF2->scaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF2->getFeaturesInArea(u, v, radius);

        if (vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->getDescriptor();

        int bestDist = INT_MAX;
        int bestIdx  = -1;
        for (vector<size_t>::const_iterator vit  = vIndices.begin(),
                                            vend = vIndices.end();
             vit != vend;
             vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF2->keyPointsUndistorted[idx];

            if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF2->descriptors.row(idx);

            const int dist = computeDescriptorDistance(dMP, dKF);

            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx  = idx;
            }
        }

        if (bestDist <= TH_HIGH)
        {
            vnMatch1[i1] = bestIdx;
        }
    }

    // Transform from KF2 to KF2 and search
    for (int i2 = 0; i2 < N2; i2++)
    {
        MapPoint *pMP = vpMapPoints2[i2];

        if (!pMP || vbAlreadyMatched2[i2])
            continue;

        if (pMP->isBad())
            continue;

        Eigen::Vector3f p3Dw  = pMP->getWorldPos();
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

        const float maxDistance = pMP->getMaxDistanceInvariance();
        const float minDistance = pMP->getMinDistanceInvariance();
        const float dist3D      = p3Dc1.norm();

        // Depth must be inside the scale pyramid of the image
        if (dist3D < minDistance || dist3D > maxDistance)
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->predictScale(dist3D, pKF1);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th * pKF1->scaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF1->getFeaturesInArea(u, v, radius);

        if (vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->getDescriptor();

        int bestDist = INT_MAX;
        int bestIdx  = -1;
        for (vector<size_t>::const_iterator vit  = vIndices.begin(),
                                            vend = vIndices.end();
             vit != vend;
             vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF1->keyPointsUndistorted[idx];

            if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF1->descriptors.row(idx);

            const int dist = computeDescriptorDistance(dMP, dKF);

            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx  = idx;
            }
        }

        if (bestDist <= TH_HIGH)
        {
            vnMatch2[i2] = bestIdx;
        }
    }

    // Check agreement
    int nFound = 0;

    for (int i1 = 0; i1 < N1; i1++)
    {
        int idx2 = vnMatch1[i1];

        if (idx2 >= 0)
        {
            int idx1 = vnMatch2[idx2];
            if (idx1 == i1)
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}

} // namespace core
} // namespace vs_graphs
