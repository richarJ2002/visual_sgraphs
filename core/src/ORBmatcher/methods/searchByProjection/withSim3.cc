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

int ORBmatcher::searchByProjection(KeyFrame                 *pKF,
                                   Sophus::Sim3f            &Scw,
                                   const vector<MapPoint *> &vpPoints,
                                   vector<MapPoint *>       &vpMatched,
                                   int                       th,
                                   float                     ratioHamming)
{
    Sophus::SE3f Tcw =
        Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    // Set of MapPoints already found in the KeyFrame
    set<MapPoint *> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint *>(nullptr));

    int nmatches = 0;

    // For each Candidate MapPoint Project and Match
    for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++)
    {
        MapPoint *pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if (pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        Eigen::Vector3f p3Dw = pMP->getWorldPos();

        // Transform into Camera Coords.
        Eigen::Vector3f p3Dc = Tcw * p3Dw;

        // Depth must be positive
        if (p3Dc(2) < 0.0)
            continue;

        // Project into Image
        const Eigen::Vector2f uv = pKF->p_camera->project(p3Dc);

        // Point must be inside the image
        if (!pKF->isInImage(uv(0), uv(1)))
            continue;

        // Depth must be inside the scale invariance region of the point
        const float     maxDistance = pMP->getMaxDistanceInvariance();
        const float     minDistance = pMP->getMinDistanceInvariance();
        Eigen::Vector3f PO          = p3Dw - Ow;
        const float     dist        = PO.norm();

        if (dist < minDistance || dist > maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn = pMP->getNormal();

        if (PO.dot(Pn) < 0.5 * dist)
            continue;

        int nPredictedLevel = pMP->predictScale(dist, pKF);

        // Search in a radius
        const float radius = th * pKF->scaleFactors[nPredictedLevel];

        const vector<size_t> vIndices =
            pKF->getFeaturesInArea(uv(0), uv(1), radius);

        if (vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->getDescriptor();

        int bestDist = 256;
        int bestIdx  = -1;
        for (vector<size_t>::const_iterator vit  = vIndices.begin(),
                                            vend = vIndices.end();
             vit != vend;
             vit++)
        {
            const size_t idx = *vit;
            if (vpMatched[idx])
                continue;

            const int &kpLevel = pKF->keyPointsUndistorted[idx].octave;

            if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->descriptors.row(idx);

            const int dist = computeDescriptorDistance(dMP, dKF);

            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx  = idx;
            }
        }

        if (bestDist <= TH_LOW * ratioHamming)
        {
            vpMatched[bestIdx] = pMP;
            nmatches++;
        }
    }

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
