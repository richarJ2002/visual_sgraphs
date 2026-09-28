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
                                   vector<MapPoint *>       &matched_inout,
                                   int                       th,
                                   float                     ratioHamming)
{
    Sophus::SE3f Tcw =
        Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    // Set of MapPoints already found in the KeyFrame
    set<MapPoint *> alreadyFounds(matched_inout.begin(), matched_inout.end());
    alreadyFounds.erase(static_cast<MapPoint *>(nullptr));

    int nmatches = 0;

    // For each Candidate MapPoint Project and Match
    for (int mapPointIndex = 0, iendMapPoint = vpPoints.size();
         mapPointIndex < iendMapPoint;
         mapPointIndex++)
    {
        MapPoint *p_mapPoint = vpPoints[mapPointIndex];

        // Discard Bad MapPoints and already found
        if (p_mapPoint->isBad() || alreadyFounds.count(p_mapPoint))
            continue;

        // Get 3D Coords.
        Eigen::Vector3f p3Dw = p_mapPoint->getWorldPos();

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
        const float maximumDistance = p_mapPoint->getMaxDistanceInvariance();
        const float minimumDistance = p_mapPoint->getMinDistanceInvariance();
        Eigen::Vector3f PO          = p3Dw - Ow;
        const float     distance    = PO.norm();

        if (distance < minimumDistance || distance > maximumDistance)
            continue;

        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn = p_mapPoint->getNormal();

        if (PO.dot(Pn) < 0.5 * distance)
            continue;

        int predictedLevelCount = p_mapPoint->predictScale(distance, pKF);

        // Search in a radius
        const float radius = th * pKF->scaleFactors[predictedLevelCount];

        const vector<size_t> indices =
            pKF->getFeaturesInArea(uv(0), uv(1), radius);

        if (indices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat mapPointDescriptor = p_mapPoint->getDescriptor();

        int bestDistance = 256;
        int bestIndex    = -1;
        for (vector<size_t>::const_iterator vit  = indices.begin(),
                                            vend = indices.end();
             vit != vend;
             vit++)
        {
            const size_t featureIndex = *vit;
            if (matched_inout[featureIndex])
                continue;

            const int &keyPointLevel =
                pKF->keyPointsUndistorted[featureIndex].octave;

            if (keyPointLevel < predictedLevelCount - 1 ||
                keyPointLevel > predictedLevelCount)
                continue;

            const cv::Mat &keyFrameDescriptor =
                pKF->descriptors.row(featureIndex);

            const int distance = computeDescriptorDistance(mapPointDescriptor,
                                                           keyFrameDescriptor);

            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex    = featureIndex;
            }
        }

        if (bestDistance <= TH_LOW * ratioHamming)
        {
            matched_inout[bestIndex] = p_mapPoint;
            nmatches++;
        }
    }

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
