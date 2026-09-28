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

int ORBmatcher::fuse(KeyFrame                 *p_keyframe_inout,
                     Sophus::Sim3f            &Scw,
                     const vector<MapPoint *> &vpPoints,
                     float                     th,
                     vector<MapPoint *>       &replacePoints_inout)
{
    // Decompose Scw
    Sophus::SE3f Tcw =
        Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    // Set of MapPoints already found in the KeyFrame
    const set<MapPoint *> alreadyFounds = p_keyframe_inout->getMapPoints();

    int fusedCount = 0;

    const int pointCount = vpPoints.size();

    // For each candidate MapPoint project and match
    for (int mapPointIndex = 0; mapPointIndex < pointCount; mapPointIndex++)
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
        if (p3Dc(2) < 0.0f)
            continue;

        // Project into Image
        const Eigen::Vector2f uv = p_keyframe_inout->p_camera->project(p3Dc);

        // Point must be inside the image
        if (!p_keyframe_inout->isInImage(uv(0), uv(1)))
            continue;

        // Depth must be inside the scale pyramid of the image
        const float maximumDistance = p_mapPoint->getMaxDistanceInvariance();
        const float minimumDistance = p_mapPoint->getMinDistanceInvariance();
        Eigen::Vector3f PO          = p3Dw - Ow;
        const float     distance3d  = PO.norm();

        if (distance3d < minimumDistance || distance3d > maximumDistance)
            continue;

        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn = p_mapPoint->getNormal();

        if (PO.dot(Pn) < 0.5 * distance3d)
            continue;

        // Compute predicted scale level
        const int predictedLevelCount =
            p_mapPoint->predictScale(distance3d, p_keyframe_inout);

        // Search in a radius
        const float radius =
            th * p_keyframe_inout->scaleFactors[predictedLevelCount];

        const vector<size_t> indices =
            p_keyframe_inout->getFeaturesInArea(uv(0), uv(1), radius);

        if (indices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        const cv::Mat mapPointDescriptor = p_mapPoint->getDescriptor();

        int bestDistance = INT_MAX;
        int bestIndex    = -1;
        for (vector<size_t>::const_iterator vit = indices.begin();
             vit != indices.end();
             vit++)
        {
            const size_t featureIndex = *vit;
            const int   &keyPointLevel =
                p_keyframe_inout->keyPointsUndistorted[featureIndex].octave;

            if (keyPointLevel < predictedLevelCount - 1 ||
                keyPointLevel > predictedLevelCount)
                continue;

            const cv::Mat &keyFrameDescriptor =
                p_keyframe_inout->descriptors.row(featureIndex);

            int distance = computeDescriptorDistance(mapPointDescriptor,
                                                     keyFrameDescriptor);

            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex    = featureIndex;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if (bestDistance <= TH_LOW)
        {
            MapPoint *p_keyFrameMapPoint =
                p_keyframe_inout->getMapPoint(bestIndex);
            if (p_keyFrameMapPoint)
            {
                if (!p_keyFrameMapPoint->isBad())
                    replacePoints_inout[mapPointIndex] = p_keyFrameMapPoint;
            }
            else
            {
                p_mapPoint->addObservation(p_keyframe_inout, bestIndex);
                p_keyframe_inout->addMapPoint(p_mapPoint, bestIndex);
            }
            fusedCount++;
        }
    }

    return fusedCount;
}

} // namespace core
} // namespace vs_graphs
