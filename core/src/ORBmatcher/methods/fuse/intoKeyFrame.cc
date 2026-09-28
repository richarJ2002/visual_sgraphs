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
                     const vector<MapPoint *> &vpMapPoints,
                     const float               th,
                     const bool                bRight)
{
    camera_models::geometriccamera::GeometricCamera *p_camera;
    Sophus::SE3f                                     Tcw;
    Eigen::Vector3f                                  Ow;

    if (bRight)
    {
        Tcw      = p_keyframe_inout->getRightPose();
        Ow       = p_keyframe_inout->getRightCameraCenter();
        p_camera = p_keyframe_inout->p_camera2;
    }
    else
    {
        Tcw      = p_keyframe_inout->getPose();
        Ow       = p_keyframe_inout->getCameraCenter();
        p_camera = p_keyframe_inout->p_camera;
    }

    int          fusedCount    = 0;
    const float &bf            = p_keyframe_inout->mbf;
    const int    mapPointCount = vpMapPoints.size();

    // For debbuging
    int notMapPointCount = 0, badCount = 0, isinKeyFrameCount = 0,
        negdepthCount = 0, notinimCount = 0, distanceCount = 0, normalCount = 0,
        notidxCount = 0, thcheckCount = 0;
    for (int mapPointIndex = 0; mapPointIndex < mapPointCount; mapPointIndex++)
    {
        MapPoint *p_mapPoint = vpMapPoints[mapPointIndex];

        if (!p_mapPoint)
        {
            notMapPointCount++;
            continue;
        }

        if (p_mapPoint->isBad())
        {
            badCount++;
            continue;
        }
        else if (p_mapPoint->isInKeyFrame(p_keyframe_inout))
        {
            isinKeyFrameCount++;
            continue;
        }

        Eigen::Vector3f p3Dw = p_mapPoint->getWorldPos();
        Eigen::Vector3f p3Dc = Tcw * p3Dw;

        // Depth must be positive
        if (p3Dc(2) < 0.0f)
        {
            negdepthCount++;
            continue;
        }

        const float invz = 1 / p3Dc(2);

        const Eigen::Vector2f uv = p_camera->project(p3Dc);

        // Point must be inside the image
        if (!p_keyframe_inout->isInImage(uv(0), uv(1)))
        {
            notinimCount++;
            continue;
        }

        const float ur = uv(0) - bf * invz;

        const float maximumDistance = p_mapPoint->getMaxDistanceInvariance();
        const float minimumDistance = p_mapPoint->getMinDistanceInvariance();
        Eigen::Vector3f PO          = p3Dw - Ow;
        const float     distance3d  = PO.norm();

        // Depth must be inside the scale pyramid of the image
        if (distance3d < minimumDistance || distance3d > maximumDistance)
        {
            distanceCount++;
            continue;
        }

        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn = p_mapPoint->getNormal();

        if (PO.dot(Pn) < 0.5 * distance3d)
        {
            normalCount++;
            continue;
        }

        int predictedLevelCount =
            p_mapPoint->predictScale(distance3d, p_keyframe_inout);

        // Search in a radius
        const float radius =
            th * p_keyframe_inout->scaleFactors[predictedLevelCount];

        const vector<size_t> indices =
            p_keyframe_inout->getFeaturesInArea(uv(0), uv(1), radius, bRight);

        if (indices.empty())
        {
            notidxCount++;
            continue;
        }

        // Match to the most similar keypoint in the radius

        const cv::Mat mapPointDescriptor = p_mapPoint->getDescriptor();

        int bestDistance = 256;
        int bestIndex    = -1;
        for (vector<size_t>::const_iterator vit  = indices.begin(),
                                            vend = indices.end();
             vit != vend;
             vit++)
        {
            size_t              featureIndex = *vit;
            const cv::KeyPoint &keyPoint =
                (p_keyframe_inout->leftKeyPointCount == -1)
                    ? p_keyframe_inout->keyPointsUndistorted[featureIndex]
                : (!bRight) ? p_keyframe_inout->keyPoints[featureIndex]
                            : p_keyframe_inout->keyPointsRight[featureIndex];

            const int &keyPointLevel = keyPoint.octave;

            if (keyPointLevel < predictedLevelCount - 1 ||
                keyPointLevel > predictedLevelCount)
                continue;

            if (p_keyframe_inout->uRight[featureIndex] >= 0)
            {
                // Check reprojection error in stereo
                const float &kpx = keyPoint.pt.x;
                const float &kpy = keyPoint.pt.y;
                const float &kpr = p_keyframe_inout->uRight[featureIndex];
                const float  ex  = uv(0) - kpx;
                const float  ey  = uv(1) - kpy;
                const float  er  = ur - kpr;
                const float  e2  = ex * ex + ey * ey + er * er;

                if (e2 * p_keyframe_inout->invLevelSigmaSquared[keyPointLevel] >
                    7.8)
                    continue;
            }
            else
            {
                const float &kpx = keyPoint.pt.x;
                const float &kpy = keyPoint.pt.y;
                const float  ex  = uv(0) - kpx;
                const float  ey  = uv(1) - kpy;
                const float  e2  = ex * ex + ey * ey;

                if (e2 * p_keyframe_inout->invLevelSigmaSquared[keyPointLevel] >
                    5.99)
                    continue;
            }

            if (bRight)
                featureIndex += p_keyframe_inout->leftKeyPointCount;

            const cv::Mat &keyFrameDescriptor =
                p_keyframe_inout->descriptors.row(featureIndex);

            const int distance = computeDescriptorDistance(mapPointDescriptor,
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
                {
                    if (p_keyFrameMapPoint->getObservationCount() >
                        p_mapPoint->getObservationCount())
                        p_mapPoint->replace(p_keyFrameMapPoint);
                    else
                        p_keyFrameMapPoint->replace(p_mapPoint);
                }
            }
            else
            {
                p_mapPoint->addObservation(p_keyframe_inout, bestIndex);
                p_keyframe_inout->addMapPoint(p_mapPoint, bestIndex);
            }
            fusedCount++;
        }
        else
            thcheckCount++;
    }

    return fusedCount;
}

} // namespace core
} // namespace vs_graphs
