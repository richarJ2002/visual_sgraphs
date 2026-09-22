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

int ORBmatcher::fuse(KeyFrame                 *pKF,
                     const vector<MapPoint *> &vpMapPoints,
                     const float               th,
                     const bool                bRight)
{
    camera_models::geometriccamera::GeometricCamera *pCamera;
    Sophus::SE3f                                     Tcw;
    Eigen::Vector3f                                  Ow;

    if (bRight)
    {
        Tcw     = pKF->getRightPose();
        Ow      = pKF->getRightCameraCenter();
        pCamera = pKF->p_camera2;
    }
    else
    {
        Tcw     = pKF->getPose();
        Ow      = pKF->getCameraCenter();
        pCamera = pKF->p_camera;
    }

    int          nFused = 0;
    const float &bf     = pKF->mbf;
    const int    nMPs   = vpMapPoints.size();

    // For debbuging
    int count_notMP = 0, count_bad = 0, count_isinKF = 0, count_negdepth = 0,
        count_notinim = 0, count_dist = 0, count_normal = 0, count_notidx = 0,
        count_thcheck = 0;
    for (int i = 0; i < nMPs; i++)
    {
        MapPoint *pMP = vpMapPoints[i];

        if (!pMP)
        {
            count_notMP++;
            continue;
        }

        if (pMP->isBad())
        {
            count_bad++;
            continue;
        }
        else if (pMP->isInKeyFrame(pKF))
        {
            count_isinKF++;
            continue;
        }

        Eigen::Vector3f p3Dw = pMP->getWorldPos();
        Eigen::Vector3f p3Dc = Tcw * p3Dw;

        // Depth must be positive
        if (p3Dc(2) < 0.0f)
        {
            count_negdepth++;
            continue;
        }

        const float invz = 1 / p3Dc(2);

        const Eigen::Vector2f uv = pCamera->project(p3Dc);

        // Point must be inside the image
        if (!pKF->isInImage(uv(0), uv(1)))
        {
            count_notinim++;
            continue;
        }

        const float ur = uv(0) - bf * invz;

        const float     maxDistance = pMP->getMaxDistanceInvariance();
        const float     minDistance = pMP->getMinDistanceInvariance();
        Eigen::Vector3f PO          = p3Dw - Ow;
        const float     dist3D      = PO.norm();

        // Depth must be inside the scale pyramid of the image
        if (dist3D < minDistance || dist3D > maxDistance)
        {
            count_dist++;
            continue;
        }

        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn = pMP->getNormal();

        if (PO.dot(Pn) < 0.5 * dist3D)
        {
            count_normal++;
            continue;
        }

        int nPredictedLevel = pMP->predictScale(dist3D, pKF);

        // Search in a radius
        const float radius = th * pKF->scaleFactors[nPredictedLevel];

        const vector<size_t> vIndices =
            pKF->getFeaturesInArea(uv(0), uv(1), radius, bRight);

        if (vIndices.empty())
        {
            count_notidx++;
            continue;
        }

        // Match to the most similar keypoint in the radius

        const cv::Mat dMP = pMP->getDescriptor();

        int bestDist = 256;
        int bestIdx  = -1;
        for (vector<size_t>::const_iterator vit  = vIndices.begin(),
                                            vend = vIndices.end();
             vit != vend;
             vit++)
        {
            size_t              idx = *vit;
            const cv::KeyPoint &kp  = (pKF->Nleft == -1)
                                          ? pKF->keyPointsUndistorted[idx]
                                      : (!bRight) ? pKF->keyPoints[idx]
                                                  : pKF->keyPointsRight[idx];

            const int &kpLevel = kp.octave;

            if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
                continue;

            if (pKF->uRight[idx] >= 0)
            {
                // Check reprojection error in stereo
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float &kpr = pKF->uRight[idx];
                const float  ex  = uv(0) - kpx;
                const float  ey  = uv(1) - kpy;
                const float  er  = ur - kpr;
                const float  e2  = ex * ex + ey * ey + er * er;

                if (e2 * pKF->invLevelSigmaSquared[kpLevel] > 7.8)
                    continue;
            }
            else
            {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float  ex  = uv(0) - kpx;
                const float  ey  = uv(1) - kpy;
                const float  e2  = ex * ex + ey * ey;

                if (e2 * pKF->invLevelSigmaSquared[kpLevel] > 5.99)
                    continue;
            }

            if (bRight)
                idx += pKF->Nleft;

            const cv::Mat &dKF = pKF->descriptors.row(idx);

            const int dist = computeDescriptorDistance(dMP, dKF);

            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx  = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if (bestDist <= TH_LOW)
        {
            MapPoint *pMPinKF = pKF->getMapPoint(bestIdx);
            if (pMPinKF)
            {
                if (!pMPinKF->isBad())
                {
                    if (pMPinKF->getObservationCount() >
                        pMP->getObservationCount())
                        pMP->replace(pMPinKF);
                    else
                        pMPinKF->replace(pMP);
                }
            }
            else
            {
                pMP->addObservation(pKF, bestIdx);
                pKF->addMapPoint(pMP, bestIdx);
            }
            nFused++;
        }
        else
            count_thcheck++;
    }

    return nFused;
}

} // namespace core
} // namespace vs_graphs
