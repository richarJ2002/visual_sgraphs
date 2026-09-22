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

int ORBmatcher::searchByProjection(Frame                 &CurrentFrame,
                                   KeyFrame              *pKF,
                                   const set<MapPoint *> &sAlreadyFound,
                                   const float            th,
                                   const int              ORBdist)
{
    int nmatches = 0;

    const Sophus::SE3f Tcw = CurrentFrame.getPose();
    Eigen::Vector3f    Ow  = Tcw.inverse().translation();

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    const vector<MapPoint *> vpMPs = pKF->getMapPointMatches();

    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
    {
        MapPoint *pMP = vpMPs[i];

        if (pMP)
        {
            if (!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                // Project
                Eigen::Vector3f x3Dw = pMP->getWorldPos();
                Eigen::Vector3f x3Dc = Tcw * x3Dw;

                const Eigen::Vector2f uv = CurrentFrame.p_camera->project(x3Dc);

                if (uv(0) < CurrentFrame.gridMinX ||
                    uv(0) > CurrentFrame.gridMaxX)
                    continue;
                if (uv(1) < CurrentFrame.gridMinY ||
                    uv(1) > CurrentFrame.gridMaxY)
                    continue;

                // Compute predicted scale level
                Eigen::Vector3f PO     = x3Dw - Ow;
                float           dist3D = PO.norm();

                const float maxDistance = pMP->getMaxDistanceInvariance();
                const float minDistance = pMP->getMinDistanceInvariance();

                // Depth must be inside the scale pyramid of the image
                if (dist3D < minDistance || dist3D > maxDistance)
                    continue;

                int nPredictedLevel = pMP->predictScale(dist3D, &CurrentFrame);

                // Search in a window
                const float radius =
                    th * CurrentFrame.scaleFactors[nPredictedLevel];

                const vector<size_t> vIndices2 =
                    CurrentFrame.getFeaturesInArea(uv(0),
                                                   uv(1),
                                                   radius,
                                                   nPredictedLevel - 1,
                                                   nPredictedLevel + 1);

                if (vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->getDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for (vector<size_t>::const_iterator vit = vIndices2.begin();
                     vit != vIndices2.end();
                     vit++)
                {
                    const size_t i2 = *vit;
                    if (CurrentFrame.mapPoints[i2])
                        continue;

                    const cv::Mat &d = CurrentFrame.descriptors.row(i2);

                    const int dist = computeDescriptorDistance(dMP, d);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdx2 = i2;
                    }
                }

                if (bestDist <= ORBdist)
                {
                    CurrentFrame.mapPoints[bestIdx2] = pMP;
                    nmatches++;

                    if (mbCheckOrientation)
                    {
                        float rot =
                            pKF->keyPointsUndistorted[i].angle -
                            CurrentFrame.keyPointsUndistorted[bestIdx2].angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }

    if (mbCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        computeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for (int i = 0; i < HISTO_LENGTH; i++)
        {
            if (i != ind1 && i != ind2 && i != ind3)
            {
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    CurrentFrame.mapPoints[rotHist[i][j]] = nullptr;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
