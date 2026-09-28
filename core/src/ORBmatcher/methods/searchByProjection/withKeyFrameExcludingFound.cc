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
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    const vector<MapPoint *> mapPoints = pKF->getMapPointMatches();

    for (size_t histogramBinIndex = 0, iend = mapPoints.size();
         histogramBinIndex < iend;
         histogramBinIndex++)
    {
        MapPoint *p_mapPoint = mapPoints[histogramBinIndex];

        if (p_mapPoint)
        {
            if (!p_mapPoint->isBad() && !sAlreadyFound.count(p_mapPoint))
            {
                // Project
                Eigen::Vector3f x3Dw = p_mapPoint->getWorldPos();
                Eigen::Vector3f x3Dc = Tcw * x3Dw;

                const Eigen::Vector2f uv = CurrentFrame.p_camera->project(x3Dc);

                if (uv(0) < CurrentFrame.gridMinX ||
                    uv(0) > CurrentFrame.gridMaxX)
                    continue;
                if (uv(1) < CurrentFrame.gridMinY ||
                    uv(1) > CurrentFrame.gridMaxY)
                    continue;

                // Compute predicted scale level
                Eigen::Vector3f PO         = x3Dw - Ow;
                float           distance3d = PO.norm();

                const float maximumDistance =
                    p_mapPoint->getMaxDistanceInvariance();
                const float minimumDistance =
                    p_mapPoint->getMinDistanceInvariance();

                // Depth must be inside the scale pyramid of the image
                if (distance3d < minimumDistance ||
                    distance3d > maximumDistance)
                    continue;

                int predictedLevelCount =
                    p_mapPoint->predictScale(distance3d, &CurrentFrame);

                // Search in a window
                const float radius =
                    th * CurrentFrame.scaleFactors[predictedLevelCount];

                const vector<size_t> indices2 =
                    CurrentFrame.getFeaturesInArea(uv(0),
                                                   uv(1),
                                                   radius,
                                                   predictedLevelCount - 1,
                                                   predictedLevelCount + 1);

                if (indices2.empty())
                    continue;

                const cv::Mat mapPointDescriptor = p_mapPoint->getDescriptor();

                int bestDistance = 256;
                int bestIndex2   = -1;

                for (vector<size_t>::const_iterator vit = indices2.begin();
                     vit != indices2.end();
                     vit++)
                {
                    const size_t i2 = *vit;
                    if (CurrentFrame.mapPoints[i2])
                        continue;

                    const cv::Mat &d = CurrentFrame.descriptors.row(i2);

                    const int distance =
                        computeDescriptorDistance(mapPointDescriptor, d);

                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        bestIndex2   = i2;
                    }
                }

                if (bestDistance <= ORBdist)
                {
                    CurrentFrame.mapPoints[bestIndex2] = p_mapPoint;
                    nmatches++;

                    if (shouldCheckOrientation)
                    {
                        float rot =
                            pKF->keyPointsUndistorted[histogramBinIndex].angle -
                            CurrentFrame.keyPointsUndistorted[bestIndex2].angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIndex2);
                    }
                }
            }
        }
    }

    if (shouldCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        computeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
             histogramBinIndex++)
        {
            if (histogramBinIndex != ind1 && histogramBinIndex != ind2 &&
                histogramBinIndex != ind3)
            {
                for (size_t binEntryIndex = 0,
                            jend          = rotHist[histogramBinIndex].size();
                     binEntryIndex < jend;
                     binEntryIndex++)
                {
                    CurrentFrame
                        .mapPoints[rotHist[histogramBinIndex][binEntryIndex]] =
                        nullptr;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
