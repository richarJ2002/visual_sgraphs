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

int ORBmatcher::searchByProjection(Frame       &CurrentFrame,
                                   const Frame &LastFrame,
                                   const float  th,
                                   const bool   bMono)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    const Sophus::SE3f    Tcw = CurrentFrame.getPose();
    const Eigen::Vector3f twc = Tcw.inverse().translation();

    const Sophus::SE3f    Tlw = LastFrame.getPose();
    const Eigen::Vector3f tlc = Tlw * twc;

    const bool isMovingForward  = tlc(2) > CurrentFrame.mb && !bMono;
    const bool isMovingBackward = -tlc(2) > CurrentFrame.mb && !bMono;

    for (int histogramBinIndex = 0; histogramBinIndex < LastFrame.keyPointCount;
         histogramBinIndex++)
    {
        MapPoint *p_mapPoint = LastFrame.mapPoints[histogramBinIndex];
        if (p_mapPoint)
        {
            if (!LastFrame.outlierFlags[histogramBinIndex])
            {
                // Project
                Eigen::Vector3f x3Dw = p_mapPoint->getWorldPos();
                Eigen::Vector3f x3Dc = Tcw * x3Dw;

                const float invzc = 1.0 / x3Dc(2);

                if (invzc < 0)
                    continue;

                Eigen::Vector2f uv = CurrentFrame.p_camera->project(x3Dc);

                if (uv(0) < CurrentFrame.gridMinX ||
                    uv(0) > CurrentFrame.gridMaxX)
                    continue;
                if (uv(1) < CurrentFrame.gridMinY ||
                    uv(1) > CurrentFrame.gridMaxY)
                    continue;

                int lastOctaveCount =
                    (LastFrame.leftKeyPointCount == -1 ||
                     histogramBinIndex < LastFrame.leftKeyPointCount)
                        ? LastFrame.keyPoints[histogramBinIndex].octave
                        : LastFrame
                              .keyPointsRight[histogramBinIndex -
                                              LastFrame.leftKeyPointCount]
                              .octave;

                // Search in a window. Size depends on scale
                float radius = th * CurrentFrame.scaleFactors[lastOctaveCount];

                vector<size_t> indices2;

                if (isMovingForward)
                    indices2 = CurrentFrame.getFeaturesInArea(uv(0),
                                                              uv(1),
                                                              radius,
                                                              lastOctaveCount);
                else if (isMovingBackward)
                    indices2 = CurrentFrame.getFeaturesInArea(uv(0),
                                                              uv(1),
                                                              radius,
                                                              0,
                                                              lastOctaveCount);
                else
                    indices2 =
                        CurrentFrame.getFeaturesInArea(uv(0),
                                                       uv(1),
                                                       radius,
                                                       lastOctaveCount - 1,
                                                       lastOctaveCount + 1);

                if (indices2.empty())
                    continue;

                const cv::Mat mapPointDescriptor = p_mapPoint->getDescriptor();

                int bestDistance = 256;
                int bestIndex2   = -1;

                for (vector<size_t>::const_iterator vit  = indices2.begin(),
                                                    vend = indices2.end();
                     vit != vend;
                     vit++)
                {
                    const size_t i2 = *vit;

                    if (CurrentFrame.mapPoints[i2])
                        if (CurrentFrame.mapPoints[i2]->getObservationCount() >
                            0)
                            continue;

                    if (CurrentFrame.leftKeyPointCount == -1 &&
                        CurrentFrame.uRight[i2] > 0)
                    {
                        const float ur = uv(0) - CurrentFrame.mbf * invzc;
                        const float er = fabs(ur - CurrentFrame.uRight[i2]);
                        if (er > radius)
                            continue;
                    }

                    const cv::Mat &d = CurrentFrame.descriptors.row(i2);

                    const int distance =
                        computeDescriptorDistance(mapPointDescriptor, d);

                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        bestIndex2   = i2;
                    }
                }

                if (bestDistance <= TH_HIGH)
                {
                    CurrentFrame.mapPoints[bestIndex2] = p_mapPoint;
                    nmatches++;

                    if (shouldCheckOrientation)
                    {
                        cv::KeyPoint keyPointLf =
                            (LastFrame.leftKeyPointCount == -1)
                                ? LastFrame
                                      .keyPointsUndistorted[histogramBinIndex]
                            : (histogramBinIndex < LastFrame.leftKeyPointCount)
                                ? LastFrame.keyPoints[histogramBinIndex]
                                : LastFrame.keyPointsRight
                                      [histogramBinIndex -
                                       LastFrame.leftKeyPointCount];

                        cv::KeyPoint keyPointCf =
                            (CurrentFrame.leftKeyPointCount == -1)
                                ? CurrentFrame.keyPointsUndistorted[bestIndex2]
                            : (bestIndex2 < CurrentFrame.leftKeyPointCount)
                                ? CurrentFrame.keyPoints[bestIndex2]
                                : CurrentFrame.keyPointsRight
                                      [bestIndex2 -
                                       CurrentFrame.leftKeyPointCount];
                        float rot = keyPointLf.angle - keyPointCf.angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIndex2);
                    }
                }
                if (CurrentFrame.leftKeyPointCount != -1)
                {
                    Eigen::Vector3f x3Dr =
                        CurrentFrame.getRelativePoseTrl() * x3Dc;
                    Eigen::Vector2f uv = CurrentFrame.p_camera->project(x3Dr);

                    int lastOctaveCount =
                        (LastFrame.leftKeyPointCount == -1 ||
                         histogramBinIndex < LastFrame.leftKeyPointCount)
                            ? LastFrame.keyPoints[histogramBinIndex].octave
                            : LastFrame
                                  .keyPointsRight[histogramBinIndex -
                                                  LastFrame.leftKeyPointCount]
                                  .octave;

                    // Search in a window. Size depends on scale
                    float radius =
                        th * CurrentFrame.scaleFactors[lastOctaveCount];

                    vector<size_t> indices2;

                    if (isMovingForward)
                        indices2 =
                            CurrentFrame.getFeaturesInArea(uv(0),
                                                           uv(1),
                                                           radius,
                                                           lastOctaveCount,
                                                           -1,
                                                           true);
                    else if (isMovingBackward)
                        indices2 =
                            CurrentFrame.getFeaturesInArea(uv(0),
                                                           uv(1),
                                                           radius,
                                                           0,
                                                           lastOctaveCount,
                                                           true);
                    else
                        indices2 =
                            CurrentFrame.getFeaturesInArea(uv(0),
                                                           uv(1),
                                                           radius,
                                                           lastOctaveCount - 1,
                                                           lastOctaveCount + 1,
                                                           true);

                    const cv::Mat mapPointDescriptor =
                        p_mapPoint->getDescriptor();

                    int bestDistance = 256;
                    int bestIndex2   = -1;

                    for (vector<size_t>::const_iterator vit  = indices2.begin(),
                                                        vend = indices2.end();
                         vit != vend;
                         vit++)
                    {
                        const size_t i2 = *vit;
                        if (CurrentFrame
                                .mapPoints[i2 + CurrentFrame.leftKeyPointCount])
                            if (CurrentFrame
                                    .mapPoints[i2 +
                                               CurrentFrame.leftKeyPointCount]
                                    ->getObservationCount() > 0)
                                continue;

                        const cv::Mat &d = CurrentFrame.descriptors.row(
                            i2 + CurrentFrame.leftKeyPointCount);

                        const int distance =
                            computeDescriptorDistance(mapPointDescriptor, d);

                        if (distance < bestDistance)
                        {
                            bestDistance = distance;
                            bestIndex2   = i2;
                        }
                    }

                    if (bestDistance <= TH_HIGH)
                    {
                        CurrentFrame.mapPoints[bestIndex2 +
                                               CurrentFrame.leftKeyPointCount] =
                            p_mapPoint;
                        nmatches++;
                        if (shouldCheckOrientation)
                        {
                            cv::KeyPoint keyPointLf =
                                (LastFrame.leftKeyPointCount == -1)
                                    ? LastFrame.keyPointsUndistorted
                                          [histogramBinIndex]
                                : (histogramBinIndex <
                                   LastFrame.leftKeyPointCount)
                                    ? LastFrame.keyPoints[histogramBinIndex]
                                    : LastFrame.keyPointsRight
                                          [histogramBinIndex -
                                           LastFrame.leftKeyPointCount];

                            cv::KeyPoint keyPointCf =
                                CurrentFrame.keyPointsRight[bestIndex2];

                            float rot = keyPointLf.angle - keyPointCf.angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(
                                bestIndex2 + CurrentFrame.leftKeyPointCount);
                        }
                    }
                }
            }
        }
    }

    // Apply rotation consistency
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
                        static_cast<MapPoint *>(nullptr);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
