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
    for (int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    const Sophus::SE3f    Tcw = CurrentFrame.getPose();
    const Eigen::Vector3f twc = Tcw.inverse().translation();

    const Sophus::SE3f    Tlw = LastFrame.getPose();
    const Eigen::Vector3f tlc = Tlw * twc;

    const bool bForward  = tlc(2) > CurrentFrame.mb && !bMono;
    const bool bBackward = -tlc(2) > CurrentFrame.mb && !bMono;

    for (int i = 0; i < LastFrame.N; i++)
    {
        MapPoint *pMP = LastFrame.mapPoints[i];
        if (pMP)
        {
            if (!LastFrame.outlierFlags[i])
            {
                // Project
                Eigen::Vector3f x3Dw = pMP->getWorldPos();
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

                int nLastOctave =
                    (LastFrame.Nleft == -1 || i < LastFrame.Nleft)
                        ? LastFrame.keyPoints[i].octave
                        : LastFrame.keyPointsRight[i - LastFrame.Nleft].octave;

                // Search in a window. Size depends on scale
                float radius = th * CurrentFrame.scaleFactors[nLastOctave];

                vector<size_t> vIndices2;

                if (bForward)
                    vIndices2 = CurrentFrame.getFeaturesInArea(uv(0),
                                                               uv(1),
                                                               radius,
                                                               nLastOctave);
                else if (bBackward)
                    vIndices2 = CurrentFrame.getFeaturesInArea(uv(0),
                                                               uv(1),
                                                               radius,
                                                               0,
                                                               nLastOctave);
                else
                    vIndices2 = CurrentFrame.getFeaturesInArea(uv(0),
                                                               uv(1),
                                                               radius,
                                                               nLastOctave - 1,
                                                               nLastOctave + 1);

                if (vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->getDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for (vector<size_t>::const_iterator vit  = vIndices2.begin(),
                                                    vend = vIndices2.end();
                     vit != vend;
                     vit++)
                {
                    const size_t i2 = *vit;

                    if (CurrentFrame.mapPoints[i2])
                        if (CurrentFrame.mapPoints[i2]->getObservationCount() >
                            0)
                            continue;

                    if (CurrentFrame.Nleft == -1 && CurrentFrame.uRight[i2] > 0)
                    {
                        const float ur = uv(0) - CurrentFrame.mbf * invzc;
                        const float er = fabs(ur - CurrentFrame.uRight[i2]);
                        if (er > radius)
                            continue;
                    }

                    const cv::Mat &d = CurrentFrame.descriptors.row(i2);

                    const int dist = computeDescriptorDistance(dMP, d);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdx2 = i2;
                    }
                }

                if (bestDist <= TH_HIGH)
                {
                    CurrentFrame.mapPoints[bestIdx2] = pMP;
                    nmatches++;

                    if (mbCheckOrientation)
                    {
                        cv::KeyPoint kpLF =
                            (LastFrame.Nleft == -1)
                                ? LastFrame.keyPointsUndistorted[i]
                            : (i < LastFrame.Nleft)
                                ? LastFrame.keyPoints[i]
                                : LastFrame.keyPointsRight[i - LastFrame.Nleft];

                        cv::KeyPoint kpCF =
                            (CurrentFrame.Nleft == -1)
                                ? CurrentFrame.keyPointsUndistorted[bestIdx2]
                            : (bestIdx2 < CurrentFrame.Nleft)
                                ? CurrentFrame.keyPoints[bestIdx2]
                                : CurrentFrame
                                      .keyPointsRight[bestIdx2 -
                                                      CurrentFrame.Nleft];
                        float rot = kpLF.angle - kpCF.angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
                if (CurrentFrame.Nleft != -1)
                {
                    Eigen::Vector3f x3Dr =
                        CurrentFrame.getRelativePoseTrl() * x3Dc;
                    Eigen::Vector2f uv = CurrentFrame.p_camera->project(x3Dr);

                    int nLastOctave =
                        (LastFrame.Nleft == -1 || i < LastFrame.Nleft)
                            ? LastFrame.keyPoints[i].octave
                            : LastFrame.keyPointsRight[i - LastFrame.Nleft]
                                  .octave;

                    // Search in a window. Size depends on scale
                    float radius = th * CurrentFrame.scaleFactors[nLastOctave];

                    vector<size_t> vIndices2;

                    if (bForward)
                        vIndices2 = CurrentFrame.getFeaturesInArea(uv(0),
                                                                   uv(1),
                                                                   radius,
                                                                   nLastOctave,
                                                                   -1,
                                                                   true);
                    else if (bBackward)
                        vIndices2 = CurrentFrame.getFeaturesInArea(uv(0),
                                                                   uv(1),
                                                                   radius,
                                                                   0,
                                                                   nLastOctave,
                                                                   true);
                    else
                        vIndices2 =
                            CurrentFrame.getFeaturesInArea(uv(0),
                                                           uv(1),
                                                           radius,
                                                           nLastOctave - 1,
                                                           nLastOctave + 1,
                                                           true);

                    const cv::Mat dMP = pMP->getDescriptor();

                    int bestDist = 256;
                    int bestIdx2 = -1;

                    for (vector<size_t>::const_iterator vit = vIndices2.begin(),
                                                        vend = vIndices2.end();
                         vit != vend;
                         vit++)
                    {
                        const size_t i2 = *vit;
                        if (CurrentFrame.mapPoints[i2 + CurrentFrame.Nleft])
                            if (CurrentFrame.mapPoints[i2 + CurrentFrame.Nleft]
                                    ->getObservationCount() > 0)
                                continue;

                        const cv::Mat &d = CurrentFrame.descriptors.row(
                            i2 + CurrentFrame.Nleft);

                        const int dist = computeDescriptorDistance(dMP, d);

                        if (dist < bestDist)
                        {
                            bestDist = dist;
                            bestIdx2 = i2;
                        }
                    }

                    if (bestDist <= TH_HIGH)
                    {
                        CurrentFrame.mapPoints[bestIdx2 + CurrentFrame.Nleft] =
                            pMP;
                        nmatches++;
                        if (mbCheckOrientation)
                        {
                            cv::KeyPoint kpLF =
                                (LastFrame.Nleft == -1)
                                    ? LastFrame.keyPointsUndistorted[i]
                                : (i < LastFrame.Nleft)
                                    ? LastFrame.keyPoints[i]
                                    : LastFrame
                                          .keyPointsRight[i - LastFrame.Nleft];

                            cv::KeyPoint kpCF =
                                CurrentFrame.keyPointsRight[bestIdx2];

                            float rot = kpLF.angle - kpCF.angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx2 +
                                                   CurrentFrame.Nleft);
                        }
                    }
                }
            }
        }
    }

    // Apply rotation consistency
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
                    CurrentFrame.mapPoints[rotHist[i][j]] =
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
