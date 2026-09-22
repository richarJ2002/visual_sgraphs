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

int ORBmatcher::searchForInitialization(Frame               &F1,
                                        Frame               &F2,
                                        vector<cv::Point2f> &vbPrevMatched,
                                        vector<int>         &vnMatches12,
                                        int                  windowSize)
{
    int nmatches = 0;
    vnMatches12  = vector<int>(F1.keyPointsUndistorted.size(), -1);

    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.keyPointsUndistorted.size(), INT_MAX);
    vector<int> vnMatches21(F2.keyPointsUndistorted.size(), -1);

    for (size_t i1 = 0, iend1 = F1.keyPointsUndistorted.size(); i1 < iend1;
         i1++)
    {
        cv::KeyPoint kp1    = F1.keyPointsUndistorted[i1];
        int          level1 = kp1.octave;
        if (level1 > 0)
            continue;

        vector<size_t> vIndices2 = F2.getFeaturesInArea(vbPrevMatched[i1].x,
                                                        vbPrevMatched[i1].y,
                                                        windowSize,
                                                        level1,
                                                        level1);

        if (vIndices2.empty())
            continue;

        cv::Mat d1 = F1.descriptors.row(i1);

        int bestDist  = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2  = -1;

        for (vector<size_t>::iterator vit = vIndices2.begin();
             vit != vIndices2.end();
             vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.descriptors.row(i2);

            int dist = computeDescriptorDistance(d1, d2);

            if (vMatchedDistance[i2] <= dist)
                continue;

            if (dist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist  = dist;
                bestIdx2  = i2;
            }
            else if (dist < bestDist2)
            {
                bestDist2 = dist;
            }
        }

        if (bestDist <= TH_LOW)
        {
            if (bestDist < (float)bestDist2 * mfNNratio)
            {
                if (vnMatches21[bestIdx2] >= 0)
                {
                    vnMatches12[vnMatches21[bestIdx2]] = -1;
                    nmatches--;
                }
                vnMatches12[i1]            = bestIdx2;
                vnMatches21[bestIdx2]      = i1;
                vMatchedDistance[bestIdx2] = bestDist;
                nmatches++;

                if (mbCheckOrientation)
                {
                    float rot = F1.keyPointsUndistorted[i1].angle -
                                F2.keyPointsUndistorted[bestIdx2].angle;
                    if (rot < 0.0)
                        rot += 360.0f;
                    int bin = round(rot * factor);
                    if (bin == HISTO_LENGTH)
                        bin = 0;
                    assert(bin >= 0 && bin < HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
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
            if (i == ind1 || i == ind2 || i == ind3)
                continue;
            for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
            {
                int idx1 = rotHist[i][j];
                if (vnMatches12[idx1] >= 0)
                {
                    vnMatches12[idx1] = -1;
                    nmatches--;
                }
            }
        }
    }

    // Update prev matched
    for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
        if (vnMatches12[i1] >= 0)
            vbPrevMatched[i1] = F2.keyPointsUndistorted[vnMatches12[i1]].pt;

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
