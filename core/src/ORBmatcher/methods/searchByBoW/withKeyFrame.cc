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

int ORBmatcher::searchByBoW(KeyFrame           *pKF1,
                            KeyFrame           *pKF2,
                            vector<MapPoint *> &vpMatches12)
{
    const vector<cv::KeyPoint> &vKeysUn1     = pKF1->keyPointsUndistorted;
    const DBoW2::FeatureVector &vFeatVec1    = pKF1->featureVector;
    const vector<MapPoint *>    vpMapPoints1 = pKF1->getMapPointMatches();
    const cv::Mat              &Descriptors1 = pKF1->descriptors;

    const vector<cv::KeyPoint> &vKeysUn2     = pKF2->keyPointsUndistorted;
    const DBoW2::FeatureVector &vFeatVec2    = pKF2->featureVector;
    const vector<MapPoint *>    vpMapPoints2 = pKF2->getMapPointMatches();
    const cv::Mat              &Descriptors2 = pKF2->descriptors;

    vpMatches12 = vector<MapPoint *>(vpMapPoints1.size(),
                                     static_cast<MapPoint *>(nullptr));
    vector<bool> vbMatched2(vpMapPoints2.size(), false);

    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f / HISTO_LENGTH;

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator f1it  = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it  = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while (f1it != f1end && f2it != f2end)
    {
        if (f1it->first == f2it->first)
        {
            for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];
                if (pKF1->Nleft != -1 &&
                    idx1 >= pKF1->keyPointsUndistorted.size())
                {
                    continue;
                }

                MapPoint *pMP1 = vpMapPoints1[idx1];
                if (!pMP1)
                    continue;
                if (pMP1->isBad())
                    continue;

                const cv::Mat &d1 = Descriptors1.row(idx1);

                int bestDist1 = 256;
                int bestIdx2  = -1;
                int bestDist2 = 256;

                for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2;
                     i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    if (pKF2->Nleft != -1 &&
                        idx2 >= pKF2->keyPointsUndistorted.size())
                    {
                        continue;
                    }

                    MapPoint *pMP2 = vpMapPoints2[idx2];

                    if (vbMatched2[idx2] || !pMP2)
                        continue;

                    if (pMP2->isBad())
                        continue;

                    const cv::Mat &d2 = Descriptors2.row(idx2);

                    int dist = computeDescriptorDistance(d1, d2);

                    if (dist < bestDist1)
                    {
                        bestDist2 = bestDist1;
                        bestDist1 = dist;
                        bestIdx2  = idx2;
                    }
                    else if (dist < bestDist2)
                    {
                        bestDist2 = dist;
                    }
                }

                if (bestDist1 < TH_LOW)
                {
                    if (static_cast<float>(bestDist1) <
                        mfNNratio * static_cast<float>(bestDist2))
                    {
                        vpMatches12[idx1]    = vpMapPoints2[bestIdx2];
                        vbMatched2[bestIdx2] = true;

                        if (mbCheckOrientation)
                        {
                            float rot =
                                vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if (f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
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
                vpMatches12[rotHist[i][j]] = static_cast<MapPoint *>(nullptr);
                nmatches--;
            }
        }
    }

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
