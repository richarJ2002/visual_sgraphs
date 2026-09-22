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

int ORBmatcher::searchByBoW(KeyFrame           *pKF,
                            Frame              &F,
                            vector<MapPoint *> &vpMapPointMatches)
{
    const vector<MapPoint *> vpMapPointsKF = pKF->getMapPointMatches();

    vpMapPointMatches =
        vector<MapPoint *>(F.N, static_cast<MapPoint *>(nullptr));

    const DBoW2::FeatureVector &vFeatVecKF = pKF->featureVector;

    int nmatches = 0;

    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node
    // (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit  = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit   = F.featureVector.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend  = F.featureVector.end();

    while (KFit != KFend && Fit != Fend)
    {
        if (KFit->first == Fit->first)
        {
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF  = Fit->second;

            for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];

                MapPoint *pMP = vpMapPointsKF[realIdxKF];

                if (!pMP)
                    continue;

                if (pMP->isBad())
                    continue;

                const cv::Mat &dKF = pKF->descriptors.row(realIdxKF);

                int bestDist1 = 256;
                int bestIdxF  = -1;
                int bestDist2 = 256;

                int bestDist1R = 256;
                int bestIdxFR  = -1;
                int bestDist2R = 256;

                for (size_t iF = 0; iF < vIndicesF.size(); iF++)
                {
                    if (F.Nleft == -1)
                    {
                        const unsigned int realIdxF = vIndicesF[iF];

                        if (vpMapPointMatches[realIdxF])
                            continue;

                        const cv::Mat &dF = F.descriptors.row(realIdxF);

                        const int dist = computeDescriptorDistance(dKF, dF);

                        if (dist < bestDist1)
                        {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF  = realIdxF;
                        }
                        else if (dist < bestDist2)
                        {
                            bestDist2 = dist;
                        }
                    }
                    else
                    {
                        const unsigned int realIdxF = vIndicesF[iF];

                        if (vpMapPointMatches[realIdxF])
                            continue;

                        const cv::Mat &dF = F.descriptors.row(realIdxF);

                        const int dist = computeDescriptorDistance(dKF, dF);

                        if (realIdxF < static_cast<unsigned int>(F.Nleft) &&
                            dist < bestDist1)
                        {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdxF  = realIdxF;
                        }
                        else if (realIdxF <
                                     static_cast<unsigned int>(F.Nleft) &&
                                 dist < bestDist2)
                        {
                            bestDist2 = dist;
                        }

                        if (realIdxF >= static_cast<unsigned int>(F.Nleft) &&
                            dist < bestDist1R)
                        {
                            bestDist2R = bestDist1R;
                            bestDist1R = dist;
                            bestIdxFR  = realIdxF;
                        }
                        else if (realIdxF >=
                                     static_cast<unsigned int>(F.Nleft) &&
                                 dist < bestDist2R)
                        {
                            bestDist2R = dist;
                        }
                    }
                }

                if (bestDist1 <= TH_LOW)
                {
                    if (static_cast<float>(bestDist1) <
                        mfNNratio * static_cast<float>(bestDist2))
                    {
                        vpMapPointMatches[bestIdxF] = pMP;

                        const cv::KeyPoint &kp =
                            (!pKF->p_camera2)
                                ? pKF->keyPointsUndistorted[realIdxKF]
                            : (realIdxKF >= static_cast<unsigned int>(
                                                pKF->Nleft))
                                ? pKF->keyPointsRight[realIdxKF - pKF->Nleft]
                                : pKF->keyPoints[realIdxKF];

                        if (mbCheckOrientation)
                        {
                            cv::KeyPoint &Fkp =
                                (!pKF->p_camera2 || F.Nleft == -1)
                                    ? F.keyPoints[bestIdxF]
                                : (bestIdxF >= F.Nleft)
                                    ? F.keyPointsRight[bestIdxF - F.Nleft]
                                    : F.keyPoints[bestIdxF];

                            float rot = kp.angle - Fkp.angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);
                        }
                        nmatches++;
                    }

                    if (bestDist1R <= TH_LOW)
                    {
                        if (static_cast<float>(bestDist1R) <
                                mfNNratio * static_cast<float>(bestDist2R) ||
                            true)
                        {
                            vpMapPointMatches[bestIdxFR] = pMP;

                            const cv::KeyPoint &kp =
                                (!pKF->p_camera2)
                                    ? pKF->keyPointsUndistorted[realIdxKF]
                                : (realIdxKF >= static_cast<unsigned int>(
                                                    pKF->Nleft))
                                    ? pKF->keyPointsRight[realIdxKF -
                                                          pKF->Nleft]
                                    : pKF->keyPoints[realIdxKF];

                            if (mbCheckOrientation)
                            {
                                cv::KeyPoint &Fkp =
                                    (!F.p_camera2) ? F.keyPoints[bestIdxFR]
                                    : (bestIdxFR >= F.Nleft)
                                        ? F.keyPointsRight[bestIdxFR - F.Nleft]
                                        : F.keyPoints[bestIdxFR];

                                float rot = kp.angle - Fkp.angle;
                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = round(rot * factor);
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                assert(bin >= 0 && bin < HISTO_LENGTH);
                                rotHist[bin].push_back(bestIdxFR);
                            }
                            nmatches++;
                        }
                    }
                }
            }

            KFit++;
            Fit++;
        }
        else if (KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = F.featureVector.lower_bound(KFit->first);
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
                vpMapPointMatches[rotHist[i][j]] =
                    static_cast<MapPoint *>(nullptr);
                nmatches--;
            }
        }
    }

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
