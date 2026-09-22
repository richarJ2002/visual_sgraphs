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

int ORBmatcher::searchForTriangulation(
    KeyFrame                     *pKF1,
    KeyFrame                     *pKF2,
    vector<pair<size_t, size_t>> &vMatchedPairs,
    const bool                    bOnlyStereo,
    const bool                    bCoarse)
{
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->featureVector;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->featureVector;

    // Compute epipole in second image
    Sophus::SE3f    T1w = pKF1->getPose();
    Sophus::SE3f    T2w = pKF2->getPose();
    Sophus::SE3f    Tw2 = pKF2->getPoseInverse(); // for convenience
    Eigen::Vector3f Cw  = pKF1->getCameraCenter();
    Eigen::Vector3f C2  = T2w * Cw;

    Eigen::Vector2f ep = pKF2->p_camera->project(C2);
    Sophus::SE3f    T12;
    Sophus::SE3f    Tll, Tlr, Trl, Trr;
    Eigen::Matrix3f R12; // for fastest computation
    Eigen::Vector3f t12; // for fastest computation

    camera_models::geometriccamera::GeometricCamera *pCamera1 = pKF1->p_camera,
                                                    *pCamera2 = pKF2->p_camera;

    if (!pKF1->p_camera2 && !pKF2->p_camera2)
    {
        T12 = T1w * Tw2;
        R12 = T12.rotationMatrix();
        t12 = T12.translation();
    }
    else
    {
        Sophus::SE3f Tr1w = pKF1->getRightPose();
        Sophus::SE3f Twr2 = pKF2->getRightPoseInverse();
        Tll               = T1w * Tw2;
        Tlr               = T1w * Twr2;
        Trl               = Tr1w * Tw2;
        Trr               = Tr1w * Twr2;
    }

    Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr = Tlr.rotationMatrix(),
                    Rrl = Trl.rotationMatrix(), Rrr = Trr.rotationMatrix();
    Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(),
                    trl = Trl.translation(), trr = Trr.translation();

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node
    int          nmatches = 0;
    vector<bool> vbMatched2(pKF2->N, false);
    vector<int>  vMatches12(pKF1->N, -1);

    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f / HISTO_LENGTH;

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

                MapPoint *pMP1 = pKF1->getMapPoint(idx1);

                // If there is already a MapPoint skip
                if (pMP1)
                {
                    continue;
                }

                const bool bStereo1 =
                    (!pKF1->p_camera2 && pKF1->uRight[idx1] >= 0);

                if (bOnlyStereo)
                    if (!bStereo1)
                        continue;

                const cv::KeyPoint &kp1 =
                    (pKF1->Nleft == -1) ? pKF1->keyPointsUndistorted[idx1]
                    : (idx1 < static_cast<size_t>(pKF1->Nleft))
                        ? pKF1->keyPoints[idx1]
                        : pKF1->keyPointsRight[idx1 - pKF1->Nleft];

                const bool bRight1 =
                    (pKF1->Nleft == -1 ||
                     idx1 < static_cast<size_t>(pKF1->Nleft))
                        ? false
                        : true;

                const cv::Mat &d1 = pKF1->descriptors.row(idx1);

                int bestDist = TH_LOW;
                int bestIdx2 = -1;

                for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2;
                     i2++)
                {
                    size_t idx2 = f2it->second[i2];

                    MapPoint *pMP2 = pKF2->getMapPoint(idx2);

                    // If we have already matched or there is a MapPoint skip
                    if (vbMatched2[idx2] || pMP2)
                        continue;

                    const bool bStereo2 =
                        (!pKF2->p_camera2 && pKF2->uRight[idx2] >= 0);

                    if (bOnlyStereo)
                        if (!bStereo2)
                            continue;

                    const cv::Mat &d2 = pKF2->descriptors.row(idx2);

                    const int dist = computeDescriptorDistance(d1, d2);

                    if (dist > TH_LOW || dist > bestDist)
                        continue;

                    const cv::KeyPoint &kp2 =
                        (pKF2->Nleft == -1) ? pKF2->keyPointsUndistorted[idx2]
                        : (idx2 < static_cast<size_t>(pKF2->Nleft))
                            ? pKF2->keyPoints[idx2]
                            : pKF2->keyPointsRight[idx2 - pKF2->Nleft];
                    const bool bRight2 =
                        (pKF2->Nleft == -1 ||
                         idx2 < static_cast<size_t>(pKF2->Nleft))
                            ? false
                            : true;

                    if (!bStereo1 && !bStereo2 && !pKF1->p_camera2)
                    {
                        const float distex = ep(0) - kp2.pt.x;
                        const float distey = ep(1) - kp2.pt.y;
                        if (distex * distex + distey * distey <
                            100 * pKF2->scaleFactors[kp2.octave])
                        {
                            continue;
                        }
                    }

                    if (pKF1->p_camera2 && pKF2->p_camera2)
                    {
                        if (bRight1 && bRight2)
                        {
                            R12 = Rrr;
                            t12 = trr;
                            T12 = Trr;

                            pCamera1 = pKF1->p_camera2;
                            pCamera2 = pKF2->p_camera2;
                        }
                        else if (bRight1 && !bRight2)
                        {
                            R12 = Rrl;
                            t12 = trl;
                            T12 = Trl;

                            pCamera1 = pKF1->p_camera2;
                            pCamera2 = pKF2->p_camera;
                        }
                        else if (!bRight1 && bRight2)
                        {
                            R12 = Rlr;
                            t12 = tlr;
                            T12 = Tlr;

                            pCamera1 = pKF1->p_camera;
                            pCamera2 = pKF2->p_camera2;
                        }
                        else
                        {
                            R12 = Rll;
                            t12 = tll;
                            T12 = Tll;

                            pCamera1 = pKF1->p_camera;
                            pCamera2 = pKF2->p_camera;
                        }
                    }

                    if (bCoarse || pCamera1->epipolarConstrain(
                                       pCamera2,
                                       kp1,
                                       kp2,
                                       R12,
                                       t12,
                                       pKF1->levelSigmaSquared[kp1.octave],
                                       pKF2->levelSigmaSquared
                                           [kp2.octave])) // MODIFICATION_2
                    {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }

                if (bestIdx2 >= 0)
                {
                    const cv::KeyPoint &kp2 =
                        (pKF2->Nleft == -1)
                            ? pKF2->keyPointsUndistorted[bestIdx2]
                        : (bestIdx2 < pKF2->Nleft)
                            ? pKF2->keyPoints[bestIdx2]
                            : pKF2->keyPointsRight[bestIdx2 - pKF2->Nleft];
                    vMatches12[idx1] = bestIdx2;
                    nmatches++;

                    if (mbCheckOrientation)
                    {
                        float rot = kp1.angle - kp2.angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
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
                vMatches12[rotHist[i][j]] = -1;
                nmatches--;
            }
        }
    }

    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (vMatches12[i] < 0)
            continue;
        vMatchedPairs.push_back(make_pair(i, vMatches12[i]));
    }

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
