/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "Frame.h"

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "G2oTypes.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "StereoMatchOutlierRejection.h"
#include "Utils/Converter/objects/Converter.h"

#include <thread>

namespace vs_graphs
{
namespace core
{

void Frame::computeStereoMatches()
{
    uRight = vector<float>(N, -1.0f);
    depths = vector<float>(N, -1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

    const int nRows = p_orbExtractorLeft->imagePyramid[0].rows;

    // Assign keypoints to row table
    vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

    for (int i = 0; i < nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = keyPointsRight.size();

    for (int iR = 0; iR < Nr; iR++)
    {
        const cv::KeyPoint &kp  = keyPointsRight[iR];
        const float        &kpY = kp.pt.y;
        const float         r = 2.0f * scaleFactors[keyPointsRight[iR].octave];
        const int           maxr = ceil(kpY + r);
        const int           minr = floor(kpY - r);

        for (int yi = minr; yi <= maxr; yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf / minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int>> vDistIdx;
    vDistIdx.reserve(N);

    for (int iL = 0; iL < N; iL++)
    {
        const cv::KeyPoint &kpL    = keyPoints[iL];
        const int          &levelL = kpL.octave;
        const float        &vL     = kpL.pt.y;
        const float        &uL     = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if (vCandidates.empty())
            continue;

        const float minU = uL - maxD;
        const float maxU = uL - minD;

        if (maxU < 0)
            continue;

        int    bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = descriptors.row(iL);

        // Compare descriptor to right keypoints
        for (size_t iC = 0; iC < vCandidates.size(); iC++)
        {
            const size_t        iR  = vCandidates[iC];
            const cv::KeyPoint &kpR = keyPointsRight[iR];

            if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                continue;

            const float &uR = kpR.pt.x;

            if (uR >= minU && uR <= maxU)
            {
                const cv::Mat &dR = descriptorsRight.row(iR);
                const int dist = ORBmatcher::computeDescriptorDistance(dL, dR);

                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if (bestDist < thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0         = keyPointsRight[bestIdxR].pt.x;
            const float scaleFactor = invScaleFactors[kpL.octave];
            const float scaleduL    = round(kpL.pt.x * scaleFactor);
            const float scaledvL    = round(kpL.pt.y * scaleFactor);
            const float scaleduR0   = round(uR0 * scaleFactor);

            // sliding window search
            const int w  = 5;
            cv::Mat   IL = p_orbExtractorLeft->imagePyramid[kpL.octave]
                             .rowRange(scaledvL - w, scaledvL + w + 1)
                             .colRange(scaleduL - w, scaleduL + w + 1);

            int           bestDist = INT_MAX;
            int           bestincR = 0;
            const int     L        = 5;
            vector<float> vDists;
            vDists.resize(2 * L + 1);

            const float iniu = scaleduR0 + L - w;
            const float endu = scaleduR0 + L + w + 1;
            if (iniu < 0 ||
                endu >= p_orbExtractorRight->imagePyramid[kpL.octave].cols)
                continue;

            for (int incR = -L; incR <= +L; incR++)
            {
                cv::Mat IR = p_orbExtractorRight->imagePyramid[kpL.octave]
                                 .rowRange(scaledvL - w, scaledvL + w + 1)
                                 .colRange(scaleduR0 + incR - w,
                                           scaleduR0 + incR + w + 1);

                float dist = cv::norm(IL, IR, cv::NORM_L1);
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestincR = incR;
                }

                vDists[L + incR] = dist;
            }

            if (bestincR == -L || bestincR == L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L + bestincR - 1];
            const float dist2 = vDists[L + bestincR];
            const float dist3 = vDists[L + bestincR + 1];

            const float deltaR =
                (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

            if (deltaR < -1 || deltaR > 1)
                continue;

            // Re-scaled coordinate
            float bestuR = scaleFactors[kpL.octave] *
                           ((float)scaleduR0 + (float)bestincR + deltaR);

            float disparity = (uL - bestuR);

            if (disparity >= minD && disparity < maxD)
            {
                if (disparity <= 0)
                {
                    disparity = 0.01;
                    bestuR    = uL - 0.01;
                }
                depths[iL] = mbf / disparity;
                uRight[iL] = bestuR;
                vDistIdx.push_back(pair<int, int>(bestDist, iL));
            }
        }
    }

    rejectOutlierStereoMatches(vDistIdx, uRight, depths);
}

} // namespace core
} // namespace vs_graphs
