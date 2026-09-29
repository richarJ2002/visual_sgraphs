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

#include <rclcpp/logging.hpp>
#include <thread>

namespace vs_graphs
{
namespace core
{

void Frame::computeStereoMatches()
{
    uRight = vector<float>(keyPointCount, -1.0f);
    depths = vector<float>(keyPointCount, -1.0f);

    const int thresholdOrbDistance =
        (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

    const int rowCount = p_orbExtractorLeft->imagePyramid[0].rows;

    // Assign keypoints to row table
    vector<vector<size_t>> rowIndices(rowCount, vector<size_t>());

    for (int rowIndex = 0; rowIndex < rowCount; rowIndex++)
        rowIndices[rowIndex].reserve(200);

    const int Nr = keyPointsRight.size();

    for (int iR = 0; iR < Nr; iR++)
    {
        const cv::KeyPoint &keyPoint  = keyPointsRight[iR];
        const float        &keyPointY = keyPoint.pt.y;
        const float         r = 2.0f * scaleFactors[keyPointsRight[iR].octave];
        const int           maxr = ceil(keyPointY + r);
        const int           minr = floor(keyPointY - r);

        for (int yi = minr; yi <= maxr; yi++)
            rowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minimumZ = mb;
    const float minimumD = 0;
    const float maximumD = mbf / minimumZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int>> distanceIndices;
    distanceIndices.reserve(keyPointCount);

    for (int iL = 0; iL < keyPointCount; iL++)
    {
        const cv::KeyPoint &keyPointL = keyPoints[iL];
        const int          &levelL    = keyPointL.octave;
        const float        &vL        = keyPointL.pt.y;
        const float        &uL        = keyPointL.pt.x;

        const vector<size_t> &candidates = rowIndices[vL];

        if (candidates.empty())
            continue;

        const float minimumU = uL - maximumD;
        const float maximumU = uL - minimumD;

        if (maximumU < 0)
            continue;

        int    bestDistance = ORBmatcher::TH_HIGH;
        size_t bestIndexR   = 0;

        const cv::Mat &dL = descriptors.row(iL);

        // Compare descriptor to right keypoints
        for (size_t iC = 0; iC < candidates.size(); iC++)
        {
            const size_t        iR        = candidates[iC];
            const cv::KeyPoint &keyPointR = keyPointsRight[iR];

            if (keyPointR.octave < levelL - 1 || keyPointR.octave > levelL + 1)
                continue;

            const float &uR = keyPointR.pt.x;

            if (uR >= minimumU && uR <= maximumU)
            {
                const cv::Mat &dR = descriptorsRight.row(iR);
                const int      distance =
                    ORBmatcher::computeDescriptorDistance(dL, dR);

                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestIndexR   = iR;
                }
            }
        }

        // Subpixel match by correlation
        if (bestDistance < thresholdOrbDistance)
        {
            // coordinates in image pyramid at keypoint scale
            const float bestRightU  = keyPointsRight[bestIndexR].pt.x;
            const float scaleFactor = invScaleFactors[keyPointL.octave];
            const float scaleduL    = round(keyPointL.pt.x * scaleFactor);
            const float scaledvL    = round(keyPointL.pt.y * scaleFactor);
            const float scaleduR0   = round(bestRightU * scaleFactor);

            // sliding window search
            const int w  = 5;
            cv::Mat   IL = p_orbExtractorLeft->imagePyramid[keyPointL.octave]
                             .rowRange(scaledvL - w, scaledvL + w + 1)
                             .colRange(scaleduL - w, scaleduL + w + 1);

            int           bestDistance = INT_MAX;
            int           bestincR     = 0;
            const int     L            = 5;
            vector<float> dists;
            dists.resize(2 * L + 1);

            const float iniu = scaleduR0 + L - w;
            const float endu = scaleduR0 + L + w + 1;
            if (iniu < 0 ||
                endu >=
                    p_orbExtractorRight->imagePyramid[keyPointL.octave].cols)
                continue;

            for (int incR = -L; incR <= +L; incR++)
            {
                cv::Mat IR = p_orbExtractorRight->imagePyramid[keyPointL.octave]
                                 .rowRange(scaledvL - w, scaledvL + w + 1)
                                 .colRange(scaleduR0 + incR - w,
                                           scaleduR0 + incR + w + 1);

                float distance = cv::norm(IL, IR, cv::NORM_L1);
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestincR     = incR;
                }

                dists[L + incR] = distance;
            }

            if (bestincR == -L || bestincR == L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float distance1 = dists[L + bestincR - 1];
            const float distance2 = dists[L + bestincR];
            const float distance3 = dists[L + bestincR + 1];

            const float deltaR =
                (distance1 - distance3) /
                (2.0f * (distance1 + distance3 - 2.0f * distance2));

            if (deltaR < -1 || deltaR > 1)
                continue;

            // Re-scaled coordinate
            float bestuR = scaleFactors[keyPointL.octave] *
                           ((float)scaleduR0 + (float)bestincR + deltaR);

            float disparity = (uL - bestuR);

            if (disparity >= minimumD && disparity < maximumD)
            {
                if (disparity <= 0)
                {
                    disparity = 0.01;
                    bestuR    = uL - 0.01;
                }
                depths[iL] = mbf / disparity;
                uRight[iL] = bestuR;
                distanceIndices.push_back(pair<int, int>(bestDistance, iL));
            }
        }
    }

    if (rejectOutlierStereoMatches(distanceIndices, uRight, depths) !=
        StereoMatchOutlierRejectionStatus::
            STEREO_MATCH_OUTLIER_REJECTION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: rejectOutlierStereoMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

} // namespace core
} // namespace vs_graphs
