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

#include <rclcpp/logging.hpp>
#include <stdint-gcc.h>

namespace vs_graphs
{
namespace core
{

int ORBmatcher::searchForInitialization(
    Frame               &F1,
    Frame               &F2,
    vector<cv::Point2f> &previousMatched_inout,
    vector<int>         &vnMatches12,
    int                  windowSize)
{
    int nmatches = 0;
    vnMatches12  = vector<int>(F1.keyPointsUndistorted.size(), -1);

    vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    vector<int> matchedDistances(F2.keyPointsUndistorted.size(), INT_MAX);
    vector<int> matchIndices21(F2.keyPointsUndistorted.size(), -1);

    for (size_t i1 = 0, iend1 = F1.keyPointsUndistorted.size(); i1 < iend1;
         i1++)
    {
        cv::KeyPoint keyPoint1 = F1.keyPointsUndistorted[i1];
        int          level1    = keyPoint1.octave;
        if (level1 > 0)
            continue;

        std::vector<size_t> indices2{};
        if (F2.getFeaturesInArea(previousMatched_inout[i1].x,
                                 previousMatched_inout[i1].y,
                                 windowSize,
                                 indices2,
                                 level1,
                                 level1) != FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFeaturesInArea returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (indices2.empty())
            continue;

        cv::Mat d1 = F1.descriptors.row(i1);

        int bestDistance  = INT_MAX;
        int bestDistance2 = INT_MAX;
        int bestIndex2    = -1;

        for (vector<size_t>::iterator vit = indices2.begin();
             vit != indices2.end();
             vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.descriptors.row(i2);

            int distance = computeDescriptorDistance(d1, d2);

            if (matchedDistances[i2] <= distance)
                continue;

            if (distance < bestDistance)
            {
                bestDistance2 = bestDistance;
                bestDistance  = distance;
                bestIndex2    = i2;
            }
            else if (distance < bestDistance2)
            {
                bestDistance2 = distance;
            }
        }

        if (bestDistance <= TH_LOW)
        {
            if (bestDistance < (float)bestDistance2 * nearestNeighborRatio)
            {
                if (matchIndices21[bestIndex2] >= 0)
                {
                    vnMatches12[matchIndices21[bestIndex2]] = -1;
                    nmatches--;
                }
                vnMatches12[i1]              = bestIndex2;
                matchIndices21[bestIndex2]   = i1;
                matchedDistances[bestIndex2] = bestDistance;
                nmatches++;

                if (shouldCheckOrientation)
                {
                    float rot = F1.keyPointsUndistorted[i1].angle -
                                F2.keyPointsUndistorted[bestIndex2].angle;
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

    if (shouldCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        computeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
             histogramBinIndex++)
        {
            if (histogramBinIndex == ind1 || histogramBinIndex == ind2 ||
                histogramBinIndex == ind3)
                continue;
            for (size_t binEntryIndex = 0,
                        jend          = rotHist[histogramBinIndex].size();
                 binEntryIndex < jend;
                 binEntryIndex++)
            {
                int index1 = rotHist[histogramBinIndex][binEntryIndex];
                if (vnMatches12[index1] >= 0)
                {
                    vnMatches12[index1] = -1;
                    nmatches--;
                }
            }
        }
    }

    // Update prev matched
    for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
        if (vnMatches12[i1] >= 0)
            previousMatched_inout[i1] =
                F2.keyPointsUndistorted[vnMatches12[i1]].pt;

    return nmatches;
}

} // namespace core
} // namespace vs_graphs
