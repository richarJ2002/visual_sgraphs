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

/*!
 * @file            searchForInitialization.cc
 *
 * @brief           Implements ORBmatcher::searchForInitialization(), declared
 *                  in ORBmatcher.h.
 */

#include "ORBmatcher.h"

#include <limits.h>

#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <cstdint>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

ORBmatcherStatus ORBmatcher::searchForInitialization(
    Frame                    &frame1_inout,
    Frame                    &frame2_inout,
    std::vector<cv::Point2f> &previousMatched_inout,
    std::vector<int>         &matches12_out,
    int                      &forInitialization_out,
    int                       windowSize_in)
{
    int nmatches = 0;
    matches12_out =
        std::vector<int>(frame1_inout.keyPointsUndistorted.size(), -1);

    std::vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    std::vector<int> matchedDistances(frame2_inout.keyPointsUndistorted.size(),
                                      INT_MAX);
    std::vector<int> matchIndices21(frame2_inout.keyPointsUndistorted.size(),
                                    -1);

    for (size_t i1 = 0, iend1 = frame1_inout.keyPointsUndistorted.size();
         i1 < iend1;
         i1++)
    {
        cv::KeyPoint keyPoint1 = frame1_inout.keyPointsUndistorted[i1];
        int          level1    = keyPoint1.octave;
        if (level1 > 0)
            continue;

        std::vector<size_t> indices2{};
        if (frame2_inout.getFeaturesInArea(previousMatched_inout[i1].x,
                                           previousMatched_inout[i1].y,
                                           windowSize_in,
                                           indices2,
                                           level1,
                                           level1) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFeaturesInArea returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (indices2.empty())
            continue;

        cv::Mat d1 = frame1_inout.descriptors.row(i1);

        int bestDistance  = INT_MAX;
        int bestDistance2 = INT_MAX;
        int bestIndex2    = -1;

        for (std::vector<size_t>::iterator vit = indices2.begin();
             vit != indices2.end();
             vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = frame2_inout.descriptors.row(i2);

            int distance{};
            if (computeDescriptorDistance(d1, d2, distance) !=
                ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computeDescriptorDistance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

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
            if (bestDistance <
                static_cast<float>(bestDistance2) * nearestNeighborRatio)
            {
                if (matchIndices21[bestIndex2] >= 0)
                {
                    matches12_out[matchIndices21[bestIndex2]] = -1;
                    nmatches--;
                }
                matches12_out[i1]            = bestIndex2;
                matchIndices21[bestIndex2]   = i1;
                matchedDistances[bestIndex2] = bestDistance;
                nmatches++;

                if (shouldCheckOrientation)
                {
                    float rot =
                        frame1_inout.keyPointsUndistorted[i1].angle -
                        frame2_inout.keyPointsUndistorted[bestIndex2].angle;
                    if (rot < 0.0)
                        rot += 360.0f;
                    int bin = std::round(rot * factor);
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

        if (computeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3) !=
            ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeThreeMaxima returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

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
                if (matches12_out[index1] >= 0)
                {
                    matches12_out[index1] = -1;
                    nmatches--;
                }
            }
        }
    }

    // Update prev matched
    for (size_t i1 = 0, iend1 = matches12_out.size(); i1 < iend1; i1++)
        if (matches12_out[i1] >= 0)
            previousMatched_inout[i1] =
                frame2_inout.keyPointsUndistorted[matches12_out[i1]].pt;

    forInitialization_out = nmatches;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
