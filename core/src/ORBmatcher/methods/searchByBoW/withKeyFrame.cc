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
 * @file            withKeyFrame.cc
 *
 * @brief           Implements ORBmatcher::searchByBoW() (withKeyFrame),
 *                  declared in ORBmatcher.h.
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

ORBmatcherStatus ORBmatcher::searchByBoW(KeyFrame                *pKF1,
                                         KeyFrame                *pKF2,
                                         std::vector<MapPoint *> &vpMatches12,
                                         int                     &byBoW_out)
{
    const std::vector<cv::KeyPoint> &undistortedKeyPoints1 =
        pKF1->keyPointsUndistorted;
    const DBoW2::FeatureVector &featureVector1 = pKF1->featureVector;
    std::vector<MapPoint *>     mapPoints1{};
    if (pKF1->getMapPointMatches(mapPoints1) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const cv::Mat &descriptors1 = pKF1->descriptors;

    const std::vector<cv::KeyPoint> &undistortedKeyPoints2 =
        pKF2->keyPointsUndistorted;
    const DBoW2::FeatureVector &featureVector2 = pKF2->featureVector;
    std::vector<MapPoint *>     mapPoints2{};
    if (pKF2->getMapPointMatches(mapPoints2) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const cv::Mat &descriptors2 = pKF2->descriptors;

    vpMatches12 = std::vector<MapPoint *>(mapPoints1.size(),
                                          static_cast<MapPoint *>(nullptr));
    std::vector<bool> matched2Flags(mapPoints2.size(), false);

    std::vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);

    const float factor = 1.0f / HISTO_LENGTH;

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator firstFeatureIt =
        featureVector1.begin();
    DBoW2::FeatureVector::const_iterator secondFeatureIt =
        featureVector2.begin();
    DBoW2::FeatureVector::const_iterator firstFeatureEnd = featureVector1.end();
    DBoW2::FeatureVector::const_iterator secondFeatureEnd =
        featureVector2.end();

    while (firstFeatureIt != firstFeatureEnd &&
           secondFeatureIt != secondFeatureEnd)
    {
        if (firstFeatureIt->first == secondFeatureIt->first)
        {
            for (size_t i1 = 0, iend1 = firstFeatureIt->second.size();
                 i1 < iend1;
                 i1++)
            {
                const size_t index1 = firstFeatureIt->second[i1];
                if (pKF1->leftKeyPointCount != -1 &&
                    index1 >= pKF1->keyPointsUndistorted.size())
                {
                    continue;
                }

                MapPoint *p_mapPoint1 = mapPoints1[index1];
                if (!p_mapPoint1)
                    continue;
                bool mapPoint1IsBad{};
                if (p_mapPoint1->isBad(mapPoint1IsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (mapPoint1IsBad)
                    continue;

                const cv::Mat &d1 = descriptors1.row(index1);

                int bestDistance1 = 256;
                int bestIndex2    = -1;
                int bestDistance2 = 256;

                for (size_t i2 = 0, iend2 = secondFeatureIt->second.size();
                     i2 < iend2;
                     i2++)
                {
                    const size_t index2 = secondFeatureIt->second[i2];

                    if (pKF2->leftKeyPointCount != -1 &&
                        index2 >= pKF2->keyPointsUndistorted.size())
                    {
                        continue;
                    }

                    MapPoint *p_mapPoint2 = mapPoints2[index2];

                    if (matched2Flags[index2] || !p_mapPoint2)
                        continue;

                    bool mapPoint2IsBad{};
                    if (p_mapPoint2->isBad(mapPoint2IsBad) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (mapPoint2IsBad)
                        continue;

                    const cv::Mat &d2 = descriptors2.row(index2);

                    int distance{};
                    if (computeDescriptorDistance(d1, d2, distance) !=
                        ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: computeDescriptorDistance returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }

                    if (distance < bestDistance1)
                    {
                        bestDistance2 = bestDistance1;
                        bestDistance1 = distance;
                        bestIndex2    = index2;
                    }
                    else if (distance < bestDistance2)
                    {
                        bestDistance2 = distance;
                    }
                }

                if (bestDistance1 < TH_LOW)
                {
                    if (static_cast<float>(bestDistance1) <
                        nearestNeighborRatio *
                            static_cast<float>(bestDistance2))
                    {
                        vpMatches12[index1]       = mapPoints2[bestIndex2];
                        matched2Flags[bestIndex2] = true;

                        if (shouldCheckOrientation)
                        {
                            float rot = undistortedKeyPoints1[index1].angle -
                                        undistortedKeyPoints2[bestIndex2].angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = std::round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(index1);
                        }
                        nmatches++;
                    }
                }
            }

            firstFeatureIt++;
            secondFeatureIt++;
        }
        else if (firstFeatureIt->first < secondFeatureIt->first)
        {
            firstFeatureIt = featureVector1.lower_bound(secondFeatureIt->first);
        }
        else
        {
            secondFeatureIt = featureVector2.lower_bound(firstFeatureIt->first);
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
                vpMatches12[rotHist[histogramBinIndex][binEntryIndex]] =
                    static_cast<MapPoint *>(nullptr);
                nmatches--;
            }
        }
    }

    byBoW_out = nmatches;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
