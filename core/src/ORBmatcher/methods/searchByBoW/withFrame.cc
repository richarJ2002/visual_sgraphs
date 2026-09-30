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

#include <cstdint>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

ORBmatcherStatus
    ORBmatcher::searchByBoW(KeyFrame                *pKF,
                            Frame                   &F,
                            std::vector<MapPoint *> &vpMapPointMatches,
                            int                     &byBoW_out)
{
    std::vector<MapPoint *> mapPointsKeyFrames{};
    if (pKF->getMapPointMatches(mapPointsKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    vpMapPointMatches =
        std::vector<MapPoint *>(F.keyPointCount,
                                static_cast<MapPoint *>(nullptr));

    const DBoW2::FeatureVector &featureVectorKeyFrame = pKF->featureVector;

    int nmatches = 0;

    std::vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node
    // (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = featureVectorKeyFrame.begin();
    DBoW2::FeatureVector::const_iterator Fit  = F.featureVector.begin();
    DBoW2::FeatureVector::const_iterator keyFrameEnd =
        featureVectorKeyFrame.end();
    DBoW2::FeatureVector::const_iterator Fend = F.featureVector.end();

    while (KFit != keyFrameEnd && Fit != Fend)
    {
        if (KFit->first == Fit->first)
        {
            const std::vector<unsigned int> indicesKeyFrames = KFit->second;
            const std::vector<unsigned int> indicesFs        = Fit->second;

            for (size_t keyFrameFeatureListIndex = 0;
                 keyFrameFeatureListIndex < indicesKeyFrames.size();
                 keyFrameFeatureListIndex++)
            {
                const unsigned int realIndexKeyFrame =
                    indicesKeyFrames[keyFrameFeatureListIndex];

                MapPoint *p_mapPoint = mapPointsKeyFrames[realIndexKeyFrame];

                if (!p_mapPoint)
                    continue;

                bool mapPointIsBad{};
                if (p_mapPoint->isBad(mapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (mapPointIsBad)
                    continue;

                const cv::Mat &keyFrameDescriptor =
                    pKF->descriptors.row(realIndexKeyFrame);

                int bestDistance1 = 256;
                int bestIndexF    = -1;
                int bestDistance2 = 256;

                int bestDistance1R = 256;
                int bestIndexFr    = -1;
                int bestDistance2R = 256;

                for (size_t iF = 0; iF < indicesFs.size(); iF++)
                {
                    if (F.leftKeyPointCount == -1)
                    {
                        const unsigned int realIndexF = indicesFs[iF];

                        if (vpMapPointMatches[realIndexF])
                            continue;

                        const cv::Mat &dF = F.descriptors.row(realIndexF);

                        int distance{};
                        if (computeDescriptorDistance(keyFrameDescriptor,
                                                      dF,
                                                      distance) !=
                            ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: computeDescriptorDistance returned a "
                                "failure status although it cannot fail; "
                                "continuing as before.",
                                __func__);
                        }

                        if (distance < bestDistance1)
                        {
                            bestDistance2 = bestDistance1;
                            bestDistance1 = distance;
                            bestIndexF    = realIndexF;
                        }
                        else if (distance < bestDistance2)
                        {
                            bestDistance2 = distance;
                        }
                    }
                    else
                    {
                        const unsigned int realIndexF = indicesFs[iF];

                        if (vpMapPointMatches[realIndexF])
                            continue;

                        const cv::Mat &dF = F.descriptors.row(realIndexF);

                        int distance{};
                        if (computeDescriptorDistance(keyFrameDescriptor,
                                                      dF,
                                                      distance) !=
                            ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: computeDescriptorDistance returned a "
                                "failure status although it cannot fail; "
                                "continuing as before.",
                                __func__);
                        }

                        if (realIndexF < static_cast<unsigned int>(
                                             F.leftKeyPointCount) &&
                            distance < bestDistance1)
                        {
                            bestDistance2 = bestDistance1;
                            bestDistance1 = distance;
                            bestIndexF    = realIndexF;
                        }
                        else if (realIndexF < static_cast<unsigned int>(
                                                  F.leftKeyPointCount) &&
                                 distance < bestDistance2)
                        {
                            bestDistance2 = distance;
                        }

                        if (realIndexF >= static_cast<unsigned int>(
                                              F.leftKeyPointCount) &&
                            distance < bestDistance1R)
                        {
                            bestDistance2R = bestDistance1R;
                            bestDistance1R = distance;
                            bestIndexFr    = realIndexF;
                        }
                        else if (realIndexF >= static_cast<unsigned int>(
                                                   F.leftKeyPointCount) &&
                                 distance < bestDistance2R)
                        {
                            bestDistance2R = distance;
                        }
                    }
                }

                if (bestDistance1 <= TH_LOW)
                {
                    if (static_cast<float>(bestDistance1) <
                        nearestNeighborRatio *
                            static_cast<float>(bestDistance2))
                    {
                        vpMapPointMatches[bestIndexF] = p_mapPoint;

                        const cv::KeyPoint &keyPoint =
                            (!pKF->p_camera2)
                                ? pKF->keyPointsUndistorted[realIndexKeyFrame]
                            : (realIndexKeyFrame >= static_cast<unsigned int>(
                                                        pKF->leftKeyPointCount))
                                ? pKF->keyPointsRight[realIndexKeyFrame -
                                                      pKF->leftKeyPointCount]
                                : pKF->keyPoints[realIndexKeyFrame];

                        if (shouldCheckOrientation)
                        {
                            cv::KeyPoint &Fkp =
                                (!pKF->p_camera2 || F.leftKeyPointCount == -1)
                                    ? F.keyPoints[bestIndexF]
                                : (bestIndexF >= F.leftKeyPointCount)
                                    ? F.keyPointsRight[bestIndexF -
                                                       F.leftKeyPointCount]
                                    : F.keyPoints[bestIndexF];

                            float rot = keyPoint.angle - Fkp.angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = std::round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(bestIndexF);
                        }
                        nmatches++;
                    }

                    if (bestDistance1R <= TH_LOW)
                    {
                        if (static_cast<float>(bestDistance1R) <
                                nearestNeighborRatio *
                                    static_cast<float>(bestDistance2R) ||
                            true)
                        {
                            vpMapPointMatches[bestIndexFr] = p_mapPoint;

                            const cv::KeyPoint &keyPoint =
                                (!pKF->p_camera2) ? pKF->keyPointsUndistorted
                                                        [realIndexKeyFrame]
                                : (realIndexKeyFrame >=
                                   static_cast<unsigned int>(
                                       pKF->leftKeyPointCount))
                                    ? pKF->keyPointsRight
                                          [realIndexKeyFrame -
                                           pKF->leftKeyPointCount]
                                    : pKF->keyPoints[realIndexKeyFrame];

                            if (shouldCheckOrientation)
                            {
                                cv::KeyPoint &Fkp =
                                    (!F.p_camera2) ? F.keyPoints[bestIndexFr]
                                    : (bestIndexFr >= F.leftKeyPointCount)
                                        ? F.keyPointsRight[bestIndexFr -
                                                           F.leftKeyPointCount]
                                        : F.keyPoints[bestIndexFr];

                                float rot = keyPoint.angle - Fkp.angle;
                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = std::round(rot * factor);
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                assert(bin >= 0 && bin < HISTO_LENGTH);
                                rotHist[bin].push_back(bestIndexFr);
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
            KFit = featureVectorKeyFrame.lower_bound(Fit->first);
        }
        else
        {
            Fit = F.featureVector.lower_bound(KFit->first);
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
                vpMapPointMatches[rotHist[histogramBinIndex][binEntryIndex]] =
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
