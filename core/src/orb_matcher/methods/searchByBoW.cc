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
 * @file            searchByBoW.cc
 *
 * @brief           Implements ORBmatcher::searchByBoW() overloads, declared in
 *                  ORBmatcher.h.
 */

#include "dbow2/feature_vector/feature_vector.h"
#include "frame/frame.h"
#include "keyframe/keyframe.h"
#include "keyframe/objects/KeyFrameStatusEnum.h"
#include "map_point/map_point.h"
#include "map_point/objects/MapPointStatusEnum.h"
#include "orb_matcher/objects/ORBmatcherClass.h"
#include <cstdint>
#include <limits.h>
#include <opencv2/core/core.hpp>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace orb_matcher
{

ORBmatcherStatus ORBmatcher::searchByBoW(
    keyframe::KeyFrame                 *p_keyframe_in,
    frame::Frame                       &frame_inout,
    std::vector<map_point::MapPoint *> &mapPointMatches_out,
    int                                &byBoW_out)
{
    std::vector<map_point::MapPoint *> mapPointsKeyFrames{};
    if (p_keyframe_in->getMapPointMatches(mapPointsKeyFrames) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    mapPointMatches_out = std::vector<map_point::MapPoint *>(
        frame_inout.keyPointCount,
        static_cast<map_point::MapPoint *>(nullptr));

    const dbow2::feature_vector::FeatureVector &featureVectorKeyFrame =
        p_keyframe_in->featureVector;

    int nmatches = 0;

    std::vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    /*!
     * We perform the matching over ORB that belong to the same vocabulary node
     * (at a certain level)
     */
    dbow2::feature_vector::FeatureVector::const_iterator KFit =
        featureVectorKeyFrame.begin();
    dbow2::feature_vector::FeatureVector::const_iterator Fit =
        frame_inout.featureVector.begin();
    dbow2::feature_vector::FeatureVector::const_iterator keyFrameEnd =
        featureVectorKeyFrame.end();
    dbow2::feature_vector::FeatureVector::const_iterator Fend =
        frame_inout.featureVector.end();

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

                map_point::MapPoint *p_mapPoint =
                    mapPointsKeyFrames[realIndexKeyFrame];

                if (!p_mapPoint)
                    continue;

                bool mapPointIsBad{};
                if (p_mapPoint->isBad(mapPointIsBad) !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (mapPointIsBad)
                    continue;

                const cv::Mat &keyFrameDescriptor =
                    p_keyframe_in->descriptors.row(realIndexKeyFrame);

                int bestDistance1 = 256;
                int bestIndexF    = -1;
                int bestDistance2 = 256;

                int bestDistance1R = 256;
                int bestIndexFr    = -1;
                int bestDistance2R = 256;

                for (size_t iF = 0; iF < indicesFs.size(); iF++)
                {
                    if (frame_inout.leftKeyPointCount == -1)
                    {
                        const unsigned int realIndexF = indicesFs[iF];

                        if (mapPointMatches_out[realIndexF])
                            continue;

                        const cv::Mat &dF =
                            frame_inout.descriptors.row(realIndexF);

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

                        if (mapPointMatches_out[realIndexF])
                            continue;

                        const cv::Mat &dF =
                            frame_inout.descriptors.row(realIndexF);

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
                                             frame_inout.leftKeyPointCount) &&
                            distance < bestDistance1)
                        {
                            bestDistance2 = bestDistance1;
                            bestDistance1 = distance;
                            bestIndexF    = realIndexF;
                        }
                        else if (realIndexF <
                                     static_cast<unsigned int>(
                                         frame_inout.leftKeyPointCount) &&
                                 distance < bestDistance2)
                        {
                            bestDistance2 = distance;
                        }

                        if (realIndexF >= static_cast<unsigned int>(
                                              frame_inout.leftKeyPointCount) &&
                            distance < bestDistance1R)
                        {
                            bestDistance2R = bestDistance1R;
                            bestDistance1R = distance;
                            bestIndexFr    = realIndexF;
                        }
                        else if (realIndexF >=
                                     static_cast<unsigned int>(
                                         frame_inout.leftKeyPointCount) &&
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
                        mapPointMatches_out[bestIndexF] = p_mapPoint;

                        const cv::KeyPoint &keyPoint =
                            (!p_keyframe_in->p_camera2)
                                ? p_keyframe_in
                                      ->keyPointsUndistorted[realIndexKeyFrame]
                            : (realIndexKeyFrame >=
                               static_cast<unsigned int>(
                                   p_keyframe_in->leftKeyPointCount))
                                ? p_keyframe_in->keyPointsRight
                                      [realIndexKeyFrame -
                                       p_keyframe_in->leftKeyPointCount]
                                : p_keyframe_in->keyPoints[realIndexKeyFrame];

                        if (shouldCheckOrientation)
                        {
                            cv::KeyPoint &Fkp =
                                (!p_keyframe_in->p_camera2 ||
                                 frame_inout.leftKeyPointCount == -1)
                                    ? frame_inout.keyPoints[bestIndexF]
                                : (bestIndexF >= frame_inout.leftKeyPointCount)
                                    ? frame_inout.keyPointsRight
                                          [bestIndexF -
                                           frame_inout.leftKeyPointCount]
                                    : frame_inout.keyPoints[bestIndexF];

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
                            mapPointMatches_out[bestIndexFr] = p_mapPoint;

                            const cv::KeyPoint &keyPoint =
                                (!p_keyframe_in->p_camera2)
                                    ? p_keyframe_in->keyPointsUndistorted
                                          [realIndexKeyFrame]
                                : (realIndexKeyFrame >=
                                   static_cast<unsigned int>(
                                       p_keyframe_in->leftKeyPointCount))
                                    ? p_keyframe_in->keyPointsRight
                                          [realIndexKeyFrame -
                                           p_keyframe_in->leftKeyPointCount]
                                    : p_keyframe_in
                                          ->keyPoints[realIndexKeyFrame];

                            if (shouldCheckOrientation)
                            {
                                cv::KeyPoint &Fkp =
                                    (!frame_inout.p_camera2)
                                        ? frame_inout.keyPoints[bestIndexFr]
                                    : (bestIndexFr >=
                                       frame_inout.leftKeyPointCount)
                                        ? frame_inout.keyPointsRight
                                              [bestIndexFr -
                                               frame_inout.leftKeyPointCount]
                                        : frame_inout.keyPoints[bestIndexFr];

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
            Fit = frame_inout.featureVector.lower_bound(KFit->first);
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
                mapPointMatches_out[rotHist[histogramBinIndex][binEntryIndex]] =
                    static_cast<map_point::MapPoint *>(nullptr);
                nmatches--;
            }
        }
    }

    byBoW_out = nmatches;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

orb_matcher::ORBmatcherStatus orb_matcher::ORBmatcher::searchByBoW(
    keyframe::KeyFrame                 *p_keyframe1_in,
    keyframe::KeyFrame                 *p_keyframe2_in,
    std::vector<map_point::MapPoint *> &matches12_out,
    int                                &byBoW_out)
{
    const std::vector<cv::KeyPoint> &undistortedKeyPoints1 =
        p_keyframe1_in->keyPointsUndistorted;
    const dbow2::feature_vector::FeatureVector &featureVector1 =
        p_keyframe1_in->featureVector;
    std::vector<map_point::MapPoint *> mapPoints1{};
    if (p_keyframe1_in->getMapPointMatches(mapPoints1) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const cv::Mat &descriptors1 = p_keyframe1_in->descriptors;

    const std::vector<cv::KeyPoint> &undistortedKeyPoints2 =
        p_keyframe2_in->keyPointsUndistorted;
    const dbow2::feature_vector::FeatureVector &featureVector2 =
        p_keyframe2_in->featureVector;
    std::vector<map_point::MapPoint *> mapPoints2{};
    if (p_keyframe2_in->getMapPointMatches(mapPoints2) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const cv::Mat &descriptors2 = p_keyframe2_in->descriptors;

    matches12_out = std::vector<map_point::MapPoint *>(
        mapPoints1.size(),
        static_cast<map_point::MapPoint *>(nullptr));
    std::vector<bool> matched2Flags(mapPoints2.size(), false);

    std::vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);

    const float factor = 1.0f / HISTO_LENGTH;

    int nmatches = 0;

    dbow2::feature_vector::FeatureVector::const_iterator firstFeatureIt =
        featureVector1.begin();
    dbow2::feature_vector::FeatureVector::const_iterator secondFeatureIt =
        featureVector2.begin();
    dbow2::feature_vector::FeatureVector::const_iterator firstFeatureEnd =
        featureVector1.end();
    dbow2::feature_vector::FeatureVector::const_iterator secondFeatureEnd =
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
                if (p_keyframe1_in->leftKeyPointCount != -1 &&
                    index1 >= p_keyframe1_in->keyPointsUndistorted.size())
                {
                    continue;
                }

                map_point::MapPoint *p_mapPoint1 = mapPoints1[index1];
                if (!p_mapPoint1)
                    continue;
                bool mapPoint1IsBad{};
                if (p_mapPoint1->isBad(mapPoint1IsBad) !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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

                    if (p_keyframe2_in->leftKeyPointCount != -1 &&
                        index2 >= p_keyframe2_in->keyPointsUndistorted.size())
                    {
                        continue;
                    }

                    map_point::MapPoint *p_mapPoint2 = mapPoints2[index2];

                    if (matched2Flags[index2] || !p_mapPoint2)
                        continue;

                    bool mapPoint2IsBad{};
                    if (p_mapPoint2->isBad(mapPoint2IsBad) !=
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                        orb_matcher::ORBmatcherStatus::
                            ORBMATCHER_STATUS_SUCCESS)
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
                        matches12_out[index1]     = mapPoints2[bestIndex2];
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
            orb_matcher::ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
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
                matches12_out[rotHist[histogramBinIndex][binEntryIndex]] =
                    static_cast<map_point::MapPoint *>(nullptr);
                nmatches--;
            }
        }
    }

    byBoW_out = nmatches;
    return orb_matcher::ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} /* namespace orb_matcher */
} /* namespace core */
} /* namespace vs_graphs */
