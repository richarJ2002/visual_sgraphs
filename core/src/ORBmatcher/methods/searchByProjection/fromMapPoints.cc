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

ORBmatcherStatus
    ORBmatcher::searchByProjection(Frame                      &F,
                                   const vector<MapPoint *>   &vpMapPoints,
                                   int                        &byProjection_out,
                                   const float                 th,
                                   const bool                  bFarPoints,
                                   const float                 thFarPoints,
                                   const std::optional<float> &thDepth)
{
    int nmatches = 0, left = 0, right = 0;

    const bool isThresholdScaled = th != 1.0;

    for (size_t mapPointIndex = 0; mapPointIndex < vpMapPoints.size();
         mapPointIndex++)
    {
        MapPoint *p_mapPoint = vpMapPoints[mapPointIndex];
        if (!p_mapPoint->isTrackedInView && !p_mapPoint->isTrackedInRightView)
            continue;

        if (bFarPoints && p_mapPoint->trackDepth > thFarPoints)
            continue;

        bool mapPointIsBad{};
        if (p_mapPoint->isBad(mapPointIsBad) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad)
            continue;

        if (p_mapPoint->isTrackedInView)
        {
            const int &predictedLevelCount = p_mapPoint->trackScaleLevel;

            // The size of the window will depend on the viewing direction
            float r{};
            if (radiusByViewingCos(p_mapPoint->trackViewCos, r) !=
                ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: radiusByViewingCos returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (isThresholdScaled)
                r *= th;

            // Depth-guided search: a tighter window for close points helps
            // in repetitive corridors where visual ambiguity is high
            if (thDepth.has_value() && p_mapPoint->trackDepth > 0)
            {
                if (p_mapPoint->trackDepth < *thDepth)
                    r *= 0.7f;
                else
                    r *= 1.2f;
            }

            std::vector<size_t> indices{};
            if (F.getFeaturesInArea(p_mapPoint->trackProjX,
                                    p_mapPoint->trackProjY,
                                    r * F.scaleFactors[predictedLevelCount],
                                    indices,
                                    predictedLevelCount - 1,
                                    predictedLevelCount) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getFeaturesInArea returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            if (!indices.empty())
            {
                cv::Mat mapPointDescriptor{};
                if (p_mapPoint->getDescriptor(mapPointDescriptor) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getDescriptor returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }

                int bestDistance  = 256;
                int bestLevel     = -1;
                int bestDistance2 = 256;
                int bestLevel2    = -1;
                int bestIndex     = -1;

                // Get best and second matches with near keypoints
                for (vector<size_t>::const_iterator vit  = indices.begin(),
                                                    vend = indices.end();
                     vit != vend;
                     vit++)
                {
                    const size_t featureIndex = *vit;

                    if (F.mapPoints[featureIndex])
                    {
                        int observationCount{};
                        if (F.mapPoints[featureIndex]->getObservationCount(
                                observationCount) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getObservationCount returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        if (observationCount > 0)
                        {
                            continue;
                        }
                    }

                    if (F.leftKeyPointCount == -1 && F.uRight[featureIndex] > 0)
                    {
                        const float er = fabs(p_mapPoint->trackProjXR -
                                              F.uRight[featureIndex]);
                        if (er > r * F.scaleFactors[predictedLevelCount])
                            continue;
                    }

                    const cv::Mat &d = F.descriptors.row(featureIndex);

                    int distance{};
                    if (computeDescriptorDistance(mapPointDescriptor,
                                                  d,
                                                  distance) !=
                        ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: computeDescriptorDistance returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }

                    if (distance < bestDistance)
                    {
                        bestDistance2 = bestDistance;
                        bestDistance  = distance;
                        bestLevel2    = bestLevel;
                        bestLevel =
                            (F.leftKeyPointCount == -1)
                                ? F.keyPointsUndistorted[featureIndex].octave
                            : (featureIndex <
                               static_cast<size_t>(F.leftKeyPointCount))
                                ? F.keyPoints[featureIndex].octave
                                : F.keyPointsRight[featureIndex -
                                                   F.leftKeyPointCount]
                                      .octave;
                        bestIndex = featureIndex;
                    }
                    else if (distance < bestDistance2)
                    {
                        bestLevel2 =
                            (F.leftKeyPointCount == -1)
                                ? F.keyPointsUndistorted[featureIndex].octave
                            : (featureIndex <
                               static_cast<size_t>(F.leftKeyPointCount))
                                ? F.keyPoints[featureIndex].octave
                                : F.keyPointsRight[featureIndex -
                                                   F.leftKeyPointCount]
                                      .octave;
                        bestDistance2 = distance;
                    }
                }

                // Apply ratio to second match (only if best and second are in
                // the same scale level)
                if (bestDistance <= TH_HIGH)
                {
                    if (bestLevel == bestLevel2 &&
                        bestDistance > nearestNeighborRatio * bestDistance2)
                        continue;

                    if (bestLevel != bestLevel2 ||
                        bestDistance <= nearestNeighborRatio * bestDistance2)
                    {
                        F.mapPoints[bestIndex] = p_mapPoint;

                        if (F.leftKeyPointCount != -1 &&
                            F.leftToRightMatches[bestIndex] != -1)
                        { // Also match with the stereo observation at right
                          // camera
                            F.mapPoints[F.leftToRightMatches[bestIndex] +
                                        F.leftKeyPointCount] = p_mapPoint;
                            nmatches++;
                            right++;
                        }

                        nmatches++;
                        left++;
                    }
                }
            }
        }

        if (F.leftKeyPointCount != -1 && p_mapPoint->isTrackedInRightView)
        {
            const int &predictedLevelCount = p_mapPoint->trackScaleLevelR;
            if (predictedLevelCount != -1)
            {
                float r{};
                if (radiusByViewingCos(p_mapPoint->trackViewCosR, r) !=
                    ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: radiusByViewingCos returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                if (thDepth.has_value() && p_mapPoint->trackDepthR > 0)
                {
                    if (p_mapPoint->trackDepthR < *thDepth)
                        r *= 0.7f;
                    else
                        r *= 1.2f;
                }

                std::vector<size_t> indices{};
                if (F.getFeaturesInArea(p_mapPoint->trackProjXR,
                                        p_mapPoint->trackProjYR,
                                        r * F.scaleFactors[predictedLevelCount],
                                        indices,
                                        predictedLevelCount - 1,
                                        predictedLevelCount,
                                        true) !=
                    FrameStatus::FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getFeaturesInArea returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                if (indices.empty())
                    continue;

                cv::Mat mapPointDescriptor{};
                if (p_mapPoint->getDescriptor(mapPointDescriptor) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getDescriptor returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }

                int bestDistance  = 256;
                int bestLevel     = -1;
                int bestDistance2 = 256;
                int bestLevel2    = -1;
                int bestIndex     = -1;

                // Get best and second matches with near keypoints
                for (vector<size_t>::const_iterator vit  = indices.begin(),
                                                    vend = indices.end();
                     vit != vend;
                     vit++)
                {
                    const size_t featureIndex = *vit;

                    if (F.mapPoints[featureIndex + F.leftKeyPointCount])
                    {
                        int observationCount2{};
                        if (F.mapPoints[featureIndex + F.leftKeyPointCount]
                                ->getObservationCount(observationCount2) !=
                            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getObservationCount returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        if (observationCount2 > 0)
                        {
                            continue;
                        }
                    }

                    const cv::Mat &d =
                        F.descriptors.row(featureIndex + F.leftKeyPointCount);

                    int distance{};
                    if (computeDescriptorDistance(mapPointDescriptor,
                                                  d,
                                                  distance) !=
                        ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: computeDescriptorDistance returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }

                    if (distance < bestDistance)
                    {
                        bestDistance2 = bestDistance;
                        bestDistance  = distance;
                        bestLevel2    = bestLevel;
                        bestLevel     = F.keyPointsRight[featureIndex].octave;
                        bestIndex     = featureIndex;
                    }
                    else if (distance < bestDistance2)
                    {
                        bestLevel2    = F.keyPointsRight[featureIndex].octave;
                        bestDistance2 = distance;
                    }
                }

                // Apply ratio to second match (only if best and second are in
                // the same scale level)
                if (bestDistance <= TH_HIGH)
                {
                    if (bestLevel == bestLevel2 &&
                        bestDistance > nearestNeighborRatio * bestDistance2)
                        continue;

                    if (F.leftKeyPointCount != -1 &&
                        F.rightToLeftMatches[bestIndex] != -1)
                    { // Also match with the stereo observation at right camera
                        F.mapPoints[F.rightToLeftMatches[bestIndex]] =
                            p_mapPoint;
                        nmatches++;
                        left++;
                    }

                    F.mapPoints[bestIndex + F.leftKeyPointCount] = p_mapPoint;
                    nmatches++;
                    right++;
                }
            }
        }
    }
    byProjection_out = nmatches;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
