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
 * @file            fromMapPoints.cc
 *
 * @brief           Implements ORBmatcher::searchByProjection() (fromMapPoints),
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

ORBmatcherStatus ORBmatcher::searchByProjection(
    Frame                         &frame_inout,
    const std::vector<MapPoint *> &mapPoints_in,
    int                           &byProjection_out,
    const float                    threshold_in,
    const bool                     farPoints_in,
    const float                    farPointsThreshold_in,
    const std::optional<float>    &depthThreshold_in)
{
    int nmatches = 0, left = 0, right = 0;

    const bool isThresholdScaled = threshold_in != 1.0;

    for (size_t mapPointIndex = 0; mapPointIndex < mapPoints_in.size();
         mapPointIndex++)
    {
        MapPoint *p_mapPoint = mapPoints_in[mapPointIndex];
        if (!p_mapPoint->isTrackedInView && !p_mapPoint->isTrackedInRightView)
            continue;

        if (farPoints_in && p_mapPoint->trackDepth > farPointsThreshold_in)
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
                r *= threshold_in;

            // Depth-guided search: a tighter window for close points helps
            // in repetitive corridors where visual ambiguity is high
            if (depthThreshold_in.has_value() && p_mapPoint->trackDepth > 0)
            {
                if (p_mapPoint->trackDepth < *depthThreshold_in)
                    r *= 0.7f;
                else
                    r *= 1.2f;
            }

            std::vector<size_t> indices{};
            if (frame_inout.getFeaturesInArea(
                    p_mapPoint->trackProjX,
                    p_mapPoint->trackProjY,
                    r * frame_inout.scaleFactors[predictedLevelCount],
                    indices,
                    predictedLevelCount - 1,
                    predictedLevelCount) != FrameStatus::FRAME_STATUS_SUCCESS)
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
                for (std::vector<size_t>::const_iterator vit  = indices.begin(),
                                                         vend = indices.end();
                     vit != vend;
                     vit++)
                {
                    const size_t featureIndex = *vit;

                    if (frame_inout.mapPoints[featureIndex])
                    {
                        int observationCount{};
                        if (frame_inout.mapPoints[featureIndex]
                                ->getObservationCount(observationCount) !=
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

                    if (frame_inout.leftKeyPointCount == -1 &&
                        frame_inout.uRight[featureIndex] > 0)
                    {
                        const float er =
                            std::fabs(p_mapPoint->trackProjXR -
                                      frame_inout.uRight[featureIndex]);
                        if (er >
                            r * frame_inout.scaleFactors[predictedLevelCount])
                            continue;
                    }

                    const cv::Mat &d =
                        frame_inout.descriptors.row(featureIndex);

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
                            (frame_inout.leftKeyPointCount == -1)
                                ? frame_inout.keyPointsUndistorted[featureIndex]
                                      .octave
                            : (featureIndex <
                               static_cast<size_t>(
                                   frame_inout.leftKeyPointCount))
                                ? frame_inout.keyPoints[featureIndex].octave
                                : frame_inout
                                      .keyPointsRight[featureIndex -
                                                      frame_inout
                                                          .leftKeyPointCount]
                                      .octave;
                        bestIndex = featureIndex;
                    }
                    else if (distance < bestDistance2)
                    {
                        bestLevel2 =
                            (frame_inout.leftKeyPointCount == -1)
                                ? frame_inout.keyPointsUndistorted[featureIndex]
                                      .octave
                            : (featureIndex <
                               static_cast<size_t>(
                                   frame_inout.leftKeyPointCount))
                                ? frame_inout.keyPoints[featureIndex].octave
                                : frame_inout
                                      .keyPointsRight[featureIndex -
                                                      frame_inout
                                                          .leftKeyPointCount]
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
                        frame_inout.mapPoints[bestIndex] = p_mapPoint;

                        if (frame_inout.leftKeyPointCount != -1 &&
                            frame_inout.leftToRightMatches[bestIndex] != -1)
                        { // Also match with the stereo observation at right
                          // camera
                            frame_inout.mapPoints
                                [frame_inout.leftToRightMatches[bestIndex] +
                                 frame_inout.leftKeyPointCount] = p_mapPoint;
                            nmatches++;
                            right++;
                        }

                        nmatches++;
                        left++;
                    }
                }
            }
        }

        if (frame_inout.leftKeyPointCount != -1 &&
            p_mapPoint->isTrackedInRightView)
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

                if (depthThreshold_in.has_value() &&
                    p_mapPoint->trackDepthR > 0)
                {
                    if (p_mapPoint->trackDepthR < *depthThreshold_in)
                        r *= 0.7f;
                    else
                        r *= 1.2f;
                }

                std::vector<size_t> indices{};
                if (frame_inout.getFeaturesInArea(
                        p_mapPoint->trackProjXR,
                        p_mapPoint->trackProjYR,
                        r * frame_inout.scaleFactors[predictedLevelCount],
                        indices,
                        predictedLevelCount - 1,
                        predictedLevelCount,
                        true) != FrameStatus::FRAME_STATUS_SUCCESS)
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
                for (std::vector<size_t>::const_iterator vit  = indices.begin(),
                                                         vend = indices.end();
                     vit != vend;
                     vit++)
                {
                    const size_t featureIndex = *vit;

                    if (frame_inout.mapPoints[featureIndex +
                                              frame_inout.leftKeyPointCount])
                    {
                        int observationCount2{};
                        if (frame_inout
                                .mapPoints[featureIndex +
                                           frame_inout.leftKeyPointCount]
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

                    const cv::Mat &d = frame_inout.descriptors.row(
                        featureIndex + frame_inout.leftKeyPointCount);

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
                            frame_inout.keyPointsRight[featureIndex].octave;
                        bestIndex = featureIndex;
                    }
                    else if (distance < bestDistance2)
                    {
                        bestLevel2 =
                            frame_inout.keyPointsRight[featureIndex].octave;
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

                    if (frame_inout.leftKeyPointCount != -1 &&
                        frame_inout.rightToLeftMatches[bestIndex] != -1)
                    { // Also match with the stereo observation at right camera
                        frame_inout.mapPoints
                            [frame_inout.rightToLeftMatches[bestIndex]] =
                            p_mapPoint;
                        nmatches++;
                        left++;
                    }

                    frame_inout
                        .mapPoints[bestIndex + frame_inout.leftKeyPointCount] =
                        p_mapPoint;
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
