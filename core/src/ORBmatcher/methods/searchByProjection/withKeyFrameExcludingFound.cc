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
 * @file            withKeyFrameExcludingFound.cc
 *
 * @brief           Implements ORBmatcher::searchByProjection()
 *                  (withKeyFrameExcludingFound), declared in ORBmatcher.h.
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
    ORBmatcher::searchByProjection(Frame    &currentFrame_inout,
                                   KeyFrame *p_keyframe_in,
                                   const std::set<MapPoint *> &alreadyFound_in,
                                   const float                 threshold_in,
                                   const int                   orbDistance_in,
                                   int                        &byProjection_out)
{
    int nmatches = 0;

    Sophus::SE3f poseWorldToCamera{};
    if (currentFrame_inout.getPose(poseWorldToCamera) !=
        FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3f cameraCenter_World =
        poseWorldToCamera.inverse().translation();

    // Rotation Histogram (to check rotation consistency)
    std::vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    std::vector<MapPoint *> mapPoints{};
    if (p_keyframe_in->getMapPointMatches(mapPoints) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    for (size_t histogramBinIndex = 0, iend = mapPoints.size();
         histogramBinIndex < iend;
         histogramBinIndex++)
    {
        MapPoint *p_mapPoint = mapPoints[histogramBinIndex];

        if (p_mapPoint)
        {
            bool mapPointIsBad{};
            if (p_mapPoint->isBad(mapPointIsBad) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!mapPointIsBad && !alreadyFound_in.count(p_mapPoint))
            {
                // Project
                Eigen::Vector3f x3Dw{};
                if (p_mapPoint->getWorldPos(x3Dw) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWorldPos returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3f x3Dc = poseWorldToCamera * x3Dw;

                const Eigen::Vector2f uv =
                    currentFrame_inout.p_camera->project(x3Dc);

                if (uv(0) < currentFrame_inout.gridMinX ||
                    uv(0) > currentFrame_inout.gridMaxX)
                    continue;
                if (uv(1) < currentFrame_inout.gridMinY ||
                    uv(1) > currentFrame_inout.gridMaxY)
                    continue;

                // Compute predicted scale level
                Eigen::Vector3f PO         = x3Dw - cameraCenter_World;
                float           distance3d = PO.norm();

                float maximumDistance{};
                if (p_mapPoint->getMaxDistanceInvariance(maximumDistance) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMaxDistanceInvariance returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                float minimumDistance{};
                if (p_mapPoint->getMinDistanceInvariance(minimumDistance) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMinDistanceInvariance returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }

                // Depth must be inside the scale pyramid of the image
                if (distance3d < minimumDistance ||
                    distance3d > maximumDistance)
                    continue;

                int predictedLevelCount{};
                if (p_mapPoint->predictScale(distance3d,
                                             &currentFrame_inout,
                                             predictedLevelCount) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: predictScale returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }

                // Search in a window
                const float radius =
                    threshold_in *
                    currentFrame_inout.scaleFactors[predictedLevelCount];

                std::vector<size_t> indices2{};
                if (currentFrame_inout.getFeaturesInArea(
                        uv(0),
                        uv(1),
                        radius,
                        indices2,
                        predictedLevelCount - 1,
                        predictedLevelCount + 1) !=
                    FrameStatus::FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getFeaturesInArea returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                if (indices2.empty())
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

                int bestDistance = 256;
                int bestIndex2   = -1;

                for (std::vector<size_t>::const_iterator vit = indices2.begin();
                     vit != indices2.end();
                     vit++)
                {
                    const size_t i2 = *vit;
                    if (currentFrame_inout.mapPoints[i2])
                        continue;

                    const cv::Mat &d = currentFrame_inout.descriptors.row(i2);

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
                        bestDistance = distance;
                        bestIndex2   = i2;
                    }
                }

                if (bestDistance <= orbDistance_in)
                {
                    currentFrame_inout.mapPoints[bestIndex2] = p_mapPoint;
                    nmatches++;

                    if (shouldCheckOrientation)
                    {
                        float rot =
                            p_keyframe_in
                                ->keyPointsUndistorted[histogramBinIndex]
                                .angle -
                            currentFrame_inout.keyPointsUndistorted[bestIndex2]
                                .angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = std::round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIndex2);
                    }
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
            if (histogramBinIndex != ind1 && histogramBinIndex != ind2 &&
                histogramBinIndex != ind3)
            {
                for (size_t binEntryIndex = 0,
                            jend          = rotHist[histogramBinIndex].size();
                     binEntryIndex < jend;
                     binEntryIndex++)
                {
                    currentFrame_inout
                        .mapPoints[rotHist[histogramBinIndex][binEntryIndex]] =
                        nullptr;
                    nmatches--;
                }
            }
        }
    }

    byProjection_out = nmatches;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
