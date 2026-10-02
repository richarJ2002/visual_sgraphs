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
 * @file            fromLastFrame.cc
 *
 * @brief           Implements ORBmatcher::searchByProjection() (fromLastFrame),
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

ORBmatcherStatus ORBmatcher::searchByProjection(Frame       &currentFrame_inout,
                                                const Frame &lastFrame_in,
                                                const float  threshold_in,
                                                const bool   mono_in,
                                                int         &byProjection_out)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    std::vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    Sophus::SE3f poseWorldToCamera{};
    if (currentFrame_inout.getPose(poseWorldToCamera) !=
        FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3f translationCameraToWorld =
        poseWorldToCamera.inverse().translation();

    Sophus::SE3f Tlw{};
    if (lastFrame_in.getPose(Tlw) != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3f tlc = Tlw * translationCameraToWorld;

    const bool isMovingForward  = tlc(2) > currentFrame_inout.mb && !mono_in;
    const bool isMovingBackward = -tlc(2) > currentFrame_inout.mb && !mono_in;

    for (int histogramBinIndex = 0;
         histogramBinIndex < lastFrame_in.keyPointCount;
         histogramBinIndex++)
    {
        MapPoint *p_mapPoint = lastFrame_in.mapPoints[histogramBinIndex];
        if (p_mapPoint)
        {
            if (!lastFrame_in.outlierFlags[histogramBinIndex])
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

                const float invzc = 1.0 / x3Dc(2);

                if (invzc < 0)
                    continue;

                Eigen::Vector2f uv = currentFrame_inout.p_camera->project(x3Dc);

                if (uv(0) < currentFrame_inout.gridMinX ||
                    uv(0) > currentFrame_inout.gridMaxX)
                    continue;
                if (uv(1) < currentFrame_inout.gridMinY ||
                    uv(1) > currentFrame_inout.gridMaxY)
                    continue;

                int lastOctaveCount =
                    (lastFrame_in.leftKeyPointCount == -1 ||
                     histogramBinIndex < lastFrame_in.leftKeyPointCount)
                        ? lastFrame_in.keyPoints[histogramBinIndex].octave
                        : lastFrame_in
                              .keyPointsRight[histogramBinIndex -
                                              lastFrame_in.leftKeyPointCount]
                              .octave;

                // Search in a window. Size depends on scale
                float radius = threshold_in *
                               currentFrame_inout.scaleFactors[lastOctaveCount];

                std::vector<size_t> indices2;

                if (isMovingForward)
                {
                    std::vector<size_t> CurrentFrameFeaturesInArea{};
                    if (currentFrame_inout.getFeaturesInArea(
                            uv(0),
                            uv(1),
                            radius,
                            CurrentFrameFeaturesInArea,
                            lastOctaveCount) !=
                        FrameStatus::FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getFeaturesInArea returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    indices2 = CurrentFrameFeaturesInArea;
                }
                else if (isMovingBackward)
                {
                    std::vector<size_t> CurrentFrameFeaturesInArea2{};
                    if (currentFrame_inout.getFeaturesInArea(
                            uv(0),
                            uv(1),
                            radius,
                            CurrentFrameFeaturesInArea2,
                            0,
                            lastOctaveCount) !=
                        FrameStatus::FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getFeaturesInArea returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    indices2 = CurrentFrameFeaturesInArea2;
                }
                else
                {
                    std::vector<size_t> CurrentFrameFeaturesInArea3{};
                    if (currentFrame_inout.getFeaturesInArea(
                            uv(0),
                            uv(1),
                            radius,
                            CurrentFrameFeaturesInArea3,
                            lastOctaveCount - 1,
                            lastOctaveCount + 1) !=
                        FrameStatus::FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getFeaturesInArea returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    indices2 = CurrentFrameFeaturesInArea3;
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

                for (std::vector<size_t>::const_iterator vit = indices2.begin(),
                                                         vend = indices2.end();
                     vit != vend;
                     vit++)
                {
                    const size_t i2 = *vit;

                    if (currentFrame_inout.mapPoints[i2])
                    {
                        int observationCount{};
                        if (currentFrame_inout.mapPoints[i2]
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

                    if (currentFrame_inout.leftKeyPointCount == -1 &&
                        currentFrame_inout.uRight[i2] > 0)
                    {
                        const float ur = uv(0) - currentFrame_inout.mbf * invzc;
                        const float er =
                            std::fabs(ur - currentFrame_inout.uRight[i2]);
                        if (er > radius)
                            continue;
                    }

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

                if (bestDistance <= TH_HIGH)
                {
                    currentFrame_inout.mapPoints[bestIndex2] = p_mapPoint;
                    nmatches++;

                    if (shouldCheckOrientation)
                    {
                        cv::KeyPoint keyPointLf =
                            (lastFrame_in.leftKeyPointCount == -1)
                                ? lastFrame_in
                                      .keyPointsUndistorted[histogramBinIndex]
                            : (histogramBinIndex <
                               lastFrame_in.leftKeyPointCount)
                                ? lastFrame_in.keyPoints[histogramBinIndex]
                                : lastFrame_in.keyPointsRight
                                      [histogramBinIndex -
                                       lastFrame_in.leftKeyPointCount];

                        cv::KeyPoint keyPointCf =
                            (currentFrame_inout.leftKeyPointCount == -1)
                                ? currentFrame_inout
                                      .keyPointsUndistorted[bestIndex2]
                            : (bestIndex2 <
                               currentFrame_inout.leftKeyPointCount)
                                ? currentFrame_inout.keyPoints[bestIndex2]
                                : currentFrame_inout.keyPointsRight
                                      [bestIndex2 -
                                       currentFrame_inout.leftKeyPointCount];
                        float rot = keyPointLf.angle - keyPointCf.angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = std::round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIndex2);
                    }
                }
                if (currentFrame_inout.leftKeyPointCount != -1)
                {
                    Sophus::SE3f CurrentFrameRelativePoseTrl{};
                    if (currentFrame_inout.getRelativePoseTrl(
                            CurrentFrameRelativePoseTrl) !=
                        FrameStatus::FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getRelativePoseTrl returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    Eigen::Vector3f x3Dr = CurrentFrameRelativePoseTrl * x3Dc;
                    Eigen::Vector2f uvRight =
                        currentFrame_inout.p_camera->project(x3Dr);

                    int lastOctaveCountRight =
                        (lastFrame_in.leftKeyPointCount == -1 ||
                         histogramBinIndex < lastFrame_in.leftKeyPointCount)
                            ? lastFrame_in.keyPoints[histogramBinIndex].octave
                            : lastFrame_in
                                  .keyPointsRight[histogramBinIndex -
                                                  lastFrame_in
                                                      .leftKeyPointCount]
                                  .octave;

                    // Search in a window. Size depends on scale
                    float radiusRight =
                        threshold_in *
                        currentFrame_inout.scaleFactors[lastOctaveCountRight];

                    std::vector<size_t> indices2Right;

                    if (isMovingForward)
                    {
                        std::vector<size_t> CurrentFrameFeaturesInArea4{};
                        if (currentFrame_inout.getFeaturesInArea(
                                uvRight(0),
                                uvRight(1),
                                radiusRight,
                                CurrentFrameFeaturesInArea4,
                                lastOctaveCountRight,
                                -1,
                                true) != FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getFeaturesInArea returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        indices2Right = CurrentFrameFeaturesInArea4;
                    }
                    else if (isMovingBackward)
                    {
                        std::vector<size_t> CurrentFrameFeaturesInArea5{};
                        if (currentFrame_inout.getFeaturesInArea(
                                uvRight(0),
                                uvRight(1),
                                radiusRight,
                                CurrentFrameFeaturesInArea5,
                                0,
                                lastOctaveCountRight,
                                true) != FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getFeaturesInArea returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        indices2Right = CurrentFrameFeaturesInArea5;
                    }
                    else
                    {
                        std::vector<size_t> CurrentFrameFeaturesInArea6{};
                        if (currentFrame_inout.getFeaturesInArea(
                                uvRight(0),
                                uvRight(1),
                                radiusRight,
                                CurrentFrameFeaturesInArea6,
                                lastOctaveCountRight - 1,
                                lastOctaveCountRight + 1,
                                true) != FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getFeaturesInArea returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        indices2Right = CurrentFrameFeaturesInArea6;
                    }

                    cv::Mat mapPointDescriptorRight{};
                    if (p_mapPoint->getDescriptor(mapPointDescriptorRight) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getDescriptor returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    int bestDistanceRight = 256;
                    int bestIndex2Right   = -1;

                    for (std::vector<size_t>::const_iterator
                             vit  = indices2Right.begin(),
                             vend = indices2Right.end();
                         vit != vend;
                         vit++)
                    {
                        const size_t i2 = *vit;
                        if (currentFrame_inout.mapPoints
                                [i2 + currentFrame_inout.leftKeyPointCount])
                        {
                            int observationCount2{};
                            if (currentFrame_inout
                                    .mapPoints[i2 + currentFrame_inout
                                                        .leftKeyPointCount]
                                    ->getObservationCount(observationCount2) !=
                                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: getObservationCount returned a "
                                    "failure status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                            if (observationCount2 > 0)
                            {
                                continue;
                            }
                        }

                        const cv::Mat &d = currentFrame_inout.descriptors.row(
                            i2 + currentFrame_inout.leftKeyPointCount);

                        int distance{};
                        if (computeDescriptorDistance(mapPointDescriptorRight,
                                                      d,
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

                        if (distance < bestDistanceRight)
                        {
                            bestDistanceRight = distance;
                            bestIndex2Right   = i2;
                        }
                    }

                    if (bestDistanceRight <= TH_HIGH)
                    {
                        currentFrame_inout
                            .mapPoints[bestIndex2Right +
                                       currentFrame_inout.leftKeyPointCount] =
                            p_mapPoint;
                        nmatches++;
                        if (shouldCheckOrientation)
                        {
                            cv::KeyPoint keyPointLf =
                                (lastFrame_in.leftKeyPointCount == -1)
                                    ? lastFrame_in.keyPointsUndistorted
                                          [histogramBinIndex]
                                : (histogramBinIndex <
                                   lastFrame_in.leftKeyPointCount)
                                    ? lastFrame_in.keyPoints[histogramBinIndex]
                                    : lastFrame_in.keyPointsRight
                                          [histogramBinIndex -
                                           lastFrame_in.leftKeyPointCount];

                            cv::KeyPoint keyPointCf =
                                currentFrame_inout
                                    .keyPointsRight[bestIndex2Right];

                            float rot = keyPointLf.angle - keyPointCf.angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = std::round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(
                                bestIndex2Right +
                                currentFrame_inout.leftKeyPointCount);
                        }
                    }
                }
            }
        }
    }

    // Apply rotation consistency
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
                        static_cast<MapPoint *>(nullptr);
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
