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

ORBmatcherStatus ORBmatcher::searchByProjection(Frame       &CurrentFrame,
                                                const Frame &LastFrame,
                                                const float  th,
                                                const bool   bMono,
                                                int         &byProjection_out)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    Sophus::SE3f Tcw{};
    if (CurrentFrame.getPose(Tcw) != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3f twc = Tcw.inverse().translation();

    Sophus::SE3f Tlw{};
    if (LastFrame.getPose(Tlw) != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3f tlc = Tlw * twc;

    const bool isMovingForward  = tlc(2) > CurrentFrame.mb && !bMono;
    const bool isMovingBackward = -tlc(2) > CurrentFrame.mb && !bMono;

    for (int histogramBinIndex = 0; histogramBinIndex < LastFrame.keyPointCount;
         histogramBinIndex++)
    {
        MapPoint *p_mapPoint = LastFrame.mapPoints[histogramBinIndex];
        if (p_mapPoint)
        {
            if (!LastFrame.outlierFlags[histogramBinIndex])
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
                Eigen::Vector3f x3Dc = Tcw * x3Dw;

                const float invzc = 1.0 / x3Dc(2);

                if (invzc < 0)
                    continue;

                Eigen::Vector2f uv = CurrentFrame.p_camera->project(x3Dc);

                if (uv(0) < CurrentFrame.gridMinX ||
                    uv(0) > CurrentFrame.gridMaxX)
                    continue;
                if (uv(1) < CurrentFrame.gridMinY ||
                    uv(1) > CurrentFrame.gridMaxY)
                    continue;

                int lastOctaveCount =
                    (LastFrame.leftKeyPointCount == -1 ||
                     histogramBinIndex < LastFrame.leftKeyPointCount)
                        ? LastFrame.keyPoints[histogramBinIndex].octave
                        : LastFrame
                              .keyPointsRight[histogramBinIndex -
                                              LastFrame.leftKeyPointCount]
                              .octave;

                // Search in a window. Size depends on scale
                float radius = th * CurrentFrame.scaleFactors[lastOctaveCount];

                vector<size_t> indices2;

                if (isMovingForward)
                {
                    std::vector<size_t> CurrentFrameFeaturesInArea{};
                    if (CurrentFrame.getFeaturesInArea(
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
                    if (CurrentFrame.getFeaturesInArea(
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
                    if (CurrentFrame.getFeaturesInArea(
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

                for (vector<size_t>::const_iterator vit  = indices2.begin(),
                                                    vend = indices2.end();
                     vit != vend;
                     vit++)
                {
                    const size_t i2 = *vit;

                    if (CurrentFrame.mapPoints[i2])
                    {
                        int observationCount{};
                        if (CurrentFrame.mapPoints[i2]->getObservationCount(
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

                    if (CurrentFrame.leftKeyPointCount == -1 &&
                        CurrentFrame.uRight[i2] > 0)
                    {
                        const float ur = uv(0) - CurrentFrame.mbf * invzc;
                        const float er = fabs(ur - CurrentFrame.uRight[i2]);
                        if (er > radius)
                            continue;
                    }

                    const cv::Mat &d = CurrentFrame.descriptors.row(i2);

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
                    CurrentFrame.mapPoints[bestIndex2] = p_mapPoint;
                    nmatches++;

                    if (shouldCheckOrientation)
                    {
                        cv::KeyPoint keyPointLf =
                            (LastFrame.leftKeyPointCount == -1)
                                ? LastFrame
                                      .keyPointsUndistorted[histogramBinIndex]
                            : (histogramBinIndex < LastFrame.leftKeyPointCount)
                                ? LastFrame.keyPoints[histogramBinIndex]
                                : LastFrame.keyPointsRight
                                      [histogramBinIndex -
                                       LastFrame.leftKeyPointCount];

                        cv::KeyPoint keyPointCf =
                            (CurrentFrame.leftKeyPointCount == -1)
                                ? CurrentFrame.keyPointsUndistorted[bestIndex2]
                            : (bestIndex2 < CurrentFrame.leftKeyPointCount)
                                ? CurrentFrame.keyPoints[bestIndex2]
                                : CurrentFrame.keyPointsRight
                                      [bestIndex2 -
                                       CurrentFrame.leftKeyPointCount];
                        float rot = keyPointLf.angle - keyPointCf.angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIndex2);
                    }
                }
                if (CurrentFrame.leftKeyPointCount != -1)
                {
                    Sophus::SE3f CurrentFrameRelativePoseTrl{};
                    if (CurrentFrame.getRelativePoseTrl(
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
                    Eigen::Vector2f uv   = CurrentFrame.p_camera->project(x3Dr);

                    int lastOctaveCount =
                        (LastFrame.leftKeyPointCount == -1 ||
                         histogramBinIndex < LastFrame.leftKeyPointCount)
                            ? LastFrame.keyPoints[histogramBinIndex].octave
                            : LastFrame
                                  .keyPointsRight[histogramBinIndex -
                                                  LastFrame.leftKeyPointCount]
                                  .octave;

                    // Search in a window. Size depends on scale
                    float radius =
                        th * CurrentFrame.scaleFactors[lastOctaveCount];

                    vector<size_t> indices2;

                    if (isMovingForward)
                    {
                        std::vector<size_t> CurrentFrameFeaturesInArea4{};
                        if (CurrentFrame.getFeaturesInArea(
                                uv(0),
                                uv(1),
                                radius,
                                CurrentFrameFeaturesInArea4,
                                lastOctaveCount,
                                -1,
                                true) != FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getFeaturesInArea returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        indices2 = CurrentFrameFeaturesInArea4;
                    }
                    else if (isMovingBackward)
                    {
                        std::vector<size_t> CurrentFrameFeaturesInArea5{};
                        if (CurrentFrame.getFeaturesInArea(
                                uv(0),
                                uv(1),
                                radius,
                                CurrentFrameFeaturesInArea5,
                                0,
                                lastOctaveCount,
                                true) != FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getFeaturesInArea returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        indices2 = CurrentFrameFeaturesInArea5;
                    }
                    else
                    {
                        std::vector<size_t> CurrentFrameFeaturesInArea6{};
                        if (CurrentFrame.getFeaturesInArea(
                                uv(0),
                                uv(1),
                                radius,
                                CurrentFrameFeaturesInArea6,
                                lastOctaveCount - 1,
                                lastOctaveCount + 1,
                                true) != FrameStatus::FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getFeaturesInArea returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        indices2 = CurrentFrameFeaturesInArea6;
                    }

                    cv::Mat mapPointDescriptor{};
                    if (p_mapPoint->getDescriptor(mapPointDescriptor) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getDescriptor returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    int bestDistance = 256;
                    int bestIndex2   = -1;

                    for (vector<size_t>::const_iterator vit  = indices2.begin(),
                                                        vend = indices2.end();
                         vit != vend;
                         vit++)
                    {
                        const size_t i2 = *vit;
                        if (CurrentFrame
                                .mapPoints[i2 + CurrentFrame.leftKeyPointCount])
                        {
                            int observationCount2{};
                            if (CurrentFrame
                                    .mapPoints[i2 +
                                               CurrentFrame.leftKeyPointCount]
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

                        const cv::Mat &d = CurrentFrame.descriptors.row(
                            i2 + CurrentFrame.leftKeyPointCount);

                        int distance{};
                        if (computeDescriptorDistance(mapPointDescriptor,
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

                        if (distance < bestDistance)
                        {
                            bestDistance = distance;
                            bestIndex2   = i2;
                        }
                    }

                    if (bestDistance <= TH_HIGH)
                    {
                        CurrentFrame.mapPoints[bestIndex2 +
                                               CurrentFrame.leftKeyPointCount] =
                            p_mapPoint;
                        nmatches++;
                        if (shouldCheckOrientation)
                        {
                            cv::KeyPoint keyPointLf =
                                (LastFrame.leftKeyPointCount == -1)
                                    ? LastFrame.keyPointsUndistorted
                                          [histogramBinIndex]
                                : (histogramBinIndex <
                                   LastFrame.leftKeyPointCount)
                                    ? LastFrame.keyPoints[histogramBinIndex]
                                    : LastFrame.keyPointsRight
                                          [histogramBinIndex -
                                           LastFrame.leftKeyPointCount];

                            cv::KeyPoint keyPointCf =
                                CurrentFrame.keyPointsRight[bestIndex2];

                            float rot = keyPointLf.angle - keyPointCf.angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].push_back(
                                bestIndex2 + CurrentFrame.leftKeyPointCount);
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
                    CurrentFrame
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
