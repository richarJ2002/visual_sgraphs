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
 * @file            searchByProjection.cc
 *
 * @brief           Implements ORBmatcher::searchByProjection() overloads,
 *                  declared in ORBmatcher.h.
 */

#include "frame/frame.h"
#include "frame/objects/FrameStatusEnum.h"
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

ORBmatcherStatus
    ORBmatcher::searchByProjection(frame::Frame       &currentFrame_inout,
                                   const frame::Frame &lastFrame_in,
                                   const float         threshold_in,
                                   const bool          mono_in,
                                   int                &byProjection_out)
{
    int nmatches = 0;

    /* Rotation Histogram (to check rotation consistency) */
    std::vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    Sophus::SE3f cameraPose_worldToCamera{};
    if (currentFrame_inout.getPose(cameraPose_worldToCamera) !=
        frame::FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3f cameraTranslation_cameraToWorld =
        cameraPose_worldToCamera.inverse().translation();

    Sophus::SE3f lastFramePose_worldToCamera{};
    if (lastFrame_in.getPose(lastFramePose_worldToCamera) !=
        frame::FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const Eigen::Vector3f tlc =
        lastFramePose_worldToCamera * cameraTranslation_cameraToWorld;

    const bool isMovingForward  = tlc(2) > currentFrame_inout.mb && !mono_in;
    const bool isMovingBackward = -tlc(2) > currentFrame_inout.mb && !mono_in;

    for (int histogramBinIndex = 0;
         histogramBinIndex < lastFrame_in.keyPointCount;
         histogramBinIndex++)
    {
        map_point::MapPoint *p_mapPoint =
            lastFrame_in.mapPoints[histogramBinIndex];
        if (p_mapPoint)
        {
            if (!lastFrame_in.outlierFlags[histogramBinIndex])
            {
                /* Project */
                Eigen::Vector3f x3Dw{};
                if (p_mapPoint->getWorldPos(x3Dw) !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWorldPos returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3f x3Dc = cameraPose_worldToCamera * x3Dw;

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

                /* Search in a window. Size depends on scale */
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
                        frame::FrameStatus::FRAME_STATUS_SUCCESS)
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
                        frame::FrameStatus::FRAME_STATUS_SUCCESS)
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
                        frame::FrameStatus::FRAME_STATUS_SUCCESS)
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
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                        frame::FrameStatus::FRAME_STATUS_SUCCESS)
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

                    /* Search in a window. Size depends on scale */
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
                                true) !=
                            frame::FrameStatus::FRAME_STATUS_SUCCESS)
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
                                true) !=
                            frame::FrameStatus::FRAME_STATUS_SUCCESS)
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
                                true) !=
                            frame::FrameStatus::FRAME_STATUS_SUCCESS)
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
                        map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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
                                map_point::MapPointStatus::
                                    MAP_POINT_STATUS_SUCCESS)
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

    /* Apply rotation consistency */
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
                        static_cast<map_point::MapPoint *>(nullptr);
                    nmatches--;
                }
            }
        }
    }

    byProjection_out = nmatches;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

ORBmatcherStatus ORBmatcher::searchByProjection(
    frame::Frame                             &frame_inout,
    const std::vector<map_point::MapPoint *> &mapPoints_in,
    int                                      &byProjection_out,
    const float                               threshold_in,
    const bool                                farPoints_in,
    const float                               farPointsThreshold_in,
    const std::optional<float>               &depthThreshold_in)
{
    int nmatches = 0, left = 0, right = 0;

    const bool isThresholdScaled = threshold_in != 1.0;

    for (size_t mapPointIndex = 0; mapPointIndex < mapPoints_in.size();
         mapPointIndex++)
    {
        map_point::MapPoint *p_mapPoint = mapPoints_in[mapPointIndex];
        if (!p_mapPoint->isTrackedInView && !p_mapPoint->isTrackedInRightView)
            continue;

        if (farPoints_in && p_mapPoint->trackDepth > farPointsThreshold_in)
            continue;

        bool mapPointIsBad{};
        if (p_mapPoint->isBad(mapPointIsBad) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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

            /* The size of the window will depend on the viewing direction */
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

            /*!
             * Depth-guided search: a tighter window for close points helps
             * in repetitive corridors where visual ambiguity is high
             */
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
                    predictedLevelCount) !=
                frame::FrameStatus::FRAME_STATUS_SUCCESS)
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
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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

                /* Get best and second matches with near keypoints */
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
                            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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

                /*!
                 * Apply ratio to second match (only if best and second are in
                 * the same scale level)
                 */
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
                        {
                            /*!
                             * Also match with the stereo observation at right
                             * camera
                             */
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
                        true) != frame::FrameStatus::FRAME_STATUS_SUCCESS)
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
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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

                /* Get best and second matches with near keypoints */
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
                            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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

                /*!
                 * Apply ratio to second match (only if best and second are in
                 * the same scale level)
                 */
                if (bestDistance <= TH_HIGH)
                {
                    if (bestLevel == bestLevel2 &&
                        bestDistance > nearestNeighborRatio * bestDistance2)
                        continue;

                    if (frame_inout.leftKeyPointCount != -1 &&
                        frame_inout.rightToLeftMatches[bestIndex] != -1)
                    {
                        /*!
                         * Also match with the stereo observation at right
                         * camera
                         */
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

ORBmatcherStatus ORBmatcher::searchByProjection(
    keyframe::KeyFrame                       *p_keyframe_in,
    Sophus::Sim3<float>                      &cameraSimilarity_worldToCamera_in,
    const std::vector<map_point::MapPoint *> &points_in,
    std::vector<map_point::MapPoint *>       &matched_inout,
    int                                       threshold_in,
    int                                      &byProjection_out,
    float                                     hammingRatio_in)
{
    Sophus::SE3f cameraPose_worldToCamera =
        Sophus::SE3f(cameraSimilarity_worldToCamera_in.rotationMatrix(),
                     cameraSimilarity_worldToCamera_in.translation() /
                         cameraSimilarity_worldToCamera_in.scale());
    Eigen::Vector3f cameraCenter_world =
        cameraPose_worldToCamera.inverse().translation();

    /* Set of MapPoints already found in the KeyFrame */
    std::set<map_point::MapPoint *> alreadyFounds(matched_inout.begin(),
                                                  matched_inout.end());
    alreadyFounds.erase(static_cast<map_point::MapPoint *>(nullptr));

    int nmatches = 0;

    /* For each Candidate MapPoint Project and Match */
    for (int mapPointIndex = 0, iendMapPoint = points_in.size();
         mapPointIndex < iendMapPoint;
         mapPointIndex++)
    {
        map_point::MapPoint *p_mapPoint = points_in[mapPointIndex];

        /* Discard Bad MapPoints and already found */
        bool mapPointIsBad{};
        if (p_mapPoint->isBad(mapPointIsBad) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad || alreadyFounds.count(p_mapPoint))
            continue;

        /* Get 3D Coords. */
        Eigen::Vector3f p3Dw{};
        if (p_mapPoint->getWorldPos(p3Dw) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Transform into Camera Coords. */
        Eigen::Vector3f p3Dc = cameraPose_worldToCamera * p3Dw;

        /* Depth must be positive */
        if (p3Dc(2) < 0.0)
            continue;

        /* Project into Image */
        const Eigen::Vector2f uv = p_keyframe_in->p_camera->project(p3Dc);

        /* Point must be inside the image */
        bool pKFIsInImage{};
        if (p_keyframe_in->isInImage(uv(0), uv(1), pKFIsInImage) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInImage returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!pKFIsInImage)
            continue;

        /* Depth must be inside the scale invariance region of the point */
        float maximumDistance{};
        if (p_mapPoint->getMaxDistanceInvariance(maximumDistance) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMaxDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        float minimumDistance{};
        if (p_mapPoint->getMinDistanceInvariance(minimumDistance) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMinDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        Eigen::Vector3f PO       = p3Dw - cameraCenter_world;
        const float     distance = PO.norm();

        if (distance < minimumDistance || distance > maximumDistance)
            continue;

        /* Viewing angle must be less than 60 deg */
        Eigen::Vector3f Pn{};
        if (p_mapPoint->getNormal(Pn) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getNormal returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        if (PO.dot(Pn) < 0.5 * distance)
            continue;

        int predictedLevelCount{};
        if (p_mapPoint->predictScale(distance,
                                     p_keyframe_in,
                                     predictedLevelCount) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: predictScale returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Search in a radius */
        const float radius =
            threshold_in * p_keyframe_in->scaleFactors[predictedLevelCount];

        std::vector<size_t> indices{};
        if (p_keyframe_in->getFeaturesInArea(uv(0), uv(1), radius, indices) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFeaturesInArea returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (indices.empty())
            continue;

        /* Match to the most similar keypoint in the radius */
        cv::Mat mapPointDescriptor{};
        if (p_mapPoint->getDescriptor(mapPointDescriptor) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDescriptor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        int bestDistance = 256;
        int bestIndex    = -1;
        for (std::vector<size_t>::const_iterator vit  = indices.begin(),
                                                 vend = indices.end();
             vit != vend;
             vit++)
        {
            const size_t featureIndex = *vit;
            if (matched_inout[featureIndex])
                continue;

            const int &keyPointLevel =
                p_keyframe_in->keyPointsUndistorted[featureIndex].octave;

            if (keyPointLevel < predictedLevelCount - 1 ||
                keyPointLevel > predictedLevelCount)
                continue;

            const cv::Mat &keyFrameDescriptor =
                p_keyframe_in->descriptors.row(featureIndex);

            int descriptorDistance{};
            if (computeDescriptorDistance(mapPointDescriptor,
                                          keyFrameDescriptor,
                                          descriptorDistance) !=
                ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computeDescriptorDistance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (descriptorDistance < bestDistance)
            {
                bestDistance = descriptorDistance;
                bestIndex    = featureIndex;
            }
        }

        if (bestDistance <= TH_LOW * hammingRatio_in)
        {
            matched_inout[bestIndex] = p_mapPoint;
            nmatches++;
        }
    }

    byProjection_out = nmatches;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

ORBmatcherStatus ORBmatcher::searchByProjection(
    keyframe::KeyFrame                       *p_keyframe_in,
    Sophus::Sim3<float>                      &cameraSimilarity_worldToCamera_in,
    const std::vector<map_point::MapPoint *> &points_in,
    const std::vector<keyframe::KeyFrame *>  &pointsKeyframes_in,
    std::vector<map_point::MapPoint *>       &matched_inout,
    std::vector<keyframe::KeyFrame *>        &matchedKeyframes_inout,
    int                                       threshold_in,
    int                                      &byProjection_out,
    float                                     hammingRatio_in)
{
    /* Get Calibration Parameters for later projection */
    const float &fx = p_keyframe_in->fx;
    const float &fy = p_keyframe_in->fy;
    const float &cx = p_keyframe_in->cx;
    const float &cy = p_keyframe_in->cy;

    Sophus::SE3f cameraPose_worldToCamera =
        Sophus::SE3f(cameraSimilarity_worldToCamera_in.rotationMatrix(),
                     cameraSimilarity_worldToCamera_in.translation() /
                         cameraSimilarity_worldToCamera_in.scale());
    Eigen::Vector3f cameraCenter_world =
        cameraPose_worldToCamera.inverse().translation();

    /* Set of MapPoints already found in the KeyFrame */
    std::set<map_point::MapPoint *> alreadyFounds(matched_inout.begin(),
                                                  matched_inout.end());
    alreadyFounds.erase(static_cast<map_point::MapPoint *>(nullptr));

    int nmatches = 0;

    /* For each Candidate MapPoint Project and Match */
    for (int mapPointIndex = 0, iendMapPoint = points_in.size();
         mapPointIndex < iendMapPoint;
         mapPointIndex++)
    {
        map_point::MapPoint *p_mapPoint = points_in[mapPointIndex];
        keyframe::KeyFrame  *p_keyFrame = pointsKeyframes_in[mapPointIndex];

        /* Discard Bad MapPoints and already found */
        bool mapPointIsBad{};
        if (p_mapPoint->isBad(mapPointIsBad) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad || alreadyFounds.count(p_mapPoint))
            continue;

        /* Get 3D Coords. */
        Eigen::Vector3f p3Dw{};
        if (p_mapPoint->getWorldPos(p3Dw) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Transform into Camera Coords. */
        Eigen::Vector3f p3Dc = cameraPose_worldToCamera * p3Dw;

        /* Depth must be positive */
        if (p3Dc(2) < 0.0)
            continue;

        /* Project into Image */
        const float invz = 1 / p3Dc(2);
        const float x    = p3Dc(0) * invz;
        const float y    = p3Dc(1) * invz;

        const float u = fx * x + cx;
        const float v = fy * y + cy;

        /* Point must be inside the image */
        bool pKFIsInImage{};
        if (p_keyframe_in->isInImage(u, v, pKFIsInImage) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInImage returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!pKFIsInImage)
            continue;

        /* Depth must be inside the scale invariance region of the point */
        float maximumDistance{};
        if (p_mapPoint->getMaxDistanceInvariance(maximumDistance) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMaxDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        float minimumDistance{};
        if (p_mapPoint->getMinDistanceInvariance(minimumDistance) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMinDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        Eigen::Vector3f PO       = p3Dw - cameraCenter_world;
        const float     distance = PO.norm();

        if (distance < minimumDistance || distance > maximumDistance)
            continue;

        /* Viewing angle must be less than 60 deg */
        Eigen::Vector3f Pn{};
        if (p_mapPoint->getNormal(Pn) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getNormal returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        if (PO.dot(Pn) < 0.5 * distance)
            continue;

        int predictedLevelCount{};
        if (p_mapPoint->predictScale(distance,
                                     p_keyframe_in,
                                     predictedLevelCount) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: predictScale returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Search in a radius */
        const float radius =
            threshold_in * p_keyframe_in->scaleFactors[predictedLevelCount];

        std::vector<size_t> indices{};
        if (p_keyframe_in->getFeaturesInArea(u, v, radius, indices) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFeaturesInArea returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (indices.empty())
            continue;

        /* Match to the most similar keypoint in the radius */
        cv::Mat mapPointDescriptor{};
        if (p_mapPoint->getDescriptor(mapPointDescriptor) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDescriptor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        int bestDistance = 256;
        int bestIndex    = -1;
        for (std::vector<size_t>::const_iterator vit  = indices.begin(),
                                                 vend = indices.end();
             vit != vend;
             vit++)
        {
            const size_t featureIndex = *vit;
            if (matched_inout[featureIndex])
                continue;

            const int &keyPointLevel =
                p_keyframe_in->keyPointsUndistorted[featureIndex].octave;

            if (keyPointLevel < predictedLevelCount - 1 ||
                keyPointLevel > predictedLevelCount)
                continue;

            const cv::Mat &keyFrameDescriptor =
                p_keyframe_in->descriptors.row(featureIndex);

            int descriptorDistance{};
            if (computeDescriptorDistance(mapPointDescriptor,
                                          keyFrameDescriptor,
                                          descriptorDistance) !=
                ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computeDescriptorDistance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (descriptorDistance < bestDistance)
            {
                bestDistance = descriptorDistance;
                bestIndex    = featureIndex;
            }
        }

        if (bestDistance <= TH_LOW * hammingRatio_in)
        {
            matched_inout[bestIndex]          = p_mapPoint;
            matchedKeyframes_inout[bestIndex] = p_keyFrame;
            nmatches++;
        }
    }

    byProjection_out = nmatches;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

ORBmatcherStatus ORBmatcher::searchByProjection(
    frame::Frame                          &currentFrame_inout,
    keyframe::KeyFrame                    *p_keyframe_in,
    const std::set<map_point::MapPoint *> &alreadyFound_in,
    const float                            threshold_in,
    const int                              orbDistance_in,
    int                                   &byProjection_out)
{
    int nmatches = 0;

    Sophus::SE3f cameraPose_worldToCamera{};
    if (currentFrame_inout.getPose(cameraPose_worldToCamera) !=
        frame::FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector3f cameraCenter_world =
        cameraPose_worldToCamera.inverse().translation();

    /* Rotation Histogram (to check rotation consistency) */
    std::vector<int> rotHist[HISTO_LENGTH];
    for (int histogramBinIndex = 0; histogramBinIndex < HISTO_LENGTH;
         histogramBinIndex++)
        rotHist[histogramBinIndex].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    std::vector<map_point::MapPoint *> mapPoints{};
    if (p_keyframe_in->getMapPointMatches(mapPoints) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
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
        map_point::MapPoint *p_mapPoint = mapPoints[histogramBinIndex];

        if (p_mapPoint)
        {
            bool mapPointIsBad{};
            if (p_mapPoint->isBad(mapPointIsBad) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!mapPointIsBad && !alreadyFound_in.count(p_mapPoint))
            {
                /* Project */
                Eigen::Vector3f x3Dw{};
                if (p_mapPoint->getWorldPos(x3Dw) !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getWorldPos returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3f x3Dc = cameraPose_worldToCamera * x3Dw;

                const Eigen::Vector2f uv =
                    currentFrame_inout.p_camera->project(x3Dc);

                if (uv(0) < currentFrame_inout.gridMinX ||
                    uv(0) > currentFrame_inout.gridMaxX)
                    continue;
                if (uv(1) < currentFrame_inout.gridMinY ||
                    uv(1) > currentFrame_inout.gridMaxY)
                    continue;

                /* Compute predicted scale level */
                Eigen::Vector3f PO         = x3Dw - cameraCenter_world;
                float           distance3d = PO.norm();

                float maximumDistance{};
                if (p_mapPoint->getMaxDistanceInvariance(maximumDistance) !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMaxDistanceInvariance returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                float minimumDistance{};
                if (p_mapPoint->getMinDistanceInvariance(minimumDistance) !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMinDistanceInvariance returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }

                /* Depth must be inside the scale pyramid of the image */
                if (distance3d < minimumDistance ||
                    distance3d > maximumDistance)
                    continue;

                int predictedLevelCount{};
                if (p_mapPoint->predictScale(distance3d,
                                             &currentFrame_inout,
                                             predictedLevelCount) !=
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: predictScale returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }

                /* Search in a window */
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
                    frame::FrameStatus::FRAME_STATUS_SUCCESS)
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
                    map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
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

} /* namespace orb_matcher */
} /* namespace core */
} /* namespace vs_graphs */
