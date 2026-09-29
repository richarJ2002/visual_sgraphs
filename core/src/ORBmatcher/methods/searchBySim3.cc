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
    ORBmatcher::searchBySim3(KeyFrame                *pKF1,
                             KeyFrame                *pKF2,
                             std::vector<MapPoint *> &matches12_inout,
                             const Sophus::Sim3f     &S12,
                             const float              th,
                             int                     &bySim3_out)
{
    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    // Camera 1 & 2 from world
    Sophus::SE3f T1w{};
    if (pKF1->getPose(T1w) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f T2w{};
    if (pKF2->getPose(T2w) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    // Transformation between cameras
    Sophus::Sim3f S21 = S12.inverse();

    std::vector<MapPoint *> mapPoints1{};
    if (pKF1->getMapPointMatches(mapPoints1) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const int N1 = mapPoints1.size();

    std::vector<MapPoint *> mapPoints2{};
    if (pKF2->getMapPointMatches(mapPoints2) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPointMatches returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const int N2 = mapPoints2.size();

    vector<bool> alreadyMatched1Flags(N1, false);
    vector<bool> alreadyMatched2Flags(N2, false);

    for (int keyPointIndex1 = 0; keyPointIndex1 < N1; keyPointIndex1++)
    {
        MapPoint *p_mapPoint = matches12_inout[keyPointIndex1];
        if (p_mapPoint)
        {
            alreadyMatched1Flags[keyPointIndex1] = true;
            std::tuple<int, int> mapPointIndexInKeyFrame{};
            if (p_mapPoint->getIndexInKeyFrame(pKF2, mapPointIndexInKeyFrame) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getIndexInKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int index2 = get<0>(mapPointIndexInKeyFrame);
            if (index2 >= 0 && index2 < N2)
                alreadyMatched2Flags[index2] = true;
        }
    }

    vector<int> matchIndices1(N1, -1);
    vector<int> matchIndices2(N2, -1);

    // Transform from KF1 to KF2 and search
    for (int i1 = 0; i1 < N1; i1++)
    {
        MapPoint *p_mapPoint = mapPoints1[i1];

        if (!p_mapPoint || alreadyMatched1Flags[i1])
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

        Eigen::Vector3f p3Dw{};
        if (p_mapPoint->getWorldPos(p3Dw) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f p3Dc1 = T1w * p3Dw;
        Eigen::Vector3f p3Dc2 = S21 * p3Dc1;

        // Depth must be positive
        if (p3Dc2(2) < 0.0)
            continue;

        const float invz = 1.0 / p3Dc2(2);
        const float x    = p3Dc2(0) * invz;
        const float y    = p3Dc2(1) * invz;

        const float u = fx * x + cx;
        const float v = fy * y + cy;

        // Point must be inside the image
        bool pKF2IsInImage{};
        if (pKF2->isInImage(u, v, pKF2IsInImage) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInImage returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!pKF2IsInImage)
            continue;

        float maximumDistance{};
        if (p_mapPoint->getMaxDistanceInvariance(maximumDistance) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMaxDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        float minimumDistance{};
        if (p_mapPoint->getMinDistanceInvariance(minimumDistance) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMinDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        const float distance3d = p3Dc2.norm();

        // Depth must be inside the scale invariance region
        if (distance3d < minimumDistance || distance3d > maximumDistance)
            continue;

        // Compute predicted octave
        int predictedLevelCount{};
        if (p_mapPoint->predictScale(distance3d, pKF2, predictedLevelCount) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: predictScale returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        // Search in a radius
        const float radius = th * pKF2->scaleFactors[predictedLevelCount];

        std::vector<size_t> indices{};
        if (pKF2->getFeaturesInArea(u, v, radius, indices) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFeaturesInArea returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (indices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        cv::Mat mapPointDescriptor{};
        if (p_mapPoint->getDescriptor(mapPointDescriptor) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDescriptor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        int bestDistance = INT_MAX;
        int bestIndex    = -1;
        for (vector<size_t>::const_iterator vit  = indices.begin(),
                                            vend = indices.end();
             vit != vend;
             vit++)
        {
            const size_t featureIndex = *vit;

            const cv::KeyPoint &keyPoint =
                pKF2->keyPointsUndistorted[featureIndex];

            if (keyPoint.octave < predictedLevelCount - 1 ||
                keyPoint.octave > predictedLevelCount)
                continue;

            const cv::Mat &keyFrameDescriptor =
                pKF2->descriptors.row(featureIndex);

            int distance{};
            if (computeDescriptorDistance(mapPointDescriptor,
                                          keyFrameDescriptor,
                                          distance) !=
                ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computeDescriptorDistance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex    = featureIndex;
            }
        }

        if (bestDistance <= TH_HIGH)
        {
            matchIndices1[i1] = bestIndex;
        }
    }

    // Transform from KF2 to KF2 and search
    for (int i2 = 0; i2 < N2; i2++)
    {
        MapPoint *p_mapPoint = mapPoints2[i2];

        if (!p_mapPoint || alreadyMatched2Flags[i2])
            continue;

        bool mapPointIsBad2{};
        if (p_mapPoint->isBad(mapPointIsBad2) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad2)
            continue;

        Eigen::Vector3f p3Dw{};
        if (p_mapPoint->getWorldPos(p3Dw) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f p3Dc2 = T2w * p3Dw;
        Eigen::Vector3f p3Dc1 = S12 * p3Dc2;

        // Depth must be positive
        if (p3Dc1(2) < 0.0)
            continue;

        const float invz = 1.0 / p3Dc1(2);
        const float x    = p3Dc1(0) * invz;
        const float y    = p3Dc1(1) * invz;

        const float u = fx * x + cx;
        const float v = fy * y + cy;

        // Point must be inside the image
        bool pKF1IsInImage{};
        if (pKF1->isInImage(u, v, pKF1IsInImage) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInImage returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!pKF1IsInImage)
            continue;

        float maximumDistance{};
        if (p_mapPoint->getMaxDistanceInvariance(maximumDistance) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMaxDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        float minimumDistance{};
        if (p_mapPoint->getMinDistanceInvariance(minimumDistance) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getMinDistanceInvariance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        const float distance3d = p3Dc1.norm();

        // Depth must be inside the scale pyramid of the image
        if (distance3d < minimumDistance || distance3d > maximumDistance)
            continue;

        // Compute predicted octave
        int predictedLevelCount{};
        if (p_mapPoint->predictScale(distance3d, pKF1, predictedLevelCount) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: predictScale returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th * pKF1->scaleFactors[predictedLevelCount];

        std::vector<size_t> indices{};
        if (pKF1->getFeaturesInArea(u, v, radius, indices) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getFeaturesInArea returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        if (indices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        cv::Mat mapPointDescriptor{};
        if (p_mapPoint->getDescriptor(mapPointDescriptor) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getDescriptor returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        int bestDistance = INT_MAX;
        int bestIndex    = -1;
        for (vector<size_t>::const_iterator vit  = indices.begin(),
                                            vend = indices.end();
             vit != vend;
             vit++)
        {
            const size_t featureIndex = *vit;

            const cv::KeyPoint &keyPoint =
                pKF1->keyPointsUndistorted[featureIndex];

            if (keyPoint.octave < predictedLevelCount - 1 ||
                keyPoint.octave > predictedLevelCount)
                continue;

            const cv::Mat &keyFrameDescriptor =
                pKF1->descriptors.row(featureIndex);

            int distance{};
            if (computeDescriptorDistance(mapPointDescriptor,
                                          keyFrameDescriptor,
                                          distance) !=
                ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: computeDescriptorDistance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }

            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex    = featureIndex;
            }
        }

        if (bestDistance <= TH_HIGH)
        {
            matchIndices2[i2] = bestIndex;
        }
    }

    // Check agreement
    int foundCount = 0;

    for (int i1 = 0; i1 < N1; i1++)
    {
        int index2 = matchIndices1[i1];

        if (index2 >= 0)
        {
            int index1 = matchIndices2[index2];
            if (index1 == i1)
            {
                matches12_inout[i1] = mapPoints2[index2];
                foundCount++;
            }
        }
    }

    bySim3_out = foundCount;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
