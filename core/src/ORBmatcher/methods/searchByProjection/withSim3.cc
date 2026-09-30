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
    ORBmatcher::searchByProjection(KeyFrame                      *pKF,
                                   Sophus::Sim3f                 &Scw,
                                   const std::vector<MapPoint *> &vpPoints,
                                   std::vector<MapPoint *>       &matched_inout,
                                   int                            th,
                                   int  &byProjection_out,
                                   float ratioHamming)
{
    Sophus::SE3f Tcw =
        Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    // Set of MapPoints already found in the KeyFrame
    std::set<MapPoint *> alreadyFounds(matched_inout.begin(),
                                       matched_inout.end());
    alreadyFounds.erase(static_cast<MapPoint *>(nullptr));

    int nmatches = 0;

    // For each Candidate MapPoint Project and Match
    for (int mapPointIndex = 0, iendMapPoint = vpPoints.size();
         mapPointIndex < iendMapPoint;
         mapPointIndex++)
    {
        MapPoint *p_mapPoint = vpPoints[mapPointIndex];

        // Discard Bad MapPoints and already found
        bool mapPointIsBad{};
        if (p_mapPoint->isBad(mapPointIsBad) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (mapPointIsBad || alreadyFounds.count(p_mapPoint))
            continue;

        // Get 3D Coords.
        Eigen::Vector3f p3Dw{};
        if (p_mapPoint->getWorldPos(p3Dw) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        // Transform into Camera Coords.
        Eigen::Vector3f p3Dc = Tcw * p3Dw;

        // Depth must be positive
        if (p3Dc(2) < 0.0)
            continue;

        // Project into Image
        const Eigen::Vector2f uv = pKF->p_camera->project(p3Dc);

        // Point must be inside the image
        bool pKFIsInImage{};
        if (pKF->isInImage(uv(0), uv(1), pKFIsInImage) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInImage returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!pKFIsInImage)
            continue;

        // Depth must be inside the scale invariance region of the point
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
        Eigen::Vector3f PO       = p3Dw - Ow;
        const float     distance = PO.norm();

        if (distance < minimumDistance || distance > maximumDistance)
            continue;

        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn{};
        if (p_mapPoint->getNormal(Pn) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getNormal returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        if (PO.dot(Pn) < 0.5 * distance)
            continue;

        int predictedLevelCount{};
        if (p_mapPoint->predictScale(distance, pKF, predictedLevelCount) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: predictScale returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        // Search in a radius
        const float radius = th * pKF->scaleFactors[predictedLevelCount];

        std::vector<size_t> indices{};
        if (pKF->getFeaturesInArea(uv(0), uv(1), radius, indices) !=
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
                pKF->keyPointsUndistorted[featureIndex].octave;

            if (keyPointLevel < predictedLevelCount - 1 ||
                keyPointLevel > predictedLevelCount)
                continue;

            const cv::Mat &keyFrameDescriptor =
                pKF->descriptors.row(featureIndex);

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

        if (bestDistance <= TH_LOW * ratioHamming)
        {
            matched_inout[bestIndex] = p_mapPoint;
            nmatches++;
        }
    }

    byProjection_out = nmatches;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
