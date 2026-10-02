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
 * @file            withSim3Replacement.cc
 *
 * @brief           Implements ORBmatcher::fuse() (withSim3Replacement),
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

ORBmatcherStatus ORBmatcher::fuse(KeyFrame      *p_keyframe_inout,
                                  Sophus::Sim3f &similarity_worldToCamera_in,
                                  const std::vector<MapPoint *> &points_in,
                                  float                          threshold_in,
                                  std::vector<MapPoint *> &replacePoints_inout,
                                  int                     &fusedCount_out)
{
    // Decompose Scw
    Sophus::SE3f pose_worldToCamera =
        Sophus::SE3f(similarity_worldToCamera_in.rotationMatrix(),
                     similarity_worldToCamera_in.translation() /
                         similarity_worldToCamera_in.scale());
    Eigen::Vector3f cameraCenter_World =
        pose_worldToCamera.inverse().translation();

    // Set of MapPoints already found in the KeyFrame
    std::set<MapPoint *> alreadyFounds{};
    if (p_keyframe_inout->getMapPoints(alreadyFounds) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapPoints returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    int fusedCount = 0;

    const int pointCount = points_in.size();

    // For each candidate MapPoint project and match
    for (int mapPointIndex = 0; mapPointIndex < pointCount; mapPointIndex++)
    {
        MapPoint *p_mapPoint = points_in[mapPointIndex];

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
        Eigen::Vector3f p3Dc = pose_worldToCamera * p3Dw;

        // Depth must be positive
        if (p3Dc(2) < 0.0f)
            continue;

        // Project into Image
        const Eigen::Vector2f uv = p_keyframe_inout->p_camera->project(p3Dc);

        // Point must be inside the image
        bool keyframeIsInImage{};
        if (p_keyframe_inout->isInImage(uv(0), uv(1), keyframeIsInImage) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isInImage returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!keyframeIsInImage)
            continue;

        // Depth must be inside the scale pyramid of the image
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
        Eigen::Vector3f PO         = p3Dw - cameraCenter_World;
        const float     distance3d = PO.norm();

        if (distance3d < minimumDistance || distance3d > maximumDistance)
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

        if (PO.dot(Pn) < 0.5 * distance3d)
            continue;

        // Compute predicted scale level
        int predictedLevelCount{};
        if (p_mapPoint->predictScale(distance3d,
                                     p_keyframe_inout,
                                     predictedLevelCount) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: predictScale returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        // Search in a radius
        const float radius =
            threshold_in * p_keyframe_inout->scaleFactors[predictedLevelCount];

        std::vector<size_t> indices{};
        if (p_keyframe_inout->getFeaturesInArea(uv(0),
                                                uv(1),
                                                radius,
                                                indices) !=
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
        for (std::vector<size_t>::const_iterator vit = indices.begin();
             vit != indices.end();
             vit++)
        {
            const size_t featureIndex = *vit;
            const int   &keyPointLevel =
                p_keyframe_inout->keyPointsUndistorted[featureIndex].octave;

            if (keyPointLevel < predictedLevelCount - 1 ||
                keyPointLevel > predictedLevelCount)
                continue;

            const cv::Mat &keyFrameDescriptor =
                p_keyframe_inout->descriptors.row(featureIndex);

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

        // If there is already a MapPoint replace otherwise add new measurement
        if (bestDistance <= TH_LOW)
        {
            MapPoint *p_keyFrameMapPoint = nullptr;
            if (p_keyframe_inout->getMapPoint(bestIndex, p_keyFrameMapPoint) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_keyFrameMapPoint)
            {
                bool keyFrameMapPointIsBad{};
                if (p_keyFrameMapPoint->isBad(keyFrameMapPointIsBad) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!keyFrameMapPointIsBad)
                    replacePoints_inout[mapPointIndex] = p_keyFrameMapPoint;
            }
            else
            {
                if (p_mapPoint->addObservation(p_keyframe_inout, bestIndex) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addObservation returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_keyframe_inout->addMapPoint(p_mapPoint, bestIndex) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addMapPoint returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
            fusedCount++;
        }
    }

    fusedCount_out = fusedCount;
    return ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
